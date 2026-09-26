//! Flash-backed persistence for IMU calibration data.
//!
//! Reserves the last 8KB of flash (two 4KB sectors — see `memory.x`'s
//! `STORAGE` region) as a `sequential-storage` key-value store, so a
//! calibration only has to be performed once and survives power cycles.
//!
//! # Safety
//! `FlashStorage::erase`/`write` call into `rp2040_flash`, which copies its
//! own code into RAM before touching flash — required because the RP2040
//! executes code directly from flash (XIP) and cannot erase/program the
//! chip it's currently fetching instructions from. `critical_section::with`
//! disables interrupts on this core for the duration. This project is
//! single-core; if a second core is ever brought up, it must also be halted
//! during any flash write, or it will fault trying to fetch from flash
//! mid-erase.

use core::ptr;
use embedded_storage_async::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};
use motion_core::{Calibration, RawSample3};
use sequential_storage::{
    cache::Cache,
    map::{MapConfig, MapStorage, SerializationError, Value},
};

/// Offset of the reserved STORAGE region within the flash chip's address
/// space (bytes from the start of flash, i.e. from 0x10000000).
/// Must match STORAGE's ORIGIN in memory.x: ORIGIN(STORAGE) - 0x10000000.
const STORAGE_OFFSET: u32 = 0x1FE000;
/// Must match STORAGE's LENGTH in memory.x.
const STORAGE_SIZE: u32 = 0x2000;

const CAL_KEY: u8 = 1;

// ---------------------------------------------------------------------
// NorFlash implementation backing sequential-storage
// ---------------------------------------------------------------------

pub struct FlashStorage;

#[derive(Debug)]
pub struct FlashError;

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

impl ErrorType for FlashStorage {
    type Error = FlashError;
}

impl ReadNorFlash for FlashStorage {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        // Flash is memory-mapped for reads: XIP base + flash offset.
        let addr = (0x10000000 + STORAGE_OFFSET + offset) as *const u8;
        unsafe { ptr::copy_nonoverlapping(addr, bytes.as_mut_ptr(), bytes.len()) };
        Ok(())
    }

    fn capacity(&self) -> usize {
        STORAGE_SIZE as usize
    }
}

impl NorFlash for FlashStorage {
    // Report a small write granularity to satisfy sequential-storage's
    // internal MAX_WORD_SIZE cap (32 bytes) — the RP2040's actual flash
    // program unit is a 256-byte page, which write() below handles by
    // reading the current page, overlaying the new bytes, and
    // reprogramming the whole page. This is what makes a small WRITE_SIZE
    // truthful: sequential-storage can call write() with small, oddly
    // aligned chunks and still get a correct result.
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        critical_section::with(|_cs| unsafe {
            rp2040_flash::flash::flash_range_erase(STORAGE_OFFSET + from, to - from, true);
        });
        Ok(())
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        const PAGE: u32 = 256; // RP2040's real flash program granularity

        let mut pos = offset;
        let mut remaining = bytes;

        while !remaining.is_empty() {
            let page_start = pos - (pos % PAGE);
            let page_offset = (pos - page_start) as usize;
            let chunk_len = (PAGE as usize - page_offset).min(remaining.len());

            // Read the page's current contents so bytes outside our slice
            // are reprogrammed unchanged rather than clobbered.
            let mut page_buf = [0u8; PAGE as usize];
            let addr = (0x10000000 + STORAGE_OFFSET + page_start) as *const u8;
            unsafe { ptr::copy_nonoverlapping(addr, page_buf.as_mut_ptr(), PAGE as usize) };

            page_buf[page_offset..page_offset + chunk_len].copy_from_slice(&remaining[..chunk_len]);

            critical_section::with(|_cs| unsafe {
                rp2040_flash::flash::flash_range_program(
                    STORAGE_OFFSET + page_start,
                    &page_buf,
                    true,
                );
            });

            pos += chunk_len as u32;
            remaining = &remaining[chunk_len..];
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Calibration <-> bytes, and the sequential-storage Value impl
// ---------------------------------------------------------------------

/// 12-byte encoding of a `Calibration` (six big-endian i16 fields).
/// `motion-core` stays free of any dependency on the storage crate — this
/// wrapper is the only place the two are coupled.
struct CalibrationBytes([u8; 12]);

impl From<&Calibration> for CalibrationBytes {
    fn from(cal: &Calibration) -> Self {
        let mut b = [0u8; 12];
        b[0..2].copy_from_slice(&cal.gyro_bias.x.to_be_bytes());
        b[2..4].copy_from_slice(&cal.gyro_bias.y.to_be_bytes());
        b[4..6].copy_from_slice(&cal.gyro_bias.z.to_be_bytes());
        b[6..8].copy_from_slice(&cal.accel_offset.x.to_be_bytes());
        b[8..10].copy_from_slice(&cal.accel_offset.y.to_be_bytes());
        b[10..12].copy_from_slice(&cal.accel_offset.z.to_be_bytes());
        Self(b)
    }
}

impl From<CalibrationBytes> for Calibration {
    fn from(cb: CalibrationBytes) -> Self {
        let b = cb.0;
        Calibration {
            gyro_bias: RawSample3 {
                x: i16::from_be_bytes([b[0], b[1]]),
                y: i16::from_be_bytes([b[2], b[3]]),
                z: i16::from_be_bytes([b[4], b[5]]),
            },
            accel_offset: RawSample3 {
                x: i16::from_be_bytes([b[6], b[7]]),
                y: i16::from_be_bytes([b[8], b[9]]),
                z: i16::from_be_bytes([b[10], b[11]]),
            },
        }
    }
}

impl<'a> Value<'a> for CalibrationBytes {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 12 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[..12].copy_from_slice(&self.0);
        Ok(12)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 12 {
            return Err(SerializationError::InvalidFormat);
        }
        let mut b = [0u8; 12];
        b.copy_from_slice(&buffer[..12]);
        Ok((Self(b), 12))
    }
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

pub async fn store_calibration(cal: &Calibration) {
    let mut storage = MapStorage::<u8, _, _>::new(
        FlashStorage,
        const { MapConfig::new(0..STORAGE_SIZE) },
        Cache::new_uncached(),
    );
    let mut buf = [0u8; 32];
    let value = CalibrationBytes::from(cal);
    storage.store_item(&mut buf, &CAL_KEY, &value).await.ok();
}

pub async fn load_calibration() -> Option<Calibration> {
    let mut storage = MapStorage::<u8, _, _>::new(
        FlashStorage,
        const { MapConfig::new(0..STORAGE_SIZE) },
        Cache::new_uncached(),
    );
    let mut buf = [0u8; 32];
    let cb: CalibrationBytes = storage.fetch_item(&mut buf, &CAL_KEY).await.ok()??;
    Some(cb.into())
}
