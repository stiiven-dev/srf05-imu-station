use embedded_hal::i2c::I2c;
use motion_core::RawSample3;

const ADDR: u8 = 0x68;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_INT_PIN_CFG: u8 = 0x37;
const REG_INT_ENABLE: u8 = 0x38;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_WHO_AM_I: u8 = 0x75;
const REG_CONFIG: u8 = 0x1A;
const REG_SMPLRT_DIV: u8 = 0x19;

#[derive(Debug, defmt::Format)]
pub enum Error<E> {
    I2c(E),
    UnexpectedWhoAmI(u8),
}

/// Wake the chip, select ±250°/s and ±2g ranges, and enable data-ready IRQ.
pub fn init<I2C, E>(i2c: &mut I2C) -> Result<(), Error<E>>
where
    I2C: I2c<Error = E>,
{
    let mut who = [0u8];
    i2c.write_read(ADDR, &[REG_WHO_AM_I], &mut who)
        .map_err(Error::I2c)?;
    if who[0] != 0x68 {
        return Err(Error::UnexpectedWhoAmI(who[0]));
    }

    i2c.write(ADDR, &[REG_PWR_MGMT_1, 0x00])
        .map_err(Error::I2c)?;
    i2c.write(ADDR, &[REG_GYRO_CONFIG, 0x00])
        .map_err(Error::I2c)?;
    i2c.write(ADDR, &[REG_ACCEL_CONFIG, 0x00])
        .map_err(Error::I2c)?;
    i2c.write(ADDR, &[REG_CONFIG, 0x03]).map_err(Error::I2c)?;
    i2c.write(ADDR, &[REG_SMPLRT_DIV, 0x09])
        .map_err(Error::I2c)?;
    i2c.write(ADDR, &[REG_INT_PIN_CFG, 0x00])
        .map_err(Error::I2c)?;
    i2c.write(ADDR, &[REG_INT_ENABLE, 0x01])
        .map_err(Error::I2c)?;
    Ok(())
}

/// Burst-read accel + gyro (skipping the temperature bytes in between).
pub fn read<I2C, E>(i2c: &mut I2C) -> Result<(RawSample3, RawSample3), Error<E>>
where
    I2C: I2c<Error = E>,
{
    let mut buf = [0u8; 14];
    i2c.write_read(ADDR, &[REG_ACCEL_XOUT_H], &mut buf)
        .map_err(Error::I2c)?;

    let accel = RawSample3 {
        x: i16::from_be_bytes([buf[0], buf[1]]),
        y: i16::from_be_bytes([buf[2], buf[3]]),
        z: i16::from_be_bytes([buf[4], buf[5]]),
    };
    let gyro = RawSample3 {
        x: i16::from_be_bytes([buf[8], buf[9]]),
        y: i16::from_be_bytes([buf[10], buf[11]]),
        z: i16::from_be_bytes([buf[12], buf[13]]),
    };
    Ok((accel, gyro))
}
