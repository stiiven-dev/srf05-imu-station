import re, statistics, sys

trig, jitter, period = [], [], []
for line in open(sys.argv[1]):
    m = re.search(r"trig_us=(\d+) wake_jitter_us=(\d+) period_us=(\d+)", line)
    if m:
        trig.append(int(m[1]))
        jitter.append(int(m[2]))
        period.append(int(m[3]))

trig, jitter, period = trig[5:], jitter[5:], period[5:]  # drop warm-up

def stats(name, xs):
    print(f"{name:16} n={len(xs)} mean={statistics.mean(xs):.1f} "
          f"sd={statistics.stdev(xs):.2f} min={min(xs)} max={max(xs)}")

stats("trig_pulse_us", trig)
stats("wake_jitter_us", jitter)
stats("period_us", period)