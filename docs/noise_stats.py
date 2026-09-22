import re, statistics, sys

raw, filt = [], []
for line in open(sys.argv[1]):
    m = re.search(r"raw=(\d+) filt=(\d+)", line)
    if m:
        raw.append(int(m[1]))
        filt.append(int(m[2]))

raw, filt = raw[5:], filt[5:]              # drop the warm-up samples
dropouts = sum(1 for x in raw if x == 65535)

def stats(name, xs):
    xs = [x for x in xs if x != 65535]     # dropouts are counted separately
    print(f"{name:9} n={len(xs)} mean={statistics.mean(xs):.1f} "
          f"sd={statistics.stdev(xs):.2f} min={min(xs)} max={max(xs)}")

stats("raw", raw)
stats("filtered", filt)
print(f"raw dropouts: {dropouts}, filtered dropouts: {sum(1 for x in filt if x == 65535)}")