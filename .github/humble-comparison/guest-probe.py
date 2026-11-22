import json
import os
import resource
import sys
import time
from pathlib import Path

sources = (
    Path("/sys/fs/cgroup/cpu.stat"),
    Path("/sys/fs/cgroup/memory.events"),
    Path("/proc/pressure/cpu"),
    Path("/proc/pressure/memory"),
    Path("/proc/pressure/io"),
    Path("/proc/stat"),
)
print(json.dumps(("采样列", "实时", "单调", "单调原始", "启动时间", "进程CPU", "自愿切换", "非自愿切换")))
print(json.dumps(("环境", sys.argv[1], os.uname().release, sorted(os.sched_getaffinity(0)))))
cycle = 0
while True:
    started = time.monotonic_ns()
    now = time.time_ns()
    stat = resource.getrusage(resource.RUSAGE_SELF)
    print(json.dumps(("采样", now, started, time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                      time.clock_gettime_ns(time.CLOCK_BOOTTIME), time.process_time_ns(),
                      stat.ru_nvcsw, stat.ru_nivcsw), separators=(",", ":")), flush=True)
    if cycle % 10 == 0:
        for path in sources:
            if not path.is_file():
                continue
            data = path.read_text().splitlines()
            if path == sources[-1]:
                data = data[:1]
            print(json.dumps(("计数器", now, str(path), data), separators=(",", ":")), flush=True)
    cycle += 1
    time.sleep(max(0, (10_000_000 - (time.monotonic_ns() - started)) / 1e9))
