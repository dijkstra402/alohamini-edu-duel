#!/usr/bin/env bash
set -eo pipefail
source /runner/build-copy.sh
source /runner/build-install.sh
cd /checkout
python3 - <<'PY'
import os
print("容器可用 CPU", sorted(os.sched_getaffinity(0)), flush=True)
PY
python3 /runner/guest-probe.py 无宿主挂载 > /verify/scheduling.jsonl &
probe=$!
trap 'kill "$probe" 2>/dev/null || true' EXIT
status=0
for run in 1 2; do
    mkdir "/verify/full-$run"
    python3 -m pytest -v -rs -p no:cacheprovider \
        --basetemp "/verify/full-$run/pytest" --junitxml "/verify/full-$run/pytest.xml" \
        ros2/src/alohamini_duel/test | tee "/verify/full-$run/pytest.log" || status=1
    python3 - "/verify/full-$run/pytest.xml" <<'PY' || status=1
import sys
import xml.etree.ElementTree as ET
suite = ET.parse(sys.argv[1]).find("testsuite")
keys = ("tests", "failures", "errors", "skipped")
counts = {name: int(suite.attrib[name]) for name in keys}
print(counts, flush=True)
assert counts["tests"] and not any(counts[name] for name in keys[1:])
PY
done
for scenario in nav2 no-nav2 round; do
    args=()
    if [[ "$scenario" == no-nav2 ]]; then args+=(--no-nav2); fi
    if [[ "$scenario" == round ]]; then args+=(--round); fi
    ros2 run alohamini_duel duel_accept_sim "${args[@]}" --output "/verify/$scenario" || status=1
done
exit "$status"
