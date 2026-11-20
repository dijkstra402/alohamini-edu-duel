"""交换由记录机本地单调时钟标记的实际故障事件。"""

import json
import os
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory

STOP_DEADLINE_S = 0.5


def recorder_boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()


@dataclass(frozen=True)
class FaultEvent:
    triggered_ns: int
    source: str

    @property
    def triggered_at(self):
        return self.triggered_ns / 1e9

    @classmethod
    def read(cls, path, boot_id):
        evidence = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(evidence, dict) or evidence.get("schema_version") != 1:
            raise ValueError("故障标记需要使用第 1 版格式")
        if evidence.get("boot_id") != boot_id:
            raise ValueError("故障标记与记录机的本次启动不一致")
        triggered_ns, source = evidence.get("triggered_ns"), evidence.get("source")
        if type(triggered_ns) is not int or not 0 < triggered_ns < 1 << 63:
            raise ValueError("故障标记缺少有效的本机单调时间")
        if not isinstance(source, str) or not source.strip():
            raise ValueError("故障标记缺少来源")
        return cls(triggered_ns, source.strip())

    def write(self, path, boot_id):
        target = Path(path)
        evidence = {
            "schema_version": 1, "boot_id": boot_id,
            "triggered_ns": self.triggered_ns, "source": self.source,
        }
        # 文件完整写入后再创建目标路径，避免观察器读取到一半的内容。
        with TemporaryDirectory(dir=target.parent) as directory:
            staging = Path(directory) / "fault_event.json"
            staging.write_text(json.dumps(evidence, ensure_ascii=False) + "\n", encoding="utf-8")
            os.link(staging, target)
