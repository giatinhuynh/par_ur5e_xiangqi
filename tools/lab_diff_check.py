#!/usr/bin/env python3
import hashlib
import paramiko
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CHECK = [
    "workspace/src/xiangqi_ai/xiangqi_ai/game_manager_node.py",
    "workspace/src/xiangqi_ai/xiangqi_ai/ai_engine_node.py",
    "workspace/src/xiangqi_ai/xiangqi_ai/minimax_engine.py",
    "workspace/src/xiangqi_msgs/srv/GetBestMove.srv",
    "tools/lab_sync_rebuild.py",
    "tools/lab_colcon_build.py",
]


def md5(path: Path) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> None:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect("10.234.7.84", username="vxlab", password="welc0me", timeout=15)
    for rel in CHECK:
        local = REPO / rel
        if not local.exists():
            print(f"LOCAL_ONLY  {rel}")
            continue
        lh = md5(local)
        _, o, _ = c.exec_command(f"md5sum ~/par_ur5e_xiangqi/{rel} 2>/dev/null || echo MISSING")
        line = o.read().decode().strip()
        if not line or line == "MISSING" or "No such file" in line:
            print(f"REMOTE_MISS {rel}")
            continue
        rh = line.split()[0]
        print(f"{'MATCH' if lh == rh else 'DIFF':11} {rel}")
    c.close()


if __name__ == "__main__":
    main()
