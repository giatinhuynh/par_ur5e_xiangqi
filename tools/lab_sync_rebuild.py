#!/usr/bin/env python3
"""Sync par_ur5e_xiangqi to the VXLab dev box and colcon build inside Docker."""
from __future__ import annotations

import argparse
import os
import sys
import tarfile
import tempfile
import time
from pathlib import Path

import paramiko

REPO = Path(__file__).resolve().parents[1]
REMOTE_REPO = "~/par_ur5e_xiangqi"
REMOTE_UR5E = "~/UR5e_Env"
CONTAINER = "ros2"

EXCLUDE_DIRS = {
    ".git",
    "__pycache__",
    ".pytest_cache",
    "node_modules",
    ".cursor",
    "build",
    "install",
    "log",
    "chess_dataset",
    "UR5e_Env-main",
}
EXCLUDE_SUFFIXES = {".pyc", ".pyo"}


def log(msg: str) -> None:
    print(msg, flush=True)


def should_skip(rel: Path) -> bool:
    parts = set(rel.parts)
    if parts & EXCLUDE_DIRS:
        return True
    if rel.suffix in EXCLUDE_SUFFIXES:
        return True
    return False


def make_tarball(repo: Path) -> Path:
    fd, tmp = tempfile.mkstemp(suffix=".tar.gz")
    os.close(fd)
    tmp_path = Path(tmp)
    log(f"Creating tarball from {repo} ...")
    count = 0
    with tarfile.open(tmp_path, "w:gz") as tar:
        for root, dirs, files in os.walk(repo):
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            root_path = Path(root)
            for name in files:
                full = root_path / name
                rel = full.relative_to(repo)
                if should_skip(rel):
                    continue
                tar.add(full, arcname=str(rel).replace("\\", "/"))
                count += 1
    size_mb = tmp_path.stat().st_size / (1024 * 1024)
    log(f"Tarball ready: {count} files, {size_mb:.1f} MB")
    return tmp_path


def run(client: paramiko.SSHClient, cmd: str, timeout: int = 3600) -> tuple[int, str, str]:
    log(f"\n>>> {cmd}")
    _, stdout, stderr = client.exec_command(cmd, timeout=timeout, get_pty=True)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    exit_status = stdout.channel.recv_exit_status()
    if out.strip():
        print(out, end="" if out.endswith("\n") else "\n")
    if err.strip() and exit_status != 0:
        print(err, file=sys.stderr, end="" if err.endswith("\n") else "\n")
    return exit_status, out, err


def sync_and_build(host: str, user: str, password: str) -> int:
    tarball = make_tarball(REPO)
    remote_tar = f"/tmp/par_ur5e_xiangqi_sync_{int(time.time())}.tar.gz"

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    log(f"Connecting to {user}@{host} ...")
    client.connect(host, username=user, password=password, timeout=30)

    try:
        sftp = client.open_sftp()
        log(f"Uploading to {remote_tar} ...")
        sftp.put(str(tarball), remote_tar)
        sftp.close()
        tarball.unlink(missing_ok=True)

        cmds = [
            f"mkdir -p {REMOTE_REPO}",
            f"tar -xzf {remote_tar} -C {REMOTE_REPO}",
            f"rm -f {remote_tar}",
            # Windows checkouts may carry CRLF; bash on Linux rejects `set -o pipefail\r`.
            f"find {REMOTE_REPO} -name '*.sh' -exec sed -i 's/\\r$//' {{}} +",
            f"mkdir -p {REMOTE_UR5E}/workspace/src",
            f"rsync -a --delete {REMOTE_REPO}/workspace/src/ {REMOTE_UR5E}/workspace/src/",
            f"mkdir -p {REMOTE_UR5E}/workspace/tools {REMOTE_UR5E}/workspace/models {REMOTE_UR5E}/workspace/config",
            f"rsync -a {REMOTE_REPO}/tools/ {REMOTE_UR5E}/workspace/tools/",
            f"rsync -a {REMOTE_REPO}/workspace/models/ {REMOTE_UR5E}/workspace/models/ 2>/dev/null || true",
            f"rsync -a {REMOTE_REPO}/workspace/config/ {REMOTE_UR5E}/workspace/config/ 2>/dev/null || true",
            f"chmod +x {REMOTE_REPO}/tools/*.sh {REMOTE_REPO}/tools/ur5e_env/*.sh 2>/dev/null || true",
        ]
        for cmd in cmds:
            code, _, err = run(client, cmd, timeout=600)
            if code != 0:
                log(f"ERROR: command failed ({code}): {cmd}\n{err}")
                return code

        code, out, err = run(
            client,
            f"docker ps --format '{{{{.Names}}}}' | grep -qx {CONTAINER} && echo running || echo stopped",
            timeout=30,
        )
        if "stopped" in out:
            log(f"ERROR: container '{CONTAINER}' is not running. Start it on the lab host:")
            log("  cd ~/UR5e_Env && ./docker-xiangqi.sh   # or ./docker-start.sh")
            return 1

        build_cmd = (
            f"docker exec -u rosuser -w /home/rosuser/workspace {CONTAINER} bash -lc "
            "'source /opt/ros/humble/setup.bash && "
            "source install/setup.bash 2>/dev/null || true && "
            "colcon build --packages-select "
            "xiangqi_msgs xiangqi_bringup xiangqi_vision xiangqi_ai "
            "xiangqi_planner xiangqi_manipulation xiangqi_dashboard'"
        )
        code, _, err = run(client, build_cmd, timeout=3600)
        if code != 0:
            log(f"ERROR: build_workspace failed ({code})\n{err}")
            return code

        run(client, f"docker ps --filter name={CONTAINER} --format 'Container: {{{{.Names}}}} {{{{.Status}}}}'")
        log("\nDone. Attach with: cd ~/UR5e_Env && ./docker-attach.sh")
        return 0
    finally:
        client.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="10.234.7.84")
    p.add_argument("--user", default="vxlab")
    p.add_argument("--password", default=os.environ.get("LAB_PASSWORD", "welc0me"))
    args = p.parse_args()
    return sync_and_build(args.host, args.user, args.password)


if __name__ == "__main__":
    raise SystemExit(main())
