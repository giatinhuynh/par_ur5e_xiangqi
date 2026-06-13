#!/usr/bin/env python3
import paramiko
import sys

cmd = (
    "docker exec -u rosuser -w /home/rosuser/workspace ros2 bash -lc "
    "'source /opt/ros/humble/setup.bash && "
    "source install/setup.bash 2>/dev/null || true && "
    "colcon build --packages-select "
    "xiangqi_msgs xiangqi_bringup xiangqi_vision xiangqi_ai "
    "xiangqi_planner xiangqi_manipulation xiangqi_dashboard'"
)

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect("10.234.7.84", username="vxlab", password="welc0me", timeout=30)
print(">>>", cmd, flush=True)
_, stdout, stderr = c.exec_command(cmd, timeout=3600, get_pty=True)
out = stdout.read().decode(errors="replace")
err = stderr.read().decode(errors="replace")
code = stdout.channel.recv_exit_status()
if out:
    print(out, end="" if out.endswith("\n") else "\n")
if err:
    print(err, file=sys.stderr)
c.close()
sys.exit(code)
