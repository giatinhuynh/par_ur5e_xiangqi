#!/usr/bin/env python3
"""Verify stop halts AI vs AI and reset restarts from move 0."""
import json
import time
import urllib.request

BASE = "http://127.0.0.1:5000"


def post(path):
    req = urllib.request.Request(BASE + path, method="POST")
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read().decode())


def state():
    with urllib.request.urlopen(BASE + "/api/state", timeout=10) as r:
        return json.loads(r.read().decode())


def main():
    post("/api/new_game")
    time.sleep(6)
    s = state()
    assert s.get("move_count", 0) >= 2, f"expected game running, got {s}"
    post("/api/stop_game")
    time.sleep(2)
    s = state()
    assert s.get("game_status") == "idle", s
    assert s.get("move_count", 0) == 0, s
    time.sleep(3)
    s2 = state()
    assert s2.get("game_status") == "idle", f"game restarted after stop: {s2}"
    assert s2.get("move_count", 0) == 0, s2
    print("STOP OK")
    post("/api/reset_game")
    time.sleep(4)
    s3 = state()
    assert s3.get("move_count", 0) >= 1, f"reset should restart: {s3}"
    assert s3.get("game_status") != "idle", s3
    print("RESET OK", "moves", s3.get("move_count"))


if __name__ == "__main__":
    main()
