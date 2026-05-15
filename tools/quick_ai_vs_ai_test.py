#!/usr/bin/env python3
import json
import time
import urllib.request

BASE = "http://127.0.0.1:5000"


def post(path, data=None):
    body = json.dumps(data or {}).encode()
    req = urllib.request.Request(
        BASE + path,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read().decode())


def get_state():
    with urllib.request.urlopen(BASE + "/api/state", timeout=10) as r:
        return json.loads(r.read().decode())


def main():
    print(post("/api/set_engines", {"red_engine": "minimax", "black_engine": "minimax"}))
    print(post("/api/new_game"))
    for i in range(20):
        time.sleep(2)
        s = get_state()
        print(
            i,
            "status=",
            s.get("game_status"),
            "moves=",
            s.get("move_count"),
            "engines=",
            s.get("red_engine"),
            s.get("black_engine"),
        )
        if s.get("move_count", 0) >= 3:
            print("PASS: game advanced past first ply")
            return
    raise SystemExit("FAIL: stuck at move_count < 3")


if __name__ == "__main__":
    main()
