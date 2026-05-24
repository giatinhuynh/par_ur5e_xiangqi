#!/usr/bin/env python3
"""Lightweight static checks for project wiring that is easy to regress."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _fail(message: str) -> None:
    print(f'FAIL: {message}', file=sys.stderr)
    raise SystemExit(1)


def _require_package_deps(package_xml: str, deps: set[str]) -> None:
    path = ROOT / package_xml
    root = ET.parse(path).getroot()
    present = {
        elem.text.strip()
        for elem in root
        if elem.tag in {'depend', 'exec_depend', 'build_depend', 'buildtool_depend'}
        and elem.text
    }
    missing = sorted(deps - present)
    if missing:
        _fail(f'{package_xml} missing dependency/dependencies: {", ".join(missing)}')


def _check_capture_tree() -> None:
    path = ROOT / 'workspace/src/xiangqi_planner/xiangqi_planner/task_planner_node.py'
    text = path.read_text()
    capture_block_start = text.find("capture_subtree.add_children([")
    if capture_block_start == -1:
        _fail('Capture subtree wiring not found')
    capture_block = text[capture_block_start:text.find('])', capture_block_start) + 2]
    if 'FailureIsSuccess' in capture_block:
        _fail('Capture skip path must not turn capture failures into success')
    if "Inverter(name='SkipCaptureIfNone', child=IsCapture())" not in capture_block:
        _fail('Capture subtree should skip only when IsCapture() is false')


def main() -> int:
    _check_capture_tree()
    _require_package_deps('workspace/src/xiangqi_ai/package.xml', {'std_srvs'})
    _require_package_deps('workspace/src/xiangqi_manipulation/package.xml', {'std_srvs'})
    _require_package_deps('workspace/src/xiangqi_planner/package.xml', {'std_srvs'})
    _require_package_deps(
        'workspace/src/xiangqi_dashboard/package.xml',
        {'python3-flask', 'python3-flask-cors', 'python3-flask-socketio'},
    )
    print('Static project checks passed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
