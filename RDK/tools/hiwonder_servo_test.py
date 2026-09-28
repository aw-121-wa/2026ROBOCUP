#!/usr/bin/env python3
"""Manual RDK X5 -> Hiwonder action-group UART test.

Run from the licang_vision project root, for example:
  python3 tools/hiwonder_servo_test.py --port /dev/ttyS1

No action group is run automatically. Choose a group interactively, or pass
--group for a deliberate one-shot test.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from hiwonder_action import HiwonderActionBoard

GROUP_NAMES = {
    0: "G0 start pose",
    1: "G1 return / disc-camera pose",
    2: "G2 disc grab",
    3: "G3 pillar camera pose",
    4: "G4 pillar grab",
    5: "G5 stair layer-1 camera pose",
    6: "G6 stair layer-1 grab",
    7: "G7 stair transition 1->2",
    8: "G8 stair layer-2 camera pose",
    9: "G9 stair layer-2 grab",
    10: "G10 stair transition 2->3",
    11: "G11 stair layer-3 camera pose",
    12: "G12 stair layer-3 grab",
}


def run_one(board, group: int, repeat: int, timeout: float) -> bool:
    name = GROUP_NAMES.get(group, f"G{group}")
    print(f"\nAbout to run: {name}, repeat={repeat}")
    answer = input("Type RUN to execute, anything else to cancel: ").strip()
    if answer != "RUN":
        print("Cancelled.")
        return False
    try:
        done = board.run_group(group, repeat, timeout)
    except TimeoutError as exc:
        print(f"TIMEOUT: {exc}")
        return False
    print(
        f"COMPLETE: group={group}, command=0x{done.command:02X}, "
        f"frame={done.raw.hex(' ')}"
    )
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="RDK X5 direct Hiwonder action test")
    parser.add_argument("--port", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--group", type=int, choices=range(0, 13))
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(argv)

    print(f"Opening {args.port} at {args.baud} 8N1")
    print("IMPORTANT: STM32 must be disconnected from the same servo UART bus.")
    with HiwonderActionBoard(args.port, args.baud) as board:
        if args.group is not None:
            return 0 if run_one(board, args.group, args.repeat, args.timeout) else 1

        print("\nAvailable groups:")
        for group, name in GROUP_NAMES.items():
            print(f"  {group:2d}: {name}")
        print("  q : quit")

        while True:
            raw = input("\nSelect group: ").strip().lower()
            if raw in {"q", "quit", "exit"}:
                return 0
            try:
                group = int(raw)
            except ValueError:
                print("Enter 0..12 or q")
                continue
            if group not in GROUP_NAMES:
                print("For this test use groups 0..12 only")
                continue
            run_one(board, group, args.repeat, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
