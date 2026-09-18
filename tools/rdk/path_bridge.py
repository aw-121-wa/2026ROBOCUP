#!/usr/bin/env python3
"""Compatibility entry point for the unchanged ZHY bridge; Q/R is retired."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from RDK.tools.rdk_stm32_bridge import main

if __name__ == "__main__":
    raise SystemExit(main())
