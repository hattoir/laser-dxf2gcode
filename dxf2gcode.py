#!/usr/bin/env python3
"""エントリポイント: python dxf2gcode.py <command> ..."""
import sys

from src.cli import main

if __name__ == "__main__":
    sys.exit(main())
