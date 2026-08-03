"""Unity Editor child-process entry point.

Unity may not expose a writable stdout/stderr handle to hidden child
processes.  Redirect both streams to the Unity Temp folder before importing
and starting the regular server so the process remains stable across Editor
AppDomain reloads.
"""
from __future__ import annotations

import asyncio
import contextlib
from pathlib import Path


def run() -> None:
    log_path = Path(__file__).resolve().parents[1] / "unity" / "Temp" / "V2XServerPython.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8", buffering=1) as log:
        with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            from main import main_async, parse_args
            asyncio.run(main_async(parse_args()))


if __name__ == "__main__":
    run()
