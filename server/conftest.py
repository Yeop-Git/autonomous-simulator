"""pytest configuration for the server package.

Adds the ``server/`` directory to ``sys.path`` so tests and modules can use
flat imports (``import world_model``, ``from planners import AStarPlanner``)
without installing the package. Mirrors how ``main.py`` is run from inside
``server/``.
"""
import sys
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))
