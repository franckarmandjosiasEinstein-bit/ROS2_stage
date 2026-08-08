"""make_plants -- entry point for agri.world.plants.

Mirrors make_world / make_robot so the three generators are invoked the
same way, and so `python3 -m agri.world.make_plants` works.
"""
from agri.world.plants import main

if __name__ == "__main__":
    raise SystemExit(main())
