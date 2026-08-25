"""
Alias kept for the old invocation. These are the same run:

    python3 person_avoiding.py --condition strong
    python3 button_pressing.py _person --condition strong

Prefer the second -- button_pressing.py is the single runner and takes the
config variant as its first argument.
"""

import sys

from button_pressing import main

if __name__ == "__main__":
    if not any(a.lstrip("_") == "person" for a in sys.argv[1:]):
        sys.argv.insert(1, "_person")
    main()
