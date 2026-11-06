"""Assert no CTRE device keeps failing during the E2E sim run. Exit 0 == pass.

Usage: python3 -I tools/elevator/check_can_errors.py <sim_console.txt>

Phoenix sim devices may each miss their first frames while booting, so one error per
(device, signal) is allowed. The same (device, signal) failing twice means the device never
came up (e.g. a CAN ID that does not exist).
"""

import collections
import re
import sys


def main() -> int:
    text = open(sys.argv[1], errors="replace").read()
    hits = collections.Counter(re.findall(r'(talon fx|cancoder|pigeon 2) (\d+) \("[^"]*"\) (.+)', text))
    repeats = {k: n for k, n in hits.items() if n > 1}
    for (dev, can_id, what), n in sorted(repeats.items()):
        print(f"FAIL {dev} {can_id}: '{what.strip()}' failed {n} times")
    print(f"can errors: {sum(hits.values())} total, {len(repeats)} repeating")
    return 1 if repeats else 0


if __name__ == "__main__":
    sys.exit(main())
