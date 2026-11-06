"""Assert robot loop time does not grow while enabled (catches per-loop leaks). Exit 0 == pass.

Usage: python3 -I tools/elevator/check_loop_time.py <file.wpilog>
Compares median UserCodeMS over the first and last second of enabled time.
"""

import pathlib
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import wpilog  # noqa: E402


def main() -> int:
    log = wpilog.read(sys.argv[1])
    enabled = log["/DriverStation/Enabled"][1]
    t_on = next(t for t, v in enabled if v)
    user = [(t, v) for t, v in log["/RealOutputs/LoggedRobot/UserCodeMS"][1] if t >= t_on]
    t_end = user[-1][0]
    first = statistics.median(v for t, v in user if t < t_on + 1.0)
    last = statistics.median(v for t, v in user if t > t_end - 1.0)
    print(f"user code ms: first enabled second {first:.2f}, last {last:.2f} "
          f"(enabled {t_end - t_on:.1f}s)")  # fmt: skip
    if last > first * 1.5 + 0.5:
        print("FAIL loop time grows while enabled (per-loop leak?)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
