"""Assert JUnit XML results: required classes ran, and nothing failed, errored or was skipped.

Usage: python3 -I check_junit.py <results_dir> <required.test.Class> [...]
"""

import pathlib
import sys
import xml.etree.ElementTree as ET


def main() -> int:
    results = pathlib.Path(sys.argv[1])
    required = set(sys.argv[2:])
    files = sorted(results.glob("TEST-*.xml"))
    if not files:
        print(f"no junit results in {results} (are tests enabled in build.gradle?)")
        return 1
    seen, problems, total = set(), [], 0
    for f in files:
        suite = ET.parse(f).getroot()
        seen.add(suite.get("name"))
        total += int(suite.get("tests", 0))
        for key in ("failures", "errors", "skipped"):
            if int(suite.get(key, 0)):
                problems.append(f"{suite.get('name')}: {suite.get(key)} {key}")
    missing = required - seen
    if missing:
        problems.append(f"required test classes did not run: {sorted(missing)}")
    if total == 0:
        problems.append("zero tests ran")
    for p in problems:
        print(p)
    print(f"junit: {len(files)} classes, {total} tests")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
