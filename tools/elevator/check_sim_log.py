"""Assert the headless E2E sim log shows a working elevator. Exit 0 == pass.

Usage: python3 -I tools/elevator/check_sim_log.py <file.wpilog>

Checks (all thresholds derived from ElevatorConstants, parsed from Constants.java):
  L1  the bottom limit closes, then Homed goes true, before any raised goal
  L2  goals after homing visit STOW, LOW, HIGH, MID, STOW in that order
  L3  each move enters and stays within POSITION_TOLERANCE of its goal, in profile time + margin
  L4  overshoot past each goal is under 2 cm
  L5  carriage never leaves [MIN - 5 mm, MAX + 5 mm]; sim truth never exceeds the top hard stop
  L6  after homing, whenever the carriage is still, encoder == sim true height within 2 mm
  L7  every logged ComponentPoses sample is [stage1 z = h / R, carriage z = h] for the
      logged height h, with zero x, y and identity rotation (the 3D model will track the code)
  L8  applied volts stay within +/-12.5 V and stator current within the limit + 5 A
  L9  the robot sits on open floor, at least 0.3 m from every field wall
"""

import bisect
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import generate_model  # noqa: E402
import wpilog  # noqa: E402

C = generate_model.load_constants()
P = "/Mech/Elevator/IO/"
O = "/RealOutputs/Mech/Elevator/"


class Series:
    def __init__(self, samples):
        self.t = [s[0] for s in samples]
        self.v = [s[1] for s in samples]

    def at(self, t):
        """Last value logged at or before t (AdvantageKit only logs changes)."""
        i = bisect.bisect_right(self.t, t + 1e-9) - 1
        return self.v[i] if i >= 0 else None


def profile_time(d: float) -> float:
    """Time for a trapezoidal profile over distance d, plus jerk ramps."""
    v, a, j = C["CRUISE_VELOCITY_MPS"], C["ACCELERATION_MPS2"], C["JERK_MPS3"]
    t = d / v + v / a if d > v * v / a else 2 * math.sqrt(d / a)
    return t + (2 * a / j if j > 0 else 0)


def main() -> int:
    log = wpilog.read(sys.argv[1])
    errors: list[str] = []

    def need(key):
        if key not in log or not log[key][1]:
            raise SystemExit(f"FAIL missing log entry {key}")
        return Series(log[key][1])

    pos = need(P + "PositionMeters")
    limit = need(P + "BottomLimitPressed")
    volts = need(P + "AppliedVolts")
    stator = need(P + "StatorCurrentAmps")
    homed = need(O + "Homed")
    goal = need(O + "GoalMeters")
    poses = need(O + "ComponentPoses")
    truth = need(O + "Sim/TrueHeightMeters")
    tol = C["POSITION_TOLERANCE_METERS"]

    # L1: a back-drivable elevator may slide onto the switch while disabled and home at boot,
    # which is fine. What matters: the switch closed, Homed followed, before any raised goal.
    t_homed = next((t for t, v in zip(homed.t, homed.v) if v), None)
    t_limit = next((t for t, v in zip(limit.t, limit.v) if v), None)
    if t_homed is None or t_limit is None or t_limit > t_homed + 1e-9:
        errors.append(f"L1 homing never completed (limit at {t_limit}, homed at {t_homed})")
        t_homed = t_homed or 0.0

    # L2
    raised_before = [v for t, v in zip(goal.t, goal.v) if t < t_homed and v > tol]
    if raised_before:
        errors.append(f"L1 raised goal {raised_before} set before homing")
    # AdvantageKit only logs changes, so seed with the goal in effect when homing finished
    seq = [(t_homed, goal.at(t_homed))]
    for t, v in zip(goal.t, goal.v):
        if t >= t_homed and (not seq or abs(seq[-1][1] - v) > 1e-9):
            seq.append((t, v))
    want = [C[k] for k in ("STOW_HEIGHT_METERS", "LOW_HEIGHT_METERS", "HIGH_HEIGHT_METERS",
                           "MID_HEIGHT_METERS", "STOW_HEIGHT_METERS")]  # fmt: skip
    got = [round(v, 6) for _, v in seq]
    if got != [round(w, 6) for w in want]:
        errors.append(f"L2 goal sequence {got} != {want}")

    # L3, L4
    end_t = pos.t[-1]
    for i, (t0, g) in enumerate(seq):
        t1 = seq[i + 1][0] if i + 1 < len(seq) else end_t
        start = pos.at(t0)
        d = abs(g - start)
        allowed = profile_time(d) + 0.6
        # settled = the first sample after which the carriage stays in tolerance until the next goal
        win = [(t, v) for t, v in zip(pos.t, pos.v) if t0 <= t <= t1]
        settled = None
        for t, v in reversed(win):
            if abs(v - g) > tol:
                break
            settled = t
        if settled is None:
            errors.append(f"L3 never reached goal {g:.3f} (from {start:.3f})")
            continue
        took = settled - t0
        status = "ok" if took <= allowed else "SLOW"
        print(f"move {start:.3f} -> {g:.3f} m: settled in {took:.2f}s (allowed {allowed:.2f}s) {status}")
        if took > allowed:
            errors.append(f"L3 move to {g:.3f} took {took:.2f}s > {allowed:.2f}s")
        seg = [v for t, v in zip(pos.t, pos.v) if t0 <= t <= t1]
        over = (max(seg) - g) if g >= start else (g - min(seg))
        if over > 0.02:
            errors.append(f"L4 overshoot {over * 100:.1f} cm past {g:.3f}")

    # L5
    lo, hi = C["MIN_HEIGHT_METERS"] - 0.005, C["MAX_HEIGHT_METERS"] + 0.005
    # strictly after the homing cycle: the reset lands on the cycle after the limit closes
    out = [v for t, v in zip(pos.t, pos.v) if t > t_homed + 0.03 and not lo <= v <= hi]
    if out:
        errors.append(f"L5 encoder left travel range: {out[:5]}")
    if max(truth.v) > C["MAX_HEIGHT_METERS"] + 0.02 + 1e-6:
        errors.append(f"L5 carriage hit the top hard stop ({max(truth.v):.3f} m)")

    # L6: the homing offset is right if the encoder matches truth whenever the carriage is still.
    # (While moving, the encoder trails truth by one sensor update, which is latency, not offset.)
    vel = need(P + "VelocityMetersPerSec")
    still = [(t, v) for t, v in zip(truth.t, truth.v)
             if t > t_homed + 0.05 and abs(vel.at(t)) < 0.05]  # fmt: skip
    worst = max((abs(pos.at(t) - v) for t, v in still), default=None)
    if worst is None:
        errors.append("L6 no stationary samples after homing")
    elif worst > 0.002:
        errors.append(f"L6 stationary encoder off sim truth by {worst * 1000:.1f} mm after homing")

    # L7
    r = C["RIGGING_RATIO"]
    bad = 0
    for t, ps in zip(poses.t, poses.v):
        h = pos.at(t)
        if h is None or len(ps) != 2:
            bad += 1
            continue
        (x0, y0, z0, w0, *q0), (x1, y1, z1, w1, *q1) = ps
        if (abs(z1 - h) > 1e-6 or abs(z0 - h / r) > 1e-6 or max(map(abs, (x0, y0, x1, y1))) > 1e-9
                or abs(w0 - 1) > 1e-9 or abs(w1 - 1) > 1e-9):  # fmt: skip
            bad += 1
    if bad:
        errors.append(f"L7 {bad}/{len(poses.t)} ComponentPoses samples disagree with position")

    # L8
    if max(map(abs, volts.v)) > 12.5:
        errors.append(f"L8 applied volts reached {max(map(abs, volts.v)):.2f}")
    if max(stator.v) > C["STATOR_CURRENT_LIMIT_AMPS"] + 5:
        errors.append(f"L8 stator current reached {max(stator.v):.1f} A")

    # L9: the robot starts on open floor, fully inside the field (not in a corner or a wall)
    import struct
    field_x, field_y, clearance = 16.541, 8.069, 0.30
    half = max(C["CHASSIS_LENGTH_METERS"], C["CHASSIS_WIDTH_METERS"]) / 2 + 0.08  # + bumpers
    odo = log.get("/RealOutputs/Drive/Odometry/Robot", ("", []))[1]
    if not odo:
        errors.append("L9 robot pose /RealOutputs/Drive/Odometry/Robot never logged")
    else:
        x, y, _ = struct.unpack("<3d", odo[-1][1])
        margin = min(x, y, field_x - x, field_y - y) - half
        print(f"robot at x={x:.2f} y={y:.2f}, {margin:.2f} m clear of the nearest wall")
        if margin < clearance:
            errors.append(f"L9 robot at ({x:.2f}, {y:.2f}) is only {margin:.2f} m from a wall")

    print(f"samples: position {len(pos.t)}, poses {len(poses.t)}, homed at {t_homed:.2f}s, "
          f"peak current {max(stator.v):.1f} A, final current {stator.v[-1]:.1f} A")  # fmt: skip
    for e in errors:
        print("FAIL", e)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
