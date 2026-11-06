"""Generate the AdvantageScope 3D model of the elevator robot from ElevatorConstants.

The geometry lives in src/main/java/frc/robot/Constants.java (ElevatorConstants), which is the
single source of truth for both the robot code and this model. Output goes to
ascope_assets/Robot_GatorboticsElevator/:

  config.json   AdvantageScope robot config (name, orientation, components)
  model.glb     chassis + fixed elevator stage (moves with the robot pose)
  model_0.glb   stage 1  (component 0, logged pose z = carriage height / RIGGING_RATIO)
  model_1.glb   carriage (component 1, logged pose z = carriage height)

Every part is modeled in place in the robot frame (+x forward, +y left, +z up, meters, origin on
the floor at robot center) with the elevator fully down. glTF is Y-up, so vertices are written as
(x, z, -y) and the config rotates the models +90 deg about x, the same convention AdvantageScope's
bundled robots use.

Usage:
  python3 -I tools/elevator/generate_model.py          write the files
  python3 -I tools/elevator/generate_model.py --check  exit 1 if the files on disk differ
"""

import json
import pathlib
import re
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONSTANTS = ROOT / "src/main/java/frc/robot/Constants.java"
OUT = ROOT / "ascope_assets/Robot_GatorboticsElevator"
ROBOT_NAME = "Gatorbotics Elevator Bot"

# RGBA, linear
GREEN = [0.05, 0.45, 0.15, 1.0]  # team green: chassis
ORANGE = [0.95, 0.45, 0.05, 1.0]  # fixed stage
SILVER = [0.75, 0.75, 0.78, 1.0]  # stage 1
BLUE = [0.10, 0.30, 0.85, 1.0]  # carriage
DARK = [0.08, 0.08, 0.08, 1.0]  # bumpers


def load_constants(path: pathlib.Path = CONSTANTS) -> dict:
    src = path.read_text()
    m = re.search(r"class ElevatorConstants \{(.*?)\n  \}", src, re.S)
    if not m:
        raise SystemExit("ElevatorConstants block not found in Constants.java")
    values = {
        name: float(num)
        for name, num in re.findall(
            r"public static final (?:double|int) (\w+) =\s*(-?[\d.]+);", m.group(1)
        )
    }
    required = [
        "RIGGING_RATIO", "MAX_HEIGHT_METERS", "ELEVATOR_X_METERS", "BASE_BOTTOM_Z_METERS",
        "BASE_HEIGHT_METERS", "BASE_RAIL_SPACING_METERS", "STAGE_1_BOTTOM_Z_METERS",
        "STAGE_1_HEIGHT_METERS", "STAGE_1_RAIL_SPACING_METERS", "CARRIAGE_BOTTOM_Z_METERS",
        "CARRIAGE_HEIGHT_METERS", "CARRIAGE_WIDTH_METERS", "RAIL_TUBE_X_METERS",
        "RAIL_TUBE_Y_METERS", "CHASSIS_LENGTH_METERS", "CHASSIS_WIDTH_METERS",
        "CHASSIS_HEIGHT_METERS", "CHASSIS_BOTTOM_Z_METERS",
    ]  # fmt: skip
    missing = [k for k in required if k not in values]
    if missing:
        raise SystemExit(f"ElevatorConstants missing plain-literal values: {missing}")
    return values


def box(cx, cy, cz, sx, sy, sz):
    """Axis-aligned box from center and size in the robot frame -> (min, max)."""
    return ((cx - sx / 2, cy - sy / 2, cz - sz / 2), (cx + sx / 2, cy + sy / 2, cz + sz / 2))


def rails(c: dict, x: float, spacing: float, bottom: float, height: float):
    """Two vertical 2x1 rails plus top and bottom crossbars, centered on y = 0."""
    tx, ty = c["RAIL_TUBE_X_METERS"], c["RAIL_TUBE_Y_METERS"]
    zc = bottom + height / 2
    parts = [box(x, side * spacing / 2, zc, tx, ty, height) for side in (1, -1)]
    span = spacing + ty
    for z in (bottom + ty / 2, bottom + height - ty / 2):
        parts.append(box(x, 0, z, tx, span, ty))
    return parts


def parts(c: dict) -> dict:
    x = c["ELEVATOR_X_METERS"]
    # Stage 1 nests just behind the fixed stage, the carriage just behind stage 1.
    x1 = x - c["RAIL_TUBE_X_METERS"]
    x2 = x1 - c["RAIL_TUBE_X_METERS"]
    cl, cw, ch, cz = (
        c["CHASSIS_LENGTH_METERS"],
        c["CHASSIS_WIDTH_METERS"],
        c["CHASSIS_HEIGHT_METERS"],
        c["CHASSIS_BOTTOM_Z_METERS"],
    )
    bumper = 0.08
    chassis = [box(0, 0, cz + ch / 2, cl, cw, ch)]
    bumpers = [
        box(cl / 2 + bumper / 2, 0, cz + ch / 2, bumper, cw + 2 * bumper, ch * 1.2),
        box(-cl / 2 - bumper / 2, 0, cz + ch / 2, bumper, cw + 2 * bumper, ch * 1.2),
        box(0, cw / 2 + bumper / 2, cz + ch / 2, cl, bumper, ch * 1.2),
        box(0, -cw / 2 - bumper / 2, cz + ch / 2, cl, bumper, ch * 1.2),
    ]
    base = rails(c, x, c["BASE_RAIL_SPACING_METERS"], c["BASE_BOTTOM_Z_METERS"],
                 c["BASE_HEIGHT_METERS"])  # fmt: skip
    stage1 = rails(c, x1, c["STAGE_1_RAIL_SPACING_METERS"], c["STAGE_1_BOTTOM_Z_METERS"],
                   c["STAGE_1_HEIGHT_METERS"])  # fmt: skip
    ccz = c["CARRIAGE_BOTTOM_Z_METERS"] + c["CARRIAGE_HEIGHT_METERS"] / 2
    carriage = [
        box(x2, 0, ccz, 0.02, c["CARRIAGE_WIDTH_METERS"], c["CARRIAGE_HEIGHT_METERS"]),
        # a short scoring shelf on the back face (away from the rails and their crossbars, so it
        # never intersects them through the full travel) to make motion easy to read in 3D
        box(x2 - 0.01 - 0.12, 0, c["CARRIAGE_BOTTOM_Z_METERS"] + 0.01, 0.24,
            c["CARRIAGE_WIDTH_METERS"], 0.02),  # fmt: skip
    ]
    return {
        "model.glb": [(GREEN, chassis), (DARK, bumpers), (ORANGE, base)],
        "model_0.glb": [(SILVER, stage1)],
        "model_1.glb": [(BLUE, carriage)],
    }


# ---- minimal deterministic GLB writer ------------------------------------------------------

FACES = [  # (normal, 4 corner selectors as (ix, iy, iz) into (min,max)), CCW from outside
    ((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)]),
    ((-1, 0, 0), [(0, 1, 0), (0, 0, 0), (0, 0, 1), (0, 1, 1)]),
    ((0, 1, 0), [(1, 1, 0), (0, 1, 0), (0, 1, 1), (1, 1, 1)]),
    ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)]),
    ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]),
    ((0, 0, -1), [(0, 1, 0), (1, 1, 0), (1, 0, 0), (0, 0, 0)]),
]


def to_gltf(v):
    """Robot frame (Z-up) -> glTF (Y-up)."""
    return (v[0], v[2], -v[1])


def mesh_arrays(boxes):
    pos, nrm, idx = [], [], []
    for lo, hi in boxes:
        corners = (lo, hi)
        for normal, sel in FACES:
            base = len(pos)
            for ix, iy, iz in sel:
                pos.append(to_gltf((corners[ix][0], corners[iy][1], corners[iz][2])))
                nrm.append(to_gltf(normal))
            idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    return pos, nrm, idx


def f32(v: float) -> float:
    return struct.unpack("<f", struct.pack("<f", v))[0]


def build_glb(groups) -> bytes:
    gltf = {
        "asset": {"version": "2.0", "generator": "tools/elevator/generate_model.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": []}],
        "materials": [],
        "accessors": [],
        "bufferViews": [],
        "buffers": [],
    }
    blob = bytearray()

    def add_view(data: bytes, target: int) -> int:
        while len(blob) % 4:
            blob.append(0)
        gltf["bufferViews"].append(
            {"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), "target": target}
        )
        blob.extend(data)
        return len(gltf["bufferViews"]) - 1

    for color, boxes in groups:
        pos, nrm, idx = mesh_arrays(boxes)
        pos = [tuple(f32(a) for a in p) for p in pos]
        mat = len(gltf["materials"])
        gltf["materials"].append(
            {"pbrMetallicRoughness": {"baseColorFactor": color, "metallicFactor": 0.1,
                                      "roughnessFactor": 0.7}}  # fmt: skip
        )
        acc = len(gltf["accessors"])
        pv = add_view(b"".join(struct.pack("<3f", *p) for p in pos), 34962)
        nv = add_view(b"".join(struct.pack("<3f", *n) for n in nrm), 34962)
        iv = add_view(struct.pack(f"<{len(idx)}H", *idx), 34963)
        gltf["accessors"] += [
            {"bufferView": pv, "componentType": 5126, "count": len(pos), "type": "VEC3",
             "min": [min(p[i] for p in pos) for i in range(3)],
             "max": [max(p[i] for p in pos) for i in range(3)]},
            {"bufferView": nv, "componentType": 5126, "count": len(nrm), "type": "VEC3"},
            {"bufferView": iv, "componentType": 5123, "count": len(idx), "type": "SCALAR"},
        ]  # fmt: skip
        gltf["meshes"][0]["primitives"].append(
            {"attributes": {"POSITION": acc, "NORMAL": acc + 1}, "indices": acc + 2,
             "material": mat}  # fmt: skip
        )
    while len(blob) % 4:
        blob.append(0)
    gltf["buffers"].append({"byteLength": len(blob)})
    js = json.dumps(gltf, separators=(",", ":"), sort_keys=True).encode()
    js += b" " * (-len(js) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)
    return (
        struct.pack("<4sII", b"glTF", 2, total)
        + struct.pack("<I4s", len(js), b"JSON")
        + js
        + struct.pack("<I4s", len(blob), b"BIN\x00")
        + bytes(blob)
    )


def config_json() -> bytes:
    up = [{"axis": "x", "degrees": 90}]
    cfg = {
        "name": ROBOT_NAME,
        "isFTC": False,
        "rotations": up,
        "position": [0, 0, 0],
        "cameras": [],
        "components": [
            {"zeroedRotations": up, "zeroedPosition": [0, 0, 0]},  # 0: stage 1
            {"zeroedRotations": up, "zeroedPosition": [0, 0, 0]},  # 1: carriage
        ],
    }
    return (json.dumps(cfg, indent=2) + "\n").encode()


def outputs() -> dict:
    c = load_constants()
    files = {name: build_glb(groups) for name, groups in parts(c).items()}
    files["config.json"] = config_json()
    return files


def main() -> int:
    files = outputs()
    if "--check" in sys.argv:
        stale = [n for n, data in files.items() if not (OUT / n).is_file()
                 or (OUT / n).read_bytes() != data]  # fmt: skip
        extra = sorted(p.name for p in OUT.glob("*") if p.name not in files) if OUT.is_dir() else []
        if stale or extra:
            print(f"stale: {stale} unexpected: {extra}; run tools/elevator/generate_model.py")
            return 1
        print(f"model in sync ({len(files)} files)")
        return 0
    OUT.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (OUT / name).write_bytes(data)
    print(f"wrote {len(files)} files to {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
