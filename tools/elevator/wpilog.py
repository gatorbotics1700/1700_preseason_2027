"""Minimal WPILOG reader (stdlib only). Spec: wpilibNewCommands datalog.adoc.

read(path) -> {entry_name: (type_string, [(timestamp_seconds, value), ...])}
Decodes double, int64, boolean, string, double[], and struct:Pose3d / struct:Pose3d[].
Other types are kept as raw bytes.
"""

import struct


def _decode(type_str: str, data: bytes):
    if type_str == "double":
        return struct.unpack("<d", data)[0]
    if type_str == "int64":
        return struct.unpack("<q", data)[0]
    if type_str == "boolean":
        return data[0] != 0
    if type_str in ("string", "json"):
        return data.decode("utf-8", "replace")
    if type_str == "double[]":
        return list(struct.unpack(f"<{len(data) // 8}d", data))
    if type_str in ("struct:Pose3d", "struct:Pose3d[]"):
        # Pose3d = Translation3d(x, y, z) + Rotation3d(Quaternion(w, x, y, z)) = 7 doubles
        n = len(data) // 56
        poses = [struct.unpack_from("<7d", data, 56 * i) for i in range(n)]
        return poses if type_str.endswith("[]") else poses[0]
    return data


def read(path: str) -> dict:
    buf = open(path, "rb").read()
    if buf[:6] != b"WPILOG":
        raise ValueError(f"{path} is not a WPILOG file")
    extra_len = struct.unpack_from("<I", buf, 8)[0]
    pos = 12 + extra_len
    entries: dict[int, tuple[str, str]] = {}
    out: dict[str, tuple[str, list]] = {}
    while pos < len(buf):
        hdr = buf[pos]
        id_len = (hdr & 0x3) + 1
        size_len = ((hdr >> 2) & 0x3) + 1
        ts_len = ((hdr >> 4) & 0x7) + 1
        pos += 1
        if pos + id_len + size_len + ts_len > len(buf):
            break  # truncated tail
        entry = int.from_bytes(buf[pos : pos + id_len], "little")
        pos += id_len
        size = int.from_bytes(buf[pos : pos + size_len], "little")
        pos += size_len
        ts = int.from_bytes(buf[pos : pos + ts_len], "little") / 1e6
        pos += ts_len
        payload = buf[pos : pos + size]
        pos += size
        if len(payload) < size:
            break
        if entry == 0:  # control record
            if payload[0] == 0:  # start
                eid, nlen = struct.unpack_from("<II", payload, 1)
                name = payload[9 : 9 + nlen].decode()
                o = 9 + nlen
                tlen = struct.unpack_from("<I", payload, o)[0]
                typ = payload[o + 4 : o + 4 + tlen].decode()
                entries[eid] = (name, typ)
                out.setdefault(name, (typ, []))
            continue
        if entry in entries:
            name, typ = entries[entry]
            out[name][1].append((ts, _decode(typ, payload)))
    return out
