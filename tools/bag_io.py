"""Read the four dataset CDR message types without installing ROS.

Only CDR1 (big/little endian) and the explicitly listed schemas are supported.
This is an offline reader; the ROS node uses generated message classes.
"""
import sqlite3
import struct
from pathlib import Path

TOPICS = {
    '/vehicle/front_bogie_velocity': 'front',
    '/vehicle/rear_bogie_velocity': 'rear',
    '/vehicle/driver_position_cmd': 'command',
    '/sensing/gnss/master/vel': 'master_vel',
    '/sensing/gnss/rover/vel': 'rover_vel',
    '/sensing/gnss/master/fix': 'master_fix',
    '/sensing/gnss/rover/fix': 'rover_fix',
}
SCHEMAS = {
    'front': 'tram_vehicle_msgs/msg/VelocitySensor',
    'rear': 'tram_vehicle_msgs/msg/VelocitySensor',
    'command': 'tram_vehicle_msgs/msg/DriverControllerCommand',
    'master_vel': 'geometry_msgs/msg/TwistStamped',
    'rover_vel': 'geometry_msgs/msg/TwistStamped',
    'master_fix': 'sensor_msgs/msg/NavSatFix',
    'rover_fix': 'sensor_msgs/msg/NavSatFix',
}


def decode(raw, kind):
    if len(raw) < 17 or raw[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('Unsupported/truncated CDR1 message')
    endian = '<' if raw[1] == 1 else '>'
    sec, ns, n = struct.unpack_from(endian + 'iII', raw, 4)
    if ns >= 1000000000 or n < 1 or 16 + n > len(raw):
        raise ValueError('Invalid Header')
    pos = 16 + n
    stamp_ns = sec * 1000000000 + ns
    if kind == 'command':
        return stamp_ns, (struct.unpack_from('b', raw, pos)[0],)
    if kind in ('front', 'rear'):
        pos += (-(pos - 4)) % 8
        return stamp_ns, struct.unpack_from(endian + 'd', raw, pos)
    if kind.endswith('_vel'):
        pos += (-(pos - 4)) % 8
        return stamp_ns, struct.unpack_from(endian + '3d', raw, pos)
    if kind.endswith('_fix'):
        status = struct.unpack_from('b', raw, pos)[0]
        pos += 1
        pos += (-(pos - 4)) % 2
        pos += 2  # NavSatStatus.service
        pos += (-(pos - 4)) % 8
        return stamp_ns, (*struct.unpack_from(endian + '3d', raw, pos), status)
    raise ValueError(kind)


def messages(bag):
    """Yield receipt_ns, header_ns, topic key, payload in recorded arrival order."""
    files = sorted(Path(bag).glob('*.db3')) if Path(bag).is_dir() else [Path(bag)]
    if len(files) != 1:
        raise ValueError('This reader expects one db3 per bag, as in the supplied data')
    conn = sqlite3.connect(files[0].resolve().as_uri() + '?mode=ro', uri=True)
    try:
        mapping = {}
        for tid, name, typ, fmt in conn.execute('SELECT id,name,type,serialization_format FROM topics'):
            if name in TOPICS:
                kind = TOPICS[name]
                if fmt != 'cdr' or typ != SCHEMAS[kind]:
                    raise ValueError(f'Unexpected schema: {name}: {typ} ({fmt})')
                mapping[tid] = kind
        for receipt, tid, raw in conn.execute('SELECT timestamp,topic_id,data FROM messages ORDER BY timestamp,id'):
            if tid in mapping:
                stamp, values = decode(raw, mapping[tid])
                yield receipt, stamp, mapping[tid], values
    finally:
        conn.close()


def load_cache(path):
    import numpy as np
    with np.load(path, allow_pickle=False) as obj:
        return {key: obj[key] for key in obj.files}
