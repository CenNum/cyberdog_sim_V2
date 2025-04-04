"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class leg_control_data_lcmt(object):
    __slots__ = ["q", "qd", "p", "v", "tau_est", "force_est", "force_desired", "q_abad_limit", "q_hip_limit", "q_knee_limit"]

    __typenames__ = ["float", "float", "float", "float", "float", "float", "float", "int32_t", "int32_t", "int32_t"]

    __dimensions__ = [[12], [12], [12], [12], [12], [12], [12], [4], [4], [4]]

    def __init__(self):
        self.q = [ 0.0 for dim0 in range(12) ]
        self.qd = [ 0.0 for dim0 in range(12) ]
        self.p = [ 0.0 for dim0 in range(12) ]
        self.v = [ 0.0 for dim0 in range(12) ]
        self.tau_est = [ 0.0 for dim0 in range(12) ]
        self.force_est = [ 0.0 for dim0 in range(12) ]
        self.force_desired = [ 0.0 for dim0 in range(12) ]
        self.q_abad_limit = [ 0 for dim0 in range(4) ]
        self.q_hip_limit = [ 0 for dim0 in range(4) ]
        self.q_knee_limit = [ 0 for dim0 in range(4) ]

    def encode(self):
        buf = BytesIO()
        buf.write(leg_control_data_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>12f', *self.q[:12]))
        buf.write(struct.pack('>12f', *self.qd[:12]))
        buf.write(struct.pack('>12f', *self.p[:12]))
        buf.write(struct.pack('>12f', *self.v[:12]))
        buf.write(struct.pack('>12f', *self.tau_est[:12]))
        buf.write(struct.pack('>12f', *self.force_est[:12]))
        buf.write(struct.pack('>12f', *self.force_desired[:12]))
        buf.write(struct.pack('>4i', *self.q_abad_limit[:4]))
        buf.write(struct.pack('>4i', *self.q_hip_limit[:4]))
        buf.write(struct.pack('>4i', *self.q_knee_limit[:4]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != leg_control_data_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return leg_control_data_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = leg_control_data_lcmt()
        self.q = struct.unpack('>12f', buf.read(48))
        self.qd = struct.unpack('>12f', buf.read(48))
        self.p = struct.unpack('>12f', buf.read(48))
        self.v = struct.unpack('>12f', buf.read(48))
        self.tau_est = struct.unpack('>12f', buf.read(48))
        self.force_est = struct.unpack('>12f', buf.read(48))
        self.force_desired = struct.unpack('>12f', buf.read(48))
        self.q_abad_limit = struct.unpack('>4i', buf.read(16))
        self.q_hip_limit = struct.unpack('>4i', buf.read(16))
        self.q_knee_limit = struct.unpack('>4i', buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if leg_control_data_lcmt in parents: return 0
        tmphash = (0xa6b1824464a42a6b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if leg_control_data_lcmt._packed_fingerprint is None:
            leg_control_data_lcmt._packed_fingerprint = struct.pack(">Q", leg_control_data_lcmt._get_hash_recursive([]))
        return leg_control_data_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", leg_control_data_lcmt._get_packed_fingerprint())[0]

