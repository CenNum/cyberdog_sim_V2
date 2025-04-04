"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class error_flag_lcmt(object):
    __slots__ = ["exist_error", "ori_error", "footpos_error", "motor_error"]

    __typenames__ = ["int8_t", "int8_t", "int32_t", "int32_t"]

    __dimensions__ = [None, None, None, [12]]

    def __init__(self):
        self.exist_error = 0
        self.ori_error = 0
        self.footpos_error = 0
        self.motor_error = [ 0 for dim0 in range(12) ]

    def encode(self):
        buf = BytesIO()
        buf.write(error_flag_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bbi", self.exist_error, self.ori_error, self.footpos_error))
        buf.write(struct.pack('>12i', *self.motor_error[:12]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != error_flag_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return error_flag_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = error_flag_lcmt()
        self.exist_error, self.ori_error, self.footpos_error = struct.unpack(">bbi", buf.read(6))
        self.motor_error = struct.unpack('>12i', buf.read(48))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if error_flag_lcmt in parents: return 0
        tmphash = (0x8779d473df5369b8) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if error_flag_lcmt._packed_fingerprint is None:
            error_flag_lcmt._packed_fingerprint = struct.pack(">Q", error_flag_lcmt._get_hash_recursive([]))
        return error_flag_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", error_flag_lcmt._get_packed_fingerprint())[0]

