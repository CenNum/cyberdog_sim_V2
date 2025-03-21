"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

import error_flag_lcmt

class motion_control_response_lcmt(object):
    __slots__ = ["pattern", "order", "order_process_bar", "foot_contact", "error_flag"]

    __typenames__ = ["int8_t", "int8_t", "int8_t", "int8_t", "error_flag_lcmt"]

    __dimensions__ = [None, None, None, None, None]

    def __init__(self):
        self.pattern = 0
        self.order = 0
        self.order_process_bar = 0
        self.foot_contact = 0
        self.error_flag = error_flag_lcmt()

    def encode(self):
        buf = BytesIO()
        buf.write(motion_control_response_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bbbb", self.pattern, self.order, self.order_process_bar, self.foot_contact))
        assert self.error_flag._get_packed_fingerprint() == error_flag_lcmt._get_packed_fingerprint()
        self.error_flag._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != motion_control_response_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return motion_control_response_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = motion_control_response_lcmt()
        self.pattern, self.order, self.order_process_bar, self.foot_contact = struct.unpack(">bbbb", buf.read(4))
        self.error_flag = error_flag_lcmt._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if motion_control_response_lcmt in parents: return 0
        newparents = parents + [motion_control_response_lcmt]
        tmphash = (0x7bea14505ededc35+ error_flag_lcmt._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if motion_control_response_lcmt._packed_fingerprint is None:
            motion_control_response_lcmt._packed_fingerprint = struct.pack(">Q", motion_control_response_lcmt._get_hash_recursive([]))
        return motion_control_response_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", motion_control_response_lcmt._get_packed_fingerprint())[0]

