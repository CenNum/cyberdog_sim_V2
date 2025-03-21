"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class new_bms_response_lcmt(object):
    __slots__ = ["dest_id", "src_id", "lowpower_ack", "reserve"]

    __typenames__ = ["int8_t", "int8_t", "int8_t", "int8_t"]

    __dimensions__ = [None, None, None, None]

    def __init__(self):
        self.dest_id = 0
        self.src_id = 0
        self.lowpower_ack = 0
        self.reserve = 0

    def encode(self):
        buf = BytesIO()
        buf.write(new_bms_response_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bbbb", self.dest_id, self.src_id, self.lowpower_ack, self.reserve))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != new_bms_response_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return new_bms_response_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = new_bms_response_lcmt()
        self.dest_id, self.src_id, self.lowpower_ack, self.reserve = struct.unpack(">bbbb", buf.read(4))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if new_bms_response_lcmt in parents: return 0
        tmphash = (0xc97fd300b4e7b308) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if new_bms_response_lcmt._packed_fingerprint is None:
            new_bms_response_lcmt._packed_fingerprint = struct.pack(">Q", new_bms_response_lcmt._get_hash_recursive([]))
        return new_bms_response_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", new_bms_response_lcmt._get_packed_fingerprint())[0]

