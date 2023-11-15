"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct


class fr3_cmd(object):
    __slots__ = ["timestamp", "cmd"]

    __typenames__ = ["int64_t", "double"]

    __dimensions__ = [None, [9]]

    def __init__(self):
        self.timestamp = 0
        self.cmd = [0.0 for dim0 in range(9)]

    def encode(self):
        buf = BytesIO()
        buf.write(fr3_cmd._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack(">9d", *self.cmd[:9]))

    def decode(data):
        if hasattr(data, "read"):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != fr3_cmd._get_packed_fingerprint():
            raise ValueError("Decode error")
        return fr3_cmd._decode_one(buf)

    decode = staticmethod(decode)

    def _decode_one(buf):
        self = fr3_cmd()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.cmd = struct.unpack(">9d", buf.read(72))
        return self

    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if fr3_cmd in parents:
            return 0
        tmphash = (0xDF592AAF6851BA0D) & 0xFFFFFFFFFFFFFFFF
        tmphash = (
            ((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)
        ) & 0xFFFFFFFFFFFFFFFF
        return tmphash

    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if fr3_cmd._packed_fingerprint is None:
            fr3_cmd._packed_fingerprint = struct.pack(
                ">Q", fr3_cmd._get_hash_recursive([])
            )
        return fr3_cmd._packed_fingerprint

    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", fr3_cmd._get_packed_fingerprint())[0]
