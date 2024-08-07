"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class fr3_cmd(object):

    __slots__ = ["timestamp", "cmd"]

    __typenames__ = ["int64_t", "double"]

    __dimensions__ = [None, [7]]

    def __init__(self):
        self.timestamp = 0
        """ LCM Type: int64_t """
        self.cmd = [ 0.0 for dim0 in range(7) ]
        """ LCM Type: double[7] """

    def encode(self):
        buf = BytesIO()
        buf.write(fr3_cmd._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack('>7d', *self.cmd[:7]))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != fr3_cmd._get_packed_fingerprint():
            raise ValueError("Decode error")
        return fr3_cmd._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = fr3_cmd()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.cmd = struct.unpack('>7d', buf.read(56))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if fr3_cmd in parents: return 0
        tmphash = (0xdf592aaf6851ba0b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if fr3_cmd._packed_fingerprint is None:
            fr3_cmd._packed_fingerprint = struct.pack(">Q", fr3_cmd._get_hash_recursive([]))
        return fr3_cmd._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", fr3_cmd._get_packed_fingerprint())[0]

