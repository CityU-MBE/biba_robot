"""\
Tests for Los.Readers
Copyright (C) 2007 BlueBotics SA
"""

from cStringIO import StringIO
from struct import pack, unpack

from ut.Assert import *
from ut.Util import *

from Los.Readers import *
from Los.Serializers import *
from Los.Types import *


def assertIdentical(lhs, rhs):
    """Check that lhs is equal and has same type as rhs."""
    __hide_traceback = True
    assertEqual(lhs, rhs)
    assertEqual(type(lhs), type(rhs))
    if isinstance(lhs, list):
        for (lhsi, rhsi) in zip(lhs, rhs):
            assertEqual(type(lhsi), type(rhsi))
    elif isinstance(lhs, dict):
        for key in lhs:
            assertEqual(type(lhs[key]), type(rhs[key]))


class StreamReaderTest(object):
    """Tests for StreamReader"""
    def strToArray(self, s):
        return [ord(each) for each in s]
        
    def makeReader(self, data):
        return StreamReader(StringIO("".join(chr(each) for each in data)))

    def readObject(self, data):
        reader = self.makeReader(data)
        return reader.readObject()
        
    def testBoolean(self):
        """Reading booleans"""
        reader = self.makeReader([0x00])
        assertIdentical(False, reader.readBoolean())
        reader = self.makeReader([0x01])
        assertIdentical(True, reader.readBoolean())
        
    def testInt8(self):
        """Reading 8-bit integer values"""
        reader = self.makeReader([0x12])
        assertIdentical(Int8(0x12), reader.readInt8())
        reader = self.makeReader([0xfe])
        assertIdentical(Int8(-2), reader.readInt8())
        reader = self.makeReader([0x80])
        assertIdentical(Int8(-(1 << 7)), reader.readInt8())
        
    def testInt16(self):
        """Reading 16-bit integer values"""
        reader = self.makeReader([0x34, 0x12])
        assertIdentical(Int16(0x1234), reader.readInt16())
        reader = self.makeReader([0xfe, 0xff])
        assertIdentical(Int16(-2), reader.readInt16())
        reader = self.makeReader([0x00, 0x80])
        assertIdentical(Int16(-(1 << 15)), reader.readInt16())

    def testInt32(self):
        """Reading 32-bit integer values"""
        reader = self.makeReader([0x78, 0x56, 0x34, 0x12])
        assertIdentical(Int32(0x12345678), reader.readInt32())
        reader = self.makeReader([0xfe, 0xff, 0xff, 0xff])
        assertIdentical(Int32(-2), reader.readInt32())
        reader = self.makeReader([0x00, 0x00, 0x00, 0x80])
        assertIdentical(Int32(-(1 << 31)), reader.readInt32())

    def testInt64(self):
        """Reading 64-bit integer values"""
        reader = self.makeReader([0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01])
        assertIdentical(Int64(0x0123456789abcdef), reader.readInt64())
        reader = self.makeReader([0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff])
        assertIdentical(Int64(-2), reader.readInt64())
        reader = self.makeReader([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80])
        assertIdentical(Int64(-(1 << 63)), reader.readInt64())

    def testFloat32(self):
        """Reading 32-bit float values"""
        value = unpack("f", pack("f", 123.456))[0]
        reader = self.makeReader(self.strToArray(pack("f", value)))
        assertIdentical(Float32(value), reader.readFloat32())
        value = unpack("f", pack("f", -12.34e21))[0]
        reader = self.makeReader(self.strToArray(pack("f", value)))
        assertIdentical(Float32(value), reader.readFloat32())
        
    def testFloat64(self):
        """Reading 64-bit float values"""
        value = unpack("d", pack("d", 123.456))[0]
        reader = self.makeReader(self.strToArray(pack("d", value)))
        assertIdentical(Float64(value), reader.readFloat64())
        value = unpack("d", pack("d", -12.3456e78))[0]
        reader = self.makeReader(self.strToArray(pack("d", value)))
        assertIdentical(Float64(value), reader.readFloat64())
        
    def testString(self):
        """Reading strings"""
        reader = self.makeReader([0x04, 0x00, 0x00, 0x00] + self.strToArray("test"))
        assertIdentical("test", reader.readString())
        reader = self.makeReader([0x00, 0x00, 0x00, 0x00])
        assertIdentical("", reader.readString())

    def testVoidObject(self):
        """Reading Void objects"""
        assertIdentical(None, self.readObject([idVoid]))

    def testBooleanObject(self):
        """Reading Boolean objects"""
        assertIdentical(False, self.readObject([idBoolean, 0x00]))
        assertIdentical(True, self.readObject([idBoolean, 0x01]))

    def testBooleanArray(self):
        """Reading Boolean arrays"""
        assertIdentical(BooleanArray([False, True, False]),
            self.readObject([idBooleanArray, 0x03, 0x00, 0x00, 0x00, 0x02]))
        assertIdentical(BooleanArray([False, True, False, True, True, False, False, False,
                False, False, True, True, True, True]),
            self.readObject([idBooleanArray, 0x0e, 0x00, 0x00, 0x00, 0x1a, 0x3c]))
        
    def testInt8Object(self):
        """Reading Int8 objects"""
        assertIdentical(Int8(0x12), self.readObject([idInt8, 0x12]))
        assertIdentical(Int8(-2), self.readObject([idInt8, 0xfe]))

    def testInt8Array(self):
        """Reading Int8 arrays"""
        assertIdentical(Int8Array([Int8(0x12), Int8(0x34)]),
            self.readObject([idInt8Array, 0x02, 0x00, 0x00, 0x00, 0x12, 0x34]))
        
    def testInt16Object(self):
        """Reading Int16 objects"""
        assertIdentical(Int16(0x1234), self.readObject([idInt16, 0x34, 0x12]))
        assertIdentical(Int16(-2), self.readObject([idInt16, 0xfe, 0xff]))

    def testInt16Array(self):
        """Reading Int16 arrays"""
        assertIdentical(Int16Array([Int16(0x1234), Int16(0x5678)]),
            self.readObject([idInt16Array, 0x02, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56]))
        
    def testInt32Object(self):
        """Reading Int32 objects"""
        assertIdentical(Int32(0x12345678), self.readObject([idInt32, 0x78, 0x56, 0x34, 0x12]))
        assertIdentical(Int32(-2), self.readObject([idInt32, 0xfe, 0xff, 0xff, 0xff]))

    def testInt32Array(self):
        """Reading Int32 arrays"""
        assertIdentical(Int32Array([Int32(0x01234567), Int32(0x76543210)]),
            self.readObject([idInt32Array, 0x02, 0x00, 0x00, 0x00, 
                0x67, 0x45, 0x23, 0x01, 0x10, 0x32, 0x54, 0x76]))
        
    def testInt64Object(self):
        """Reading Int64 objects"""
        assertIdentical(Int64(0x0123456789abcdef), 
            self.readObject([idInt64, 0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01]))
        assertIdentical(Int64(-2),
            self.readObject([idInt64, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]))

    def testInt64Array(self):
        """Reading Int64 arrays"""
        assertIdentical(Int64Array([Int64(0x0123456789abcdef)]),
            self.readObject([idInt64Array, 0x01, 0x00, 0x00, 0x00, 
                0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01]))
        
    def testFloat32Object(self):
        """Reading Float32 objects"""
        value = unpack("f", pack("f", 123.456))[0]
        assertIdentical(Float32(value), 
            self.readObject([idFloat32] + self.strToArray(pack("f", value))))
        
    def testFloat32Array(self):
        """Reading Float32 arrays"""
        a = unpack("f", pack("f", 123.456))[0]
        b = unpack("f", pack("f", -4.3e21))[0]
        assertIdentical(Float32Array([Float32(a), Float32(b)]),
            self.readObject([idFloat32Array, 0x02, 0x00, 0x00, 0x00]
                + self.strToArray(pack("f", a) + pack("f", b))))
        
    def testFloat64Object(self):
        """Reading Float64 objects"""
        value = unpack("d", pack("d", 123.456))[0]
        assertIdentical(Float64(value), 
            self.readObject([idFloat64] + self.strToArray(pack("d", value))))
        
    def testFloat64Array(self):
        """Reading Float64 arrays"""
        a = unpack("d", pack("d", 123.456))[0]
        b = unpack("d", pack("d", -4.3e210))[0]
        assertIdentical(Float64Array([Float64(a), Float64(b)]),
            self.readObject([idFloat64Array, 0x02, 0x00, 0x00, 0x00]
                + self.strToArray(pack("d", a) + pack("d", b))))
        
    def testStringObject(self):
        """Reading string objects"""
        input = "test string"
        assertIdentical(String(input),
            self.readObject([idString, 0x0b, 0x00, 0x00, 0x00]
                + self.strToArray(input)))

    def testStringArray(self):
        """Reading string arrays"""
        a = "A string"
        b = "Another string"
        assertIdentical(StringArray([String(a), String(b)]),
            self.readObject([idStringArray, 0x02, 0x00, 0x00, 0x00,
                0x08, 0x00, 0x00, 0x00] + self.strToArray(a)
                + [0x0e, 0x00, 0x00, 0x00] + self.strToArray(b)))

    def testObjectArray(self):
        """Reading object arrays"""
        assertIdentical([Int8(0x12), Int16(0x3456), Int8(0x78)],
            self.readObject([idArray, 0x03, 0x00, 0x00, 0x00,
                idInt8, 0x12, idInt16, 0x56, 0x34, idInt8, 0x78]))
        assertIdentical([""],
            self.readObject([idArray, 0x01, 0x00, 0x00, 0x00,
                idString, 0x00, 0x00, 0x00, 0x00]))

    def testCall(self):
        """Reading Call objects"""
        assertIdentical(Call("name", [Int8(0x12), Int16(0x3456)]),
            self.readObject([idCall, 0x04, 0x00, 0x00, 0x00] + self.strToArray("name")
                + [0x02, 0x00, 0x00, 0x00, idInt8, 0x12, idInt16, 0x56, 0x34]))
        assertIdentical(Call("login", ["", ""]),
            self.readObject([idCall, 0x05, 0x00, 0x00, 0x00] + self.strToArray("login")
                + [0x02, 0x00, 0x00, 0x00, idString, 0x00, 0x00, 0x00, 0x00, idString, 0x00, 0x00, 0x00, 0x00]))

    def testCallResult(self):
        """Reading CallResult objects"""
        assertIdentical(CallResult(Int8(0x12)),
            self.readObject([idCallResult, idInt8, 0x12]))

    def testCallException(self):
        """Reading CallException objects"""
        assertIdentical(CallException("name", "message", Int16(0x1234)),
            self.readObject([idCallException, 0x04, 0x00, 0x00, 0x00] + self.strToArray("name")
                + [0x07, 0x00, 0x00, 0x00] + self.strToArray("message")
                + [idInt16, 0x34, 0x12]))

    def testStruct(self):
        """Reading structures"""
        assertIdentical(Struct(key=Int8(0x12), key2=Int16(0x3456), longkey=Int8(0x78)),
            self.readObject([idStruct, 0x03, 0x00, 0x00, 0x00]
                + [0x03, 0x00, 0x00, 0x00] + self.strToArray("key") + [idInt8, 0x12]
                + [0x04, 0x00, 0x00, 0x00] + self.strToArray("key2") + [idInt16, 0x56, 0x34]
                + [0x07, 0x00, 0x00, 0x00] + self.strToArray("longkey") + [idInt8, 0x78]))

