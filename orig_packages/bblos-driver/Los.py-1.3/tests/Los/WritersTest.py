"""\
Tests for Los.Writers
Copyright (C) 2007 BlueBotics SA
"""

from cStringIO import StringIO
from struct import pack

from ut.Assert import *
from ut.Util import *

from Los.Serializers import *
from Los.Types import *
from Los.Writers import *


def assertContains(data, writer, msg="writer doesn't contain data", **kwargs):
    """Check that writer contains data."""
    __hide_traceback = True
    lhs = data
    if isinstance(lhs, str):
        lhs = [ord(each) for each in lhs]
    rhs = [ord(each) for each in writer.out.getvalue()]
    assert lhs == rhs, Args(msg, [("lhs", "rhs")], **kwargs)


class StreamWriterTest(object):
    """Tests for StreamWriter"""
    def strToArray(self, s):
        return [ord(each) for each in s]
        
    def writeObject(self, obj):
        writer = StreamWriter(StringIO())
        writer.writeObject(obj)
        return writer
        
    def testBoolean(self):
        """Writing booleans"""
        writer = StreamWriter(StringIO())
        writer.writeBoolean(False)
        assertContains([0], writer)

        writer = StreamWriter(StringIO())
        writer.writeBoolean(True)
        assertContains([1], writer)

        writer = StreamWriter(StringIO())
        writer.writeBoolean(123)
        assertContains([1], writer)

    def testInt8(self):
        """Writing 8-bit integers"""
        writer = StreamWriter(StringIO())
        writer.writeInt8(12)
        assertContains([12], writer)

        writer = StreamWriter(StringIO())
        writer.writeInt8(-2)
        assertContains([0xfe], writer)
        
    def testInt16(self):
        """Writing 16-bit integers"""
        writer = StreamWriter(StringIO())
        writer.writeInt16(0x1234)
        assertContains([0x34, 0x12], writer)

        writer = StreamWriter(StringIO())
        writer.writeInt16(-2)
        assertContains([0xfe, 0xff], writer)
        
    def testInt32(self):
        """Writing 32-bit integers"""
        writer = StreamWriter(StringIO())
        writer.writeInt32(0x12345678)
        assertContains([0x78, 0x56, 0x34, 0x12], writer)

        writer = StreamWriter(StringIO())
        writer.writeInt32(-2)
        assertContains([0xfe, 0xff, 0xff, 0xff], writer)
        
    def testInt64(self):
        """Writing 64-bit integers"""
        writer = StreamWriter(StringIO())
        writer.writeInt64(0x0123456789abcdef)
        assertContains([0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01], writer)

        writer = StreamWriter(StringIO())
        writer.writeInt64(-2)
        assertContains([0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff], writer)
        
    def testFloat32(self):
        """Writing 32-bit floats"""
        result = pack("f", 123.456)
        assertEqual(4, len(result))
        
        writer = StreamWriter(StringIO())
        writer.writeFloat32(123.456)
        assertContains(result, writer)
         
        writer = StreamWriter(StringIO())
        writer.writeFloat32(-12.345e21)
        assertContains(pack("f", -12.345e21), writer)

    def testFloat64(self):
        """Writing 64-bit floats"""
        result = pack("d", 123.456)
        assertEqual(8, len(result))
        
        writer = StreamWriter(StringIO())
        writer.writeFloat64(123.456)
        assertContains(result, writer)
         
        writer = StreamWriter(StringIO())
        writer.writeFloat64(-12.345e67)
        assertContains(pack("d", -12.345e67), writer)

    def testString(self):
        """Writing strings"""
        input = "This is a string"
        writer = StreamWriter(StringIO())
        writer.writeString(input)
        assertContains([0x10, 0x00, 0x00, 0x00] + [ord(each) for each in input], writer)

    def testVoidObject(self):
        """Writing Void objects"""
        assertContains([idVoid], self.writeObject(None))
        
    def testBooleanObject(self):
        """Writing Boolean objects"""
        assertContains([idBoolean, 0x00], self.writeObject(False))
        assertContains([idBoolean, 0x01], self.writeObject(True))

    def testBooleanArray(self):
        """Writing Boolean arrays"""
        assertContains([idBooleanArray, 0x04, 0x00, 0x00, 0x00, 0x06],
            self.writeObject(BooleanArray([False, True, True, False])))
        assertContains([idBooleanArray, 0x0d, 0x00, 0x00, 0x00, 0xe7, 0x09],
            self.writeObject(BooleanArray([True, True, True, False, False, True, True, True,
                    True, False, False, True, False])))
        
    def testInt8Object(self):
        """Writing Int8 objects"""
        assertContains([idInt8, 0x12], self.writeObject(Int8(0x12)))
        
    def testInt8Array(self):
        """Writing Int8 arrays"""
        assertContains([idInt8Array, 0x03, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56],
            self.writeObject(Int8Array([0x12, 0x34, 0x56])))
        
    def testInt16Object(self):
        """Writing Int16 objects"""
        assertContains([idInt16, 0x34, 0x12], self.writeObject(Int16(0x1234)))
        
    def testInt16Array(self):
        """Writing Int16 arrays"""
        assertContains([idInt16Array, 0x02, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56],
            self.writeObject(Int16Array([0x1234, 0x5678])))
        
    def testInt32Object(self):
        """Writing Int32 objects"""
        assertContains([idInt32, 0x78, 0x56, 0x34, 0x12], self.writeObject(Int32(0x12345678)))
        
    def testInt32Array(self):
        """Writing Int32 arrays"""
        assertContains([idInt32Array, 0x02, 0x00, 0x00, 0x00,
            0x78, 0x56, 0x34, 0x12, 0x01, 0xef, 0xcd, 0xab],
            self.writeObject(Int32Array([0x12345678, 0xabcdef01])))
        
    def testInt64Object(self):
        """Writing Int64 objects"""
        assertContains([idInt64, 0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01],
            self.writeObject(Int64(0x0123456789abcdef)))
        
    def testInt64Array(self):
        """Writing Int64 arrays"""
        assertContains([idInt64Array, 0x02, 0x00, 0x00, 0x00,
            0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01,
            0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12],
            self.writeObject(Int64Array([0x0123456789abcdef, 0x1234123412341234])))
        
    def testFloat32Object(self):
        """Writing Float32 objects"""
        a = pack("f", 123.456)
        assertContains([idFloat32] + self.strToArray(a),
            self.writeObject(Float32(123.456)))
        
    def testFloat32Array(self):
        """Writing Float32 arrays"""
        a = pack("f", 123.456)
        b = pack("f", 78.9e10)
        assertContains([idFloat32Array, 0x02, 0x00, 0x00, 0x00] + self.strToArray(a + b),
            self.writeObject(Float32Array([123.456, 78.9e10])))
        
    def testFloat64Object(self):
        """Writing Float64 objects"""
        a = pack("d", 123.456)
        assertContains([idFloat64] + [ord(each) for each in a],
            self.writeObject(Float64(123.456)))
        
    def testFloat64Array(self):
        """Writing Float64 arrays"""
        a = pack("d", 123.456)
        b = pack("d", 78.9e103)
        assertContains([idFloat64Array, 0x02, 0x00, 0x00, 0x00] + self.strToArray(a + b),
            self.writeObject(Float64Array([123.456, 78.9e103])))
        
    def testStringObject(self):
        """Writing string objects"""
        input = "Other string"
        assertContains([idString, 0x0c, 0x00, 0x00, 0x00] + self.strToArray(input),
            self.writeObject(input))
        
    def testStringArray(self):
        """Writing string arrays"""
        a = "One string"
        b = "Another string"
        assertContains([idStringArray, 0x02, 0x00, 0x00, 0x00,
            0x0a, 0x00, 0x00, 0x00] + self.strToArray(a)
            + [0x0e, 0x00, 0x00, 0x00] + self.strToArray(b),
            self.writeObject(StringArray([a, b])))

    def testObjectArray(self):
        """Writing object arrays"""
        assertContains([idArray, 0x03, 0x00, 0x00, 0x00,
            idInt8, 0x12, idInt16, 0x56, 0x34, idInt8, 0x78],
            self.writeObject([Int8(0x12), Int16(0x3456), Int8(0x78)]))

    def testCall(self):
        """Writing Call objects"""
        assertContains([idCall, 0x04, 0x00, 0x00, 0x00] + self.strToArray("name")
            + [0x02, 0x00, 0x00, 0x00, idInt8, 0x12, idInt8, 0x34],
            self.writeObject(Call("name", [Int8(0x12), Int8(0x34)])))
            
        assertContains([idCall, 0x05, 0x00, 0x00, 0x00] + self.strToArray("login")
            + [0x02, 0x00, 0x00, 0x00, idString, 0x00, 0x00, 0x00, 0x00, idString, 0x00, 0x00, 0x00, 0x00],
            self.writeObject(Call("login", ["", ""])))
        
    def testCallResult(self):
        """Writing CallResult objects"""
        assertContains([idCallResult, idInt16, 0x34, 0x12],
            self.writeObject(CallResult(Int16(0x1234))))

    def testCallException(self):
        """Writing CallException objects"""
        assertContains([idCallException, 0x04, 0x00, 0x00, 0x00] + self.strToArray("name")
            + [0x07, 0x00, 0x00, 0x00] + self.strToArray("message")
            + [idInt8, 0x12],
            self.writeObject(CallException("name", "message", Int8(0x12))))

    def testStruct(self):
        """Writing structures"""
        # FIXME: This assumes the ordering in a dict is repeatable
        assertContains([idStruct, 0x03, 0x00, 0x00, 0x00]
                + [0x07, 0x00, 0x00, 0x00] + self.strToArray("longkey") + [idInt8, 0x78]
                + [0x04, 0x00, 0x00, 0x00] + self.strToArray("key2") + [idInt16, 0x56, 0x34]
                + [0x03, 0x00, 0x00, 0x00] + self.strToArray("key") + [idInt8, 0x12],
            self.writeObject(Struct(key=Int8(0x12), key2=Int16(0x3456), longkey=Int8(0x78))))

