"""\
Object readers
Copyright (C) 2007 BlueBotics SA
"""

from struct import unpack

from Los.Serializers import defaultRegistry
from Los.Types import *

__all__ = ["StreamReader"]


class StreamReader(object):
    """Object reader for streams"""
    def __init__(self, inp, registry=None):
        self.inp = inp
        self.serializerRegistry = registry or defaultRegistry
        
    def read(self, count):
        """Read raw data from input."""
        result = self.inp.read(count)
        if len(result) < count:
            raise IOError("End of stream while reading object")
        return result
        
    def readBoolean(self, count=None):
        """Read boolean values from input."""
        if count is None:
            return Boolean(ord(self.read(1)) != 0)
        return BooleanArray(Boolean(ord(each) != 0) for each in self.read(count))
        
    def readInt(self, size):
        """Read a sized integer value from input."""
        s = self.read(size)
        value = 0
        for i in range(size):
            value |= ord(s[i]) << (i << 3)
        mask = 1 << (size << 3)
        if value & (mask >> 1):
            return -((value ^ (mask - 1)) + 1)
        return value
    
    def readIntArray(self, size, array, count, itemType):
        """Read sized integer values from input."""
        s = self.read(size * count)
        for j in range(count):
            value = 0
            for i in range(size):
                value |= ord(s[j * size + i]) << (i << 3)
            mask = 1 << (size << 3)
            if value & (mask >> 1):
                value = -((value ^ (mask - 1)) + 1)
            array.append(itemType(value))
        return array
    
    def readInt8(self, count=None):
        """Read 8-bit integer values from input."""
        if count is None:
            return Int8(self.readInt(1))
        return self.readIntArray(1, Int8Array(), count, Int8)

    def readInt16(self, count=None):
        """Read 16-bit integer values from input."""
        if count is None:
            return Int16(self.readInt(2))
        return self.readIntArray(2, Int16Array(), count, Int16)

    def readInt32(self, count=None):
        """Read 32-bit integer values from input."""
        if count is None:
            return Int32(self.readInt(4))
        return self.readIntArray(4, Int32Array(), count, Int32)

    def readInt64(self, count=None):
        """Read 64-bit integer values from input."""
        if count is None:
            return Int64(self.readInt(8))
        return self.readIntArray(8, Int64Array(), count, Int64)

    def readFloat32(self, count=None):
        """Read 32-bit float values from input."""
        if count is None:
            return Float32(unpack("f", self.read(4))[0])
        return Float32Array(Float32(each) for each in unpack("%df" % count, self.read(4 * count)))
        
    def readFloat64(self, count=None):
        """Read 64-bit float values from input."""
        if count is None:
            return Float64(unpack("d", self.read(8))[0])
        return Float64Array(Float64(each) for each in unpack("%dd" % count, self.read(8 * count)))
        
    readLength = readInt32
    
    def readString(self, count=None):
        """Read string values from input."""
        if count is None:
            return String(self.read(self.readLength()))
        return StringArray(String(self.read(self.readLength())) for each in range(count))

    def readType(self):
        """Read a type code from input and return the corresponding serializer."""
        return self.serializerRegistry.fromId(ord(self.read(1)))
        
    def readObject(self):
        """Read objects from input."""
        objType = self.readType()
        return objType.read(self)

    def readArray(self):
        """Read an array of objects from input."""
        return Array(self.readObject() for each in range(self.readLength()))
        
