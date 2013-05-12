"""\
Object writers
Copyright (C) 2007 BlueBotics SA
"""

from struct import pack

from Los.Serializers import defaultRegistry

__all__ = ["StreamWriter"]


class StreamWriter(object):
    """Object writer for streams"""
    def __init__(self, out, registry=None):
        self.out = out
        self.serializerRegistry = registry or defaultRegistry
        
    def write(self, *args):
        """Write raw data to output."""
        out = self.out
        for arg in args:
            out.write(arg)
    
    def writeBoolean(self, *args):
        """Write boolean values to output."""
        out = self.out
        for arg in args:
            out.write(chr(bool(arg)))
    
    def writeInt(self, size, *args):
        """Write sized integer values to output."""
        out = self.out
        for arg in args:
            for i in range(size):
                out.write(chr(arg & 0xff))
                arg >>= 8
    
    def writeInt8(self, *args):
        """Write 8-bit integer values to output."""
        out = self.out
        for arg in args:
            out.write(chr(arg & 0xff))

    def writeInt16(self, *args):
        """Write 16-bit integer values to output."""
        self.writeInt(2, *args)
        
    def writeInt32(self, *args):
        """Write 32-bit integer values to output."""
        self.writeInt(4, *args)
        
    def writeInt64(self, *args):
        """Write 64-bit integer values to output."""
        self.writeInt(8, *args)
        
    def writeFloat32(self, *args):
        """Write 32-bit float values to output."""
        self.out.write(pack("%df" % len(args), *args))
        
    def writeFloat64(self, *args):
        """Write 64-bit float values to output."""
        self.out.write(pack("%dd" % len(args), *args))

    writeLength = writeInt32
    
    def writeString(self, *args):
        """Write string values to output."""
        out = self.out
        for arg in args:
            self.writeLength(len(arg))
            out.write(arg)

    def writeType(self, serializer):
        """Write a type code to output."""
        self.out.write(chr(serializer.id & 0xff))
        
    def writeObject(self, *args):
        """Write objects to output."""
        for arg in args:
            serializer = self.serializerRegistry.fromObject(arg)
            self.writeType(serializer)
            serializer.write(self, arg)
    
    def writeArray(self, arg):
        """Write an array of objects to output."""
        self.writeLength(len(arg))
        self.writeObject(*arg)
        
