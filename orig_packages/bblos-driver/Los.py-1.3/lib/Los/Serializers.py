"""\
Standard object serializers
Copyright (C) 2007 BlueBotics SA
"""

import Los.Types as Types


# Standard type codes
idVoid = 0;
idBoolean = 1;
idBooleanArray = 2;
idInt8 = 3;
idInt8Array = 4;
idInt16 = 5;
idInt16Array = 6;
idInt32 = 7;
idInt32Array = 8;
idInt64 = 9;
idInt64Array = 10;
idFloat32 = 11;
idFloat32Array = 12;
idFloat64 = 13;
idFloat64Array = 14;
idString = 15;
idStringArray = 16;
idArray = 17;
idCall = 18;
idCallResult = 19;
idCallException = 20;
idStruct = 21;

# Start of user-defined type codes
idUser = 32


# Serializers
class VoidSerializer(object):
    """Serializer for Void type"""
    def read(self, reader):
        return None
        
    def write(self, writer, obj):
        pass
        

class BaseTypeSerializer(object):
    """Generic serializer for base types"""
    def __init__(self, name):
        self.readMethod = "read%s" % name
        self.writeMethod = "write%s" % name
        
    def read(self, reader):
        return getattr(reader, self.readMethod)()
        
    def write(self, writer, obj):
        getattr(writer, self.writeMethod)(obj)
        

class BaseTypeArraySerializer(object):
    """Generic serializer for base type array types"""
    def __init__(self, name):
        self.readMethod = "read%s" % name
        self.writeMethod = "write%s" % name
        
    def read(self, reader):
        length = reader.readLength()
        return getattr(reader, self.readMethod)(length)
        
    def write(self, writer, obj):
        writer.writeLength(len(obj))
        getattr(writer, self.writeMethod)(*obj)
        

class BooleanArraySerializer(object):
    """Serializer for BooleanArray type"""
    def read(self, reader):
        length = reader.readLength()
        data = reader.read((length + 7) >> 3)
        return Types.BooleanArray((ord(data[i >> 3]) & (1 << (i & 7))) != 0 for i in range(length))
        
    def write(self, writer, obj):
        data = [0] * ((len(obj) + 7) >> 3)
        for (i, each) in enumerate(obj):
            if each:
                data[i >> 3] |= 1 << (i & 7)
        writer.writeLength(len(obj))
        writer.write("".join(chr(each) for each in data))


class StructSerializer(object):
    """Serializer for Struct type"""
    def read(self, reader):
        result = Types.Struct()
        for i in range(reader.readLength()):
            name = reader.readString()
            value = reader.readObject()
            result[name] = value
        return result

    def write(self, writer, obj):
        writer.writeLength(len(obj))
        for (k, v) in obj.iteritems():
            writer.writeString(k)
            writer.writeObject(v)


class CallSerializer(object):
    """Serializer for Call type"""
    def read(self, reader):
        name = reader.readString()
        arguments = reader.readArray()
        return Types.Call(name, arguments)
        
    def write(self, writer, obj):
        writer.writeString(obj.name)
        writer.writeArray(obj.arguments)
        

class CallResultSerializer(object):
    """Serializer for CallResult type"""
    def read(self, reader):
        return Types.CallResult(reader.readObject())
        
    def write(self, writer, obj):
        writer.writeObject(obj.value)
        

class CallExceptionSerializer(object):
    """Serializer for CallException type"""
    def read(self, reader):
        name = reader.readString()
        message = reader.readString()
        data = reader.readObject()
        return Types.CallException(name, message, data)

    def write(self, writer, obj):
        writer.writeString(obj.name)
        writer.writeString(obj.message)
        writer.writeObject(obj.data)


# Registry of serializers
class SerializerRegistry(object):
    """Registry of serializers"""
    def __init__(self):
        self._fromId = {}
        self._fromType = {}
        
    def fromObject(self, obj):
        """Get the serializer from a given object."""
        return self._fromType[type(obj)]
        
    def fromId(self, id):
        """Get the serializer from a type ID."""
        return self._fromId[id]
        
    def register(self, id, type, serializer):
        """Register a serializer."""
        serializer.id = id
        self._fromId[id] = serializer
        self._fromType[type] = serializer

    def registerDefaults(self):
        """Register default serializers."""
        self.register(idVoid, Types.Void, VoidSerializer())
        self.register(idBoolean, Types.Boolean, BaseTypeSerializer("Boolean"))
        self.register(idBooleanArray, Types.BooleanArray, BooleanArraySerializer())
        for name in ["Int8", "Int16", "Int32", "Int64",
                "Float32", "Float64", "String"]:
            self.register(globals()["id%s" % name], getattr(Types, name),
                BaseTypeSerializer(name))
            self.register(globals()["id%sArray" % name], getattr(Types, name + "Array"),
                BaseTypeArraySerializer(name))
        self.register(idArray, Types.Array, BaseTypeSerializer("Array"))
        self.register(idCall, Types.Call, CallSerializer())
        self.register(idCallResult, Types.CallResult, CallResultSerializer())
        self.register(idCallException, Types.CallException, CallExceptionSerializer())
        self.register(idStruct, Types.Struct, StructSerializer())


# Default serializer registry
defaultRegistry = SerializerRegistry()
defaultRegistry.registerDefaults()

