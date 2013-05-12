"""\
Standard object types
Copyright (C) 2007 BlueBotics SA
"""

# Base types
class ArrayBase(list):
    """Base class for homogeneous array types"""
    __slots__ = []
    
    def __repr__(self):
        return "%s(%s)" % (self.__class__.__name__, super(ArrayBase, self).__repr__())

Void = type(None)
Boolean = bool
class BooleanArray(ArrayBase): __slots__ = []
class Int8(int): __slots__ = []
class Int8Array(ArrayBase): __slots__ = []
class Int16(int): __slots__ = []
class Int16Array(ArrayBase): __slots__ = []
class Int32(int): __slots__ = []
class Int32Array(ArrayBase): __slots__ = []
class Int64(long): __slots__ = []
class Int64Array(ArrayBase): __slots__ = []
class Float32(float): __slots__ = []
class Float32Array(ArrayBase): __slots__ = []
Float64 = float
class Float64Array(ArrayBase): __slots__ = []
String = str
class StringArray(ArrayBase): __slots__ = []
Array = list


class Struct(dict):
    """Name/value structure"""
    __slots__ = []
    
    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError("'%s' object has no attribute '%s'" % (self.__class__.__name__, name))
            
    def __setattr__(self, name, value):
        self[name] = value
        
    def __delattr__(self, name):
        try:
            del self[name]
        except KeyError:
            raise AttributeError("'%s' object has no attribute '%s'" % (self.__class__.__name__, name))
        
    def __cmp__(self, other):
        if self.__class__ != other.__class__:
            return cmp(self.__class__, other.__class__)
        return super(Struct, self).__cmp__(other)
        
    def __repr__(self):
        return "Struct(%s)" % ", ".join("%s=%r" % (k, v) for (k, v) in self.iteritems())


# Call-related types
class RemoteException(RuntimeError):
    """Forwarded remote exception"""
    def __init__(self, name, message, data, callName, callArguments):
        RuntimeError.__init__(self, name)
        self.name = name
        self.message = message
        self.data = data
        self.callName = callName
        self.callArguments = callArguments
    
    def __str__(self):
        return self.name + ": " + self.message
        
    def write(self, out):
        out.write(str(self) + "\n")
        if self.data is not None:
            out.write(str(self.data) + "\n")


class Call(object):
    """Procedure call"""
    def __init__(self, name, arguments=[]):
        self.name = name
        self.arguments = arguments

    def __cmp__(self, other):
        if self.__class__ != other.__class__:
            return cmp(self.__class__, other.__class__)
        return cmp((self.name, self.arguments), (other.name, other.arguments))
            
    def __repr__(self):
        return "Call(%r, %r)" % (self.name, self.arguments)


class CallResultBase(object):
    """Base class for call results"""
    def getValue(self, call):
        raise NotImplemented
        

class CallResult(CallResultBase):
    """Procedure call result"""
    def __init__(self, value):
        self.value = value
        
    def getValue(self, call):
        return self.value
    
    def __cmp__(self, other):
        if self.__class__ != other.__class__:
            return cmp(self.__class__, other.__class__)
        return cmp(self.value, other.value)

    def __repr__(self):
        return "CallResult(%r)" % self.value
        

class CallException(CallResultBase):
    """Procedure call exception"""
    def __init__(self, name, message, data=None):
        self.name = name
        self.message = message
        self.data = data
        
    def getValue(self, call):
        raise RemoteException(self.name, self.message, self.data, call.name, call.arguments)
    
    def __cmp__(self, other):
        if self.__class__ != other.__class__:
            return cmp(self.__class__, other.__class__)
        return cmp((self.name, self.message, self.data), (other.name, other.message, other.data))
            
    def __repr__(self):
        return "CallException(%r, %r, %r)" % (self.name, self.message, self.data)

