"""\
Tests for Los.Types
Copyright (C) 2007 BlueBotics SA
"""

from ut.Assert import *
from ut.Util import *

from Los.Types import *


class CallTypesTest(object):
    """Tests for call types"""
    def testCall(self):
        """Working with Call"""
        call = Call("Module.proc", [123, 456, "string"])
        assertEqual(Call("Module.proc", [123, 456, "string"]), call)
        assertNotEqual(Call("Module.otherProc", [123, 456, "string"]), call)
        assertNotEqual(Call("Module.proc", [789, "other string"]), call)
        
    def testCallResult(self):
        """Working with CallResult"""
        result = CallResult(123)
        assertEqual(CallResult(123), result)
        
        call = Call("Module.proc", [123, 456, "string"])
        assertEqual(123, result.getValue(call))
        
    def testCallException(self):
        """Working with CallException"""
        exception = CallException("NotFound", "Could not find it", 123)
        assertEqual(CallException("NotFound", "Could not find it", 123), exception)
        assertNotEqual(CallException("NotFoundEither", "Could not find it", 123), exception)
        assertNotEqual(CallException("NotFound", "Could not find it here", 123), exception)
        assertNotEqual(CallException("NotFound", "Could not find it", 456), exception)
        
        call = Call("Module.proc", [123, 456, "string"])
        try:
            exception.getValue(call)
        except RemoteException, e:
            assertEqual("NotFound", e.name)
            assertEqual("Could not find it", e.message)
            assertEqual(123, e.data)
            assertEqual("Module.proc", e.callName)
            assertEqual([123, 456, "string"], e.callArguments)
        else:
            fail("No RemoteException raised")

    def testStruct(self):
        """Working with Struct"""
        struct = Struct(a=1, b="test")
        assertEqual(Struct(a=1, b="test"), struct)
        assertNotEqual(Struct(a=2, b="test"), struct)
        assertNotEqual(Struct(a=1, b="other test"), struct)
        assertNotEqual(Struct(a=1), struct)
        assertNotEqual(Struct(a=1, b="test", c=3.75), struct)
        assertNotEqual(Struct(ab=1, b="test"), struct)

        assertEqual(2, len(struct))        
        assertTrue("a" in struct)
        assertEqual(1, struct.a)
        assertTrue("b" in struct)
        assertEqual("test", struct.b)
        assertEqual(1, struct["a"])
        assertEqual("test", struct["b"])
        assertTrue("c" not in struct)
        
        assertEqual(Struct({"a": 1, "b": "test"}), struct)

        struct.a = 2
        struct["b"] = "second"
        
        assertEqual(2, struct["a"])
        assertEqual("second", struct.b)
        
        struct.c = 4
        assertEqual(3, len(struct))
        del struct.c
        assertEqual(2, len(struct))
        del struct["b"]
        assertEqual(1, len(struct))
        struct.clear()
        assertEqual(0, len(struct))
        
