"""\
Tests for Los.Client and Los.Server
Copyright (C) 2007 BlueBotics SA
"""

import sys
import traceback

from ut.Assert import *
from ut.Util import *

from Los.Client import Connection
from Los.Server import *
from Los.Types import *


class ClientServerTest(object):
    """Tests for LOS client and server"""
    address = ("127.0.0.1", 1234)
    
    def setUp(self):
        self.server = ThreadedServer(self.address)
        self.server.setLogger(self)
        self.server.publish("object.add", self.add)
        self.server.publish("object.identity", self.identity)
        self.server.start()
        
    def tearDown(self):
        self.server.stop()

    def debug(self, msg, *args, **kwargs):
        sys.stderr.write(msg + "\n")
        
    def exception(self, msg, *args):
        sys.stderr.write(msg + "\n")
        traceback.print_exc()
        
    def add(self, *args):
        return Int32(sum(args))

    def identity(self, *args):
        return list(args)
        
    def _nop(self, *args):
        return None
    
    def testPing(self):
        """Ping between client and server"""
        c = Connection(self.address)
        c.ping()
        c.close()
        c.ping()
        c.close()
        
    def testCall(self):
        """Calling a function"""
        c = Connection(self.address)
        assertEqual(6, c.object.add(Int32(1), Int32(2), Int32(3)))
        assertEqual(["", ""], c.object.identity("", ""))
        c.close()
        
    def testException(self):
        """Calling a function raising an exception"""
        c = Connection(self.address)
        try:
            c.object.add(Int32(1), None, Int32(3))
            fail()
        except RemoteException, e:
            assertEqual("TypeError", e.name)
        c.close()
        
    def testCallNotFound(self):
        """Calling an unpublished call"""
        c = Connection(self.address)
        try:
            c.object._nop()
            fail()
        except RemoteException, e:
            assertEqual("CallNotFound", e.name)
        c.close()
        
