"""\
Client objects
Copyright (C) 2008 BlueBotics SA
"""

from __future__ import with_statement

from functools import wraps
import socket
from threading import Lock

from Los.Readers import StreamReader
from Los.Streams import EofError, SocketStream, trySockOpt
from Los.Types import Array, Call, CallResultBase
from Los.Writers import StreamWriter

__all__ = ["EofError", "Connection"]


def synchronized(f):
    """Synchronize method calls on the object's _lock attribute."""
    @wraps(f)
    def wrapper(*args, **kwargs):
        with args[0]._lock:
            return f(*args, **kwargs)
    return wrapper


class Connection(object):
    """Client connection for remote procedure calls"""
    def __init__(self, address, family=socket.AF_INET, type=socket.SOCK_STREAM, proto=0, 
            timeout=None, capture=None):
        self.address = address
        self.socketArgs = (family, type, proto)
        self.timeout = timeout
        self._capture = capture
        self._stream = None
        self._lock = Lock()

    @synchronized
    def open(self):
        """Open the connection."""
        self._open()
        
    @synchronized
    def close(self):
        """Close the connection."""
        self._close()
        
    @synchronized
    def isOpen(self):
        """Return True iff the connection is open."""
        return self._stream is not None

    @synchronized
    def setCapture(self, capture):
        self._capture = capture
        if self._stream is not None:
            self._stream.setCapture(capture)
        
    @synchronized
    def __call__(self, name, *args):
        """Call the given function on the remote and return the result."""
        call = Call(name, Array(args))
        result = self._sendRequest(call)
        if not isinstance(result, CallResultBase):
            self._close()
            raise IOError("Invalid result type '%s'" % result.__class__.__name__)
        return result.getValue(call)
    
    @synchronized
    def ping(self):
        """Send a keepalive."""
        result = self._sendRequest(None)
        if result is not None:
            self._close()
            raise IOError("Invalid ping reply type '%s'" % result.__class__.__name__)
        
    def __getattr__(self, name):
        """Return a proxy object for the given module or function."""
        return FunctionProxy(self, name)
    
    # Protected interface
    def _open(self):
        """Open connection to server."""
        self._close()
        sock = socket.socket(*self.socketArgs)
        trySockOpt(sock, socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.timeout)
        sock.connect(self.address)
        self._stream = SocketStream(sock, capture=self._capture)
        self._writer = StreamWriter(self._stream)
        self._reader = StreamReader(self._stream)
        
    def _close(self):
        """Close connection to server."""
        if self._stream is not None:
            del self._writer
            del self._reader
            self._stream.close()
            self._stream = None
        
    def _sendRequest(self, object):
        """Send a request object and read the reply."""
        if self._stream is None:
            self._open()
        try:
            self._stream.startTransaction()
            self._writer.writeObject(object)
            self._stream.flush()
            reply = self._reader.readObject()
            self._stream.endTransaction()
            return reply
        except:
            self._close()
            raise
        

class FunctionProxy(object):
    """Remote function proxy"""
    def __init__(self, proxy, name):
        self._proxy = proxy
        self._name = name
        
    def __getattr__(self, name):
        return FunctionProxy(self._proxy, ".".join([self._name, name]))
        
    def __call__(self, *args):
        return self._proxy(self._name, *args)

