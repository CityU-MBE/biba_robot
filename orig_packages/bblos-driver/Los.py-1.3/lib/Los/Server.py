"""\
Server objects
Copyright (C) 2007 BlueBotics SA
"""

from __future__ import with_statement

import logging
import socket
import sys
import traceback
from threading import Condition, Thread

from Los.Readers import StreamReader
from Los.Streams import EofError, SocketStream, trySockOpt
from Los.Types import Call, CallResult, CallException
from Los.Writers import StreamWriter

__all__ = ["Server", "ThreadedServer"]


class Server(object):
    """Server base class"""
    def __init__(self):
        self._handlers = {}
        self.setLogger(None)
    
    def publish(self, name, handler):
        """Publish a remote call."""
        self._handlers[name] = handler
    
    def _dispatch(self, call):
        """Dispatch a call and return its result."""
        # Find handler
        handler = self._handlers.get(call.name)
        if handler is None:
            return CallException("CallNotFound", "Cannot find function", call.name)
        
        # Execute handler and return result
        try:
            return CallResult(handler(*call.arguments))
        except:
            (type, value, tb) = sys.exc_info()
            return CallException(type.__name__, str(value), 
                "".join(traceback.format_exception(type, value, tb)))
    
    def setLogger(self, logger):
        """Set a logger for logging events in the server and its connections."""
        self.log = logger or DummyLogger()
        

class ThreadedServer(Server):
    """Multi-threaded server using one thread per connection"""
    acceptPollInterval = 0.2
    
    def __init__(self, address, family=socket.AF_INET, type=socket.SOCK_STREAM, proto=0, backlog=5, timeout=None):
        super(ThreadedServer, self).__init__()
        self.address = address
        self.socketArgs = (family, type, proto)
        self.backlog = backlog
        self.timeout = timeout
        self._thread = None
        self._lock = Condition()
        self._connections = []
    
    def start(self):
        """Start the server."""
        with self._lock:
            if self._thread is not None:
                return
            self._thread = Thread(target=self._listen, name="ThreadedServer on %r" % (self.address, ))
            self._thread.setDaemon(True)
            self._thread.start()
            self._lock.wait()
    
    def stop(self, timeout=None):
        """Stop the server."""
        with self._lock:
            if self._thread is None:
                return
            (thread, self._thread) = (self._thread, None)
            self._lock.wait(timeout)
    
    def closeOpenConnections(self):
        with self._lock:
            for connection in self._connections:
                connection._stream.close()
        
    def _listen(self):
        """Listen for and dispatch new connections."""
        self.log.debug("Starting listen thread on %r" % (self.address, ))
        sock = socket.socket(*self.socketArgs)
        try:
            trySockOpt(sock, socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(self.acceptPollInterval)
            sock.bind(self.address)
            sock.listen(self.backlog)
            
            with self._lock:
                self._lock.notify()
            
            self.log.debug("Starting accept loop")
            while self._thread is not None:
                try:
                    (connection, address) = sock.accept()
                except socket.timeout:
                    pass
                else:
                    self.log.debug("Connection accepted from %r" % (address, ))
                    trySockOpt(connection, socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection.settimeout(self.timeout)
                    with self._lock:
                        self._connections.append(ThreadedConnection(self, connection, address))
        except:
            sock.close()
            self.log.exception("Exception in server thread")
        
        with self._lock:
            self._thread = None
            self._lock.notify()
    
    def _removeConnection(self, connection):
        """Remove a connection from the list of active connections."""
        with self._lock:
            self._connections.remove(connection)
        

class ThreadedConnection(object):
    """Connection for a ThreadedServer"""
    def __init__(self, server, sock, address):
        self._server = server
        self._stream = SocketStream(sock)
        self.address = address
        self._writer = StreamWriter(self._stream)
        self._reader = StreamReader(self._stream)
        self._thread = Thread(target=self.process, name="ThreadedConnection from %r" % (address, ))
        self._thread.setDaemon(True)
        self._thread.start()
    
    def process(self):
        """Process incoming requests."""
        self._server.log.debug("Starting connection thread from %r" % (self.address, ))
        exceptions = (socket.timeout, socket.error, EofError)
        try:
            while True:
                obj = self._reader.readObject()
                if obj is None:                 # Ping
                    self._server.log.debug("Received ping request")
                    self._writer.writeObject(None)
                elif isinstance(obj, Call):     # Call
                    self._server.log.debug("Received call to '%s'" % obj.name)
                    self._writer.writeObject(self._server._dispatch(obj))
                else:                           # Invalid object type
                    self._server.log.debug("Received invalid request")
                    break
                self._stream.flush()
        except exceptions:
            pass
        except:
            self._server.log.exception("Exception in connection thread from %r" % (self.address, ))
        
        self._server.log.debug("Terminating connection thread on %r" % (self.address, ))
        self._server._removeConnection(self)
        try:
            self._stream.close()
        except:
            pass
        

class DummyLogger(object):
    """A dummy logger class"""
    def __getattr__(self, name):
        return self
    
    def __call__(self, *args, **kwargs):
        return self
    
