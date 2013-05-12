"""\
Stream adapters
Copyright (C) 2008 BlueBotics SA
"""

from cStringIO import StringIO
from socket import SHUT_RDWR

__all__ = ["EofError", "SocketStream", "trySockOpt"]


class EofError(Exception):
    """Exception thrown on an EOF condition"""


class SocketStream(object):
    """A file-like interface for a socket"""
    def __init__(self, socket, buffer=4096, capture=None):
        self._socket = socket
        self._buffer = buffer
        self._inBuffer = []
        self._inLen = 0
        self._outBuffer = StringIO()
        self.setCapture(capture)
        
    def setCapture(self, capture):
        self._capture = capture
        
    def read(self, size=None):
        if size == 0:
            return ""
        buffer = self._inBuffer
        available = self._inLen
        while size is None or available < size:
            data = self._socket.recv(self._buffer)
            if not data:
                break
            buffer.append(data)
            available += len(data)
        if available == 0:
            raise EofError("Peer has disconnected")
        head = buffer[0]
        if size > len(head):
            head = "".join(buffer)
            del buffer[1:]
        data = head[:size]
        buffer[0] = head[size:]
        self._inLen = available - len(data)
        if self._capture is not None:
            self._capture.incoming(data)
        return data
        
    def write(self, data):
        if self._capture is not None:
            self._capture.outgoing(data)
        self._outBuffer.write(data)
        if self._outBuffer.tell() >= self._buffer:
            data = self._outBuffer.getvalue()
            self._outBuffer.truncate(0)
            self._socket.sendall(data)
        
    def flush(self):
        if self._outBuffer.tell() > 0:
            data = self._outBuffer.getvalue()
            self._outBuffer.truncate(0)
            self._socket.sendall(data)
        
    def startTransaction(self):
        if self._capture is not None:
            self._capture.startTransaction()
        
    def endTransaction(self):
        if self._capture is not None:
            self._capture.endTransaction()
        
    def close(self):
        self._socket.shutdown(SHUT_RDWR)
        self._socket.close()


def trySockOpt(sock, level, optname, value):
    """Try to set a socket option, but don't throw if it fails."""
    try:
        sock.setsockopt(level, optname, value)
    except socktet.error:
        pass


class Capture(object):
    """An interface to capture traffic on a LOS connection"""
    def startTransaction(self):
        """Report the start a transaction on the connectionn."""
        
    def endTransaction(self):
        """Report the end of a transaction on the connection."""
        
    def outgoing(self, data):
        """Report data written by the socket user."""
        
    def incoming(self, data):
        """Report data read by the socket user."""
        
