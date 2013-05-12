"""\
Block data stream handling
Copyright (C) 2008 BlueBotics SA
"""

# TODO: Make player backwards-streamable

from cStringIO import StringIO
import os
import Queue
from struct import Struct
from threading import Thread
import time
from zlib import decompress, compress, crc32

from Los.Streams import Capture

__all__ = ["Reader", "Writer", "Recorder"]


# Size limits
maxStringLength = 1 << 16 - 1
maxHeaderLength = 1 << 20
maxBlockLength = 1 << 20

# Markers
magicMarker = "BStream\0"
eohMarker = 0xffffffff

# Field codecs
uint32 = Struct("<I")
float64 = Struct("<d")


class Writer(object):
    """A block stream data writer"""
    def __init__(self, path=None, output=None):
        self.headers = []
        self.headersWritten = False
        if path is not None:
            output = open(path, "wb")
        self.output = output
        
    def addHeader(self, key, value):
        """Add a header key/value pair to the stream."""
        if self.headersWritten:
            raise TypeError("Header added after blocks")
        if len(key) > maxStringLength:
            raise ValueError("Header key too long (%d bytes, max %d)" % (len(key), maxStringLength))
        if len(value) > maxStringLength:
            raise ValueError("Header value too long (%d bytes, max %d)" % (len(value), maxStringLength))
        self.headers.append((key, value))
        
    def addBlock(self, time, data):
        """Add a data block to the stream."""
        if not self.headersWritten:
            self._writeHeaders()
        time = float64.pack(time)
        data = compress(data)
        if len(data) > maxBlockLength:
            raise ValueError("Data block too long (%d bytes, max %d)" % (len(data), maxBlockLength))
        length = uint32.pack(len(data))
        self.output.write("".join([
            length,
            uint32.pack(crc32(data, crc32(time)) & 0xffffffff),
            time,
            data,
            length]))
        
    def flush(self):
        """Write headers if necessary and flush the output stream."""
        if not self.headersWritten:
            self._writeHeaders()
        self.output.flush()
        
    def close(self):
        """Close the stream."""
        if not self.headersWritten:
            self._writeHeaders()
        self.output.close()

    # Protected interface
    def _writeHeaders(self):
        """Write the stream headers to the output."""
        data = compress("".join(
            uint32.pack(len(key)) + key
            + uint32.pack(len(value)) + value
            for (key, value) in self.headers))
        self.output.write("".join([
            magicMarker,
            uint32.pack(len(data)),
            uint32.pack(crc32(data) & 0xffffffff),
            data,
            uint32.pack(eohMarker)]))
        self.headersWritten = True


class Reader(object):
    """A block stream data reader"""
    def __init__(self, path=None, input=None):
        if path is not None:
            input = open(path, "rb")
        self.input = input
        self.headers = None
        self._readHeaders()

    def getHeaders(self):
        if self.headers is None:
            self._readHeaders()
        return self.headers
        
    def __iter__(self):
        return self

    def next(self):
        block = self._readBlock()
        if block is None:
            raise StopIteration()
        return block
        
    def close(self):
        self.input.close()
        
    # Protected interface
    def _readHeaders(self):
        magic = self._read(len(magicMarker))
        if magic != magicMarker:
            raise IOError("Stream is not a BStream")
        length = uint32.unpack(self._read(uint32.size))[0]
        if length > maxHeaderLength:
            raise IOError("Stream header too long (%d bytes, max %d)" % (length, maxHeaderLength))
        checksum = uint32.unpack(self._read(uint32.size))[0]
        data = self._read(length)
        if checksum != crc32(data) & 0xffffffff:
            raise IOError("Bad stream header checksum")
        data = decompress(data)
        length = len(data)
        data = StringIO(data)
        headers = []
        while data.tell() < length:
            key = data.read(uint32.unpack(data.read(uint32.size))[0])
            value = data.read(uint32.unpack(data.read(uint32.size))[0])
            headers.append((key, value))
        self.headers = headers
        
    def _readBlock(self):
        if self.headers is None:
            self._readHeaders()
        prevLength = uint32.unpack(self._read(uint32.size))[0]
        lenData = self._read(uint32.size, True)
        if not lenData:
            self.input.seek(-uint32.size, os.SEEK_CUR)
            return None
        length = uint32.unpack(lenData)[0]
        checksum = uint32.unpack(self._read(uint32.size))[0]
        time = self._read(float64.size)
        if length > maxBlockLength:
            raise IOError("Data block too long (%d bytes, max %d)" % (length, maxBlockLength))
        data = self._read(length)
        if checksum != crc32(data, crc32(time)) & 0xffffffff:
            raise IOError("Bad data block checksum")
        return (float64.unpack(time)[0], decompress(data))
        
    def _read(self, count, allowEof=False):
        data = self.input.read(count)
        if not data and allowEof:
            return None
        while len(data) < count:
            newData = self.input.read(count - len(data))
            if not newData:
                raise IOError("Unexpected EOF while reading input")
            data += newData
        return data


class Recorder(object):
    """A multi-endpoint data recorder"""
    def __init__(self, headers=[]):
        self.headers = headers
        self.endpoints = []
        
    def addEndpoint(self, connection, name, type, headers=[]):
        """Add a communication endpoint to the recorder."""
        self.endpoints.append((connection, Queue.Queue(), name, type, headers))
        
    def start(self, output):
        """Start recording."""
        # Setup output writer
        self.writer = Writer(output=output)
        self.writer.addHeader("Container.Type", "BStream.Recorder")
        self.writer.addHeader("Container.Version", "1")
        for header in self.headers:
            self.writer.addHeader(*header)
        self.writer.addHeader("Endpoints", str(len(self.endpoints)))
        for (i, (_, _, name, type, headers)) in enumerate(self.endpoints):
            self.writer.addHeader("Endpoint[%d].Name" % i, name)
            self.writer.addHeader("Endpoint[%d].Type" % i, type)
            for (k, v) in headers:
                self.writer.addHeader("Endpoint[%d].%s" % (i, k), v)
        self.writer.flush()
        
        # Setup connections for capturing
        for (i, each) in enumerate(self.endpoints):
            each[0].setCapture(RecordCapture(self, i))
            
        # Start recording thread
        self.thread = Thread(target=self.writeToFile)
        self.thread.setDaemon(True)
        self.startTime = time.time()
        self.running = True
        self.thread.start()
        
    def stop(self, timeout=None):
        """Stop recording."""
        for each in self.endpoints:
            each[0].setCapture(None)
        self.running = False
        self.thread.join(timeout)
        self.writer.close()
        
    def record(self, endpoint, time, input, output):
        """Record a input / output pair on one endpoint."""
        if self.running:
            self.endpoints[endpoint][1].put((time, input, output))
        
    def writeToFile(self):
        """Write endpoint data blocks to the output."""
        items = [None] * len(self.endpoints)
        while self.running or any(items):
            # Get new items for empty slots
            for (endpoint, each) in enumerate(self.endpoints):
                if items[endpoint] is None:
                    try:
                        items[endpoint] = each[1].get(False)
                    except Queue.Empty:
                        pass
            if not any(items):
                time.sleep(0.05)
                continue
            
            # Select the earliest item
            activeItems = [(each, i) for (i, each) in enumerate(items) if each is not None]
            activeItems.sort()
            
            # Write the earliest item to the output
            ((t, input, output), endpoint) = activeItems[0]
            self.writer.addBlock(t, "".join([uint32.pack(endpoint), input, output]))
            items[endpoint] = None


class RecordCapture(Capture):
    """Capture to a recorder"""
    def __init__(self, recorder, endpoint):
        self.recorder = recorder
        self.endpoint = endpoint
        self.output = StringIO()
        self.input = StringIO()

    def startTransaction(self):
        self.output.truncate(0)
        self.input.truncate(0)
        
    def endTransaction(self):
        self.recorder.record(self.endpoint, time.time(),
            self.input.getvalue(), self.output.getvalue())
        
    def outgoing(self, data):
        self.output.write(data)
        
    def incoming(self, data):
        self.input.write(data)
        
