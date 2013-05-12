"""\
Tests for Los.BStream
Copyright (C) 2008 BlueBotics SA
"""

from cStringIO import StringIO
from zlib import decompress, compress, crc32

from ut.Assert import *
from ut.Util import *

from Los.BStream import *
from Los.BStream import eohMarker, float64, magicMarker


def intToStr(value, length):
    """Serialize an integer to a string of the given length."""
    return "".join(chr((value >> (8 * i)) & 0xff) for i in range(length))


def strToInt(data):
    """Deserialize an integer from a string."""
    return sum(ord(c) << (8 * i) for (i, c) in enumerate(data))
    

class WriterTest(object):
    """Tests for data recorder"""
    def testIntToStr(self):
        """Serialize an integer to a string"""
        assertEqual("\x34\x12", intToStr(0x1234, 2))
        assertEqual("\x78\x56\x34\x12", intToStr(0x12345678, 4))
        
    def testEmpty(self):
        """Create an empty stream"""
        out = StringIO()
        recorder = Writer(output=out)
        recorder.flush()
        
        headerData = compress("")
        assertEqual(
            magicMarker
            + intToStr(len(headerData), 4)
            + intToStr(crc32(headerData), 4)
            + headerData
            + intToStr(eohMarker, 4)
            , out.getvalue())

    def testHeaderOnly(self):
        """Create a stream with only a header"""
        out = StringIO()
        recorder = Writer(output=out)
        headers = [
            ("First header", "Value of first header"),
            ("Header 2", "Another value"),
        ]
        for header in headers:
            recorder.addHeader(*header)
        recorder.flush()
        
        headerData = compress("".join(
            intToStr(len(k), 4) + k + intToStr(len(v), 4) + v
            for (k, v) in headers))
            
        assertEqual(
            magicMarker
            + intToStr(len(headerData), 4)
            + intToStr(crc32(headerData), 4)
            + headerData
            + intToStr(eohMarker, 4)
            , out.getvalue())

    def testDataOnly(self):
        """Create a stream with only data blocks"""
        out = StringIO()
        recorder = Writer(output=out)
        blocks = [
            (12.34, "First data block"),
            (56.78, "Another data block"),
        ]
        for block in blocks:
            recorder.addBlock(*block)
        recorder.flush()
        
        headerData = compress("")
        blockData = [compress(data) for (_, data) in blocks]
        blockData = "".join(
            intToStr(len(data), 4)
            + intToStr(crc32(float64.pack(time) + data), 4)
            + float64.pack(time)
            + data
            + intToStr(len(data), 4)
            for ((time, _), data) in zip(blocks, blockData))
        
        assertEqual(
            magicMarker
            + intToStr(len(headerData), 4)
            + intToStr(crc32(headerData), 4)
            + headerData
            + intToStr(eohMarker, 4)
            + blockData
            , out.getvalue())

    def testHeaderAndData(self):
        """Create a stream with both header and data"""
        out = StringIO()
        recorder = Writer(output=out)
        headers = [
            ("Version", "1.2.3"),
            ("Configuration", "Standalone"),
        ]
        for header in headers:
            recorder.addHeader(*header)
        blocks = [
            (98.76, "Outgoing data"),
            (54.32, "Incoming data"),
        ]
        for block in blocks:
            recorder.addBlock(*block)
        recorder.flush()
        
        headerData = compress("".join(
            intToStr(len(k), 4) + k + intToStr(len(v), 4) + v
            for (k, v) in headers))
        blockData = [compress(data) for (_, data) in blocks]
        blockData = "".join(
            intToStr(len(data), 4)
            + intToStr(crc32(float64.pack(time) + data), 4)
            + float64.pack(time)
            + data
            + intToStr(len(data), 4)
            for ((time, _), data) in zip(blocks, blockData))
            
        assertEqual(
            magicMarker
            + intToStr(len(headerData), 4)
            + intToStr(crc32(headerData), 4)
            + headerData
            + intToStr(eohMarker, 4)
            + blockData
            , out.getvalue())

    def testHeaderAfterData(self):
        """Adding a header after data blocks"""
        out = StringIO()
        recorder = Writer(output=out)
        recorder.addBlock(12.34, "Data block")
        assertRaises(TypeError, recorder.addHeader, "A header", "Header data")


class ReaderTest(object):
    """Tests for data player"""
    def testStrToInt(self):
        """Deserialize an integer from a string"""
        assertEqual(0x1234, strToInt("\x34\x12"))
        assertEqual(0x12345678, strToInt("\x78\x56\x34\x12"))
        
    def testInvalidMagic(self):
        """Reading a stream with an invalid magic marker"""
        inp = StringIO("Invalid_\x00\x00\x00\x00")
        assertRaises(IOError, Reader, input=inp)
        
    def testEmptyLoopback(self):
        """Loopback-reading from an empty stream"""
        out = StringIO()
        recorder = Writer(output=out)
        recorder.flush()
        data = out.getvalue()
        recorder.close()
        
        inp = StringIO(data)
        player = Reader(input=inp)
        assertEqual([], player.getHeaders())

    def testHeaderOnlyLoopback(self):
        """Loopback-reading from a stream with only headers"""
        out = StringIO()
        recorder = Writer(output=out)
        headers = [
            ("First header", "Value of first header"),
            ("Header 2", "Another value"),
        ]
        for header in headers:
            recorder.addHeader(*header)
        recorder.flush()
        data = out.getvalue()
        recorder.close()
        
        inp = StringIO(data)
        player = Reader(input=inp)
        assertEqual(headers, player.getHeaders())

    def testDataOnlyLoopback(self):
        """Loopback-reading from a stream with only data blocks"""
        out = StringIO()
        recorder = Writer(output=out)
        blocks = [
            (12.34, "First data block"),
            (56.78, "Another data block"),
        ]
        for block in blocks:
            recorder.addBlock(*block)
        recorder.flush()
        data = out.getvalue()
        recorder.close()
        
        inp = StringIO(data)
        player = Reader(input=inp)
        assertEqual(blocks, list(player))

