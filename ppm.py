import numpy as np


class BitStream:
    def __init__(self, ifile: str, ofile: str):
        """Create a BitStream

        Parameters
        ----------
        ifile: str
            Name of input file
        ofile: str
            Name of output file
        """
        self.ifile = np.fromfile(ifile, dtype=np.uint8)
        self.file_size = len(self.ifile)
        self.ibits = np.unpackbits(self.ifile, bitorder="little")
        self.bits_count = self.file_size * 8
        self.ibyte_idx = 0
        self.bits_in_ibuffer = 0

        self.ofp = open(ofile, "wb")
        self.buffer_size = 64
        self.bits_in_obuffer = 0
        self.obuffer = np.zeros(self.buffer_size, dtype=np.uint8)

    def read_bit(self) -> np.uint8:
        """Read 1 bit from ifile

        Returns
        -------
        bit: np.uint8
            Next bit from ifile
        """
        if self.bits_in_ibuffer >= self.bits_count:
            return np.array([], dtype=np.uint8)
        bit = self.ibits[self.bits_in_ibuffer]
        self.bits_in_ibuffer += 1
        return bit

    def write_bit(self, bit: np.uint8):
        """Write 1 bit to ofile

        Parameters
        ----------
        bit: dtype=np.uint8
            1 bit to write to ofile
        """
        self.obuffer[self.bits_in_obuffer] = bit
        self.bits_in_obuffer += 1
        if self.bits_in_obuffer == self.buffer_size:
            self.bits_in_obuffer = 0
            self.ofp.write(np.packbits(self.obuffer, bitorder="little").tobytes())

    def read_byte(self) -> np.uint8:
        """Read 1 symbol (1 byte) from ifile

        Returns
        -------
        byte: np.uint8
            Next symbol (byte) from ifile
        """
        if self.ibyte_idx >= self.file_size:
            return np.array([], dtype=np.uint8)
        byte = self.ifile[self.ibyte_idx]
        self.ibyte_idx += 1
        return byte

    def write_byte(self, byte: np.uint8):
        """Write 1 symbol (byte) to ofile

        Parameters
        ----------
        byte : np.uint8
            1 symbol (byte) to write to ofile
        """
        self.ofp.write(byte)

    def close(self):
        """Write last bits from obuffer to ofile and close ifile, ofile"""
        if self.bits_in_obuffer > 0:
            self.obuffer[self.bits_in_obuffer :] = 0
            n_bytes = self.bits_in_obuffer // 8
            self.ofp.write(
                np.packbits(self.obuffer, bitorder="little")[: n_bytes].tobytes()
            )
        self.ofp.close()


def compress_ppm(ifile : str, ofile : str):
    """PUT YOUR CODE HERE
       implement a ppm algorithm for compression
    Parameters
    ----------
    ifile: str
        Name of input file
    ofile: str
        Name of output file
    """
    
    ### This is an implementation of simple copying
    bitstream = BitStream(ifile, ofile)
    bit = 0
    while (bit := bitstream.read_bit()).size:
        bitstream.write_bit(bit)
    bitstream.close()


def decompress_ppm(ifile : str, ofile : str):
    """PUT YOUR CODE HERE
       implement a ppm algorithm for decompression
    Parameters
    ----------
    ifile: str
        Name of input file
    ofile: str
        Name of output file
    """

    ### This is an implementation of simple copying
    bitstream = BitStream(ifile, ofile)
    byte = 0
    while (byte := bitstream.read_byte()).size:
        bitstream.write_byte(byte)
    bitstream.close()
