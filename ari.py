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
            return None
            # return np.array([], dtype=np.uint8)
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
        # print(self.obuffer)
        
        self.bits_in_obuffer += 1
        # print(self.obuffer)
        if self.bits_in_obuffer == self.buffer_size:
            self.bits_in_obuffer = 0
            # print(self.obuffer)
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
        # print(byte)
        self.ofp.write(byte)

    def close(self):
        """Write last bits from obuffer to ofile and close ifile, ofile"""
        # print(self.bits_in_obuffer)
        if self.bits_in_obuffer > 0:
            # print("зашел")
            self.obuffer[self.bits_in_obuffer :] = 0
            n_bytes = self.bits_in_obuffer // 8 + 1
            # print(self.obuffer)
            # print(np.packbits(self.obuffer, bitorder="little")[: n_bytes])
            # self.ofp.write(np.uint8(90))

            self.ofp.write(
                np.packbits(self.obuffer, bitorder="little")[: n_bytes].tobytes()
            )

            # self.ofp.write(
            #     np.packbits(self.obuffer, bitorder="big")[: n_bytes].tobytes()
            # )
        self.ofp.close()


class FrequencyTable:
    def __init__(self):
        """Create frequency table
        """
        self.table = {"All": 0}
        self.fullTable = []
        pass


    def update(self, byte : np.uint8):
        """Use 1 byte (symbol) to update frequency table

        Parameters
        ----------
        byte : np.uint8
            1 byte (symbol)
        """
        if byte in self.table:
            self.table[byte] += 1
        else:
            self.table[byte] = 1
        self.table['All'] += 1
        pass

    def createTable(self):
        sorted_items = sorted(self.table.items(), key=lambda item: item[1], reverse=True)
        end = 0
        for key, value in sorted_items:
            if key != 'All':
                end += value
                self.fullTable.append([key, value, end])
        self.saveTable()

    def getLen(self):
        # print((self.fullTable[-1])[2])
        return (self.fullTable[-1])[2]

    def saveTable(self, ofile = 'table.txt'):
        ofp = open(ofile, "w")
        for item in self.fullTable:
            ofp.write(str(item[0]) + ',' + str(item[1]) + ',' + str(item[2]) + '\n')

    def loadTable(self, ifile = 'table.txt'):
        ifp = open(ifile, "r")
        for line in ifp:
            line = line.strip()
            elements = line.split(',')
            self.fullTable.append([int(element) for element in elements])




class ArithmeticCompressor:
    def __init__(self, bitstream : BitStream, frequency_table : FrequencyTable):
        """Create arithmetic compressor, initialize all parameters

        Parameters
        ----------
        bitstream : BitStream
            bitstream to read/write bits/bytes
        frequency_table : FrequencyTable
            Frequency table for arithmetic compressor
        """
        self.bitstream = bitstream
        self.frequency_table = frequency_table
        self.l_last = 0
        self.h_last = 65535
        self.delitel = self.frequency_table.getLen()
        self.First_qtr = (self.h_last + 1)/4 # = 16384
        self.Half = self.First_qtr * 2 # = 32768
        self.Third_qtr = self.First_qtr * 3 # = 49152
        self.bits_to_follow = 0 


    def encode_byte(self, byte : np.uint8, bitstream: BitStream):
        """Encode 1 byte (symbol) using arithmetic encoding algorithm

        Parameters
        ----------
        byte : np.uint8
            1 byte (symbol) to encode
        """
        pos_last, pos = findSimbol(self.frequency_table.fullTable, byte)
        # print('позиции: ', pos_last, pos)
        l_new = self.l_last + pos_last * (self.h_last - self.l_last + 1)/self.delitel
        h_new = self.l_last + pos * (self.h_last - self.l_last + 1)/self.delitel - 1
        # print('-----------------------------')
        # print(l_new, h_new)
        while 1:
            if h_new < self.Half:
                self.bits_to_follow = bits_plus_follow(bitstream, 0, self.bits_to_follow)
                # print(l_new, h_new, '--start')
            elif l_new >= self.Half:
                self.bits_to_follow = bits_plus_follow(bitstream, 1, self.bits_to_follow)
                l_new -= self.Half
                h_new -= self.Half
                # print(l_new, h_new, '--half')
            elif ((l_new >= self.First_qtr) and (h_new < self.Third_qtr)) :
                # print(self.bits_to_follow)
                self.bits_to_follow += 1
                l_new -= self.First_qtr
                h_new -= self.First_qtr
                # print(l_new, h_new, '--midle')
            else:
                break

            l_new += l_new
            h_new += h_new + 1
        # print(l_new, h_new, 'nen')
        self.l_last = l_new
        self.h_last = h_new


    def decode_byte(self, byte: int, bitstream: BitStream):
        """Decode 1 byte (symbol) using arithmetic decoding algorithm

        Returns
        -------
        byte : np.uint8
            1 decoded byte (symbol)
        """
        low, h, c = findSimbol_decode(self.frequency_table.fullTable, byte, self.l_last, self.h_last, self.delitel)
        bitstream.write_byte(np.uint8(c))

        l_new = self.l_last + low
        h_new = self.l_last + h
        # print("Новые границы", low, h, chr(c), l_new, h_new, byte)
        while(1):
            if l_new >= self.Half:
                byte -= self.Half
                l_new -= self.Half
                h_new -= self.Half
                # print(l_new, h_new, byte, '--half')
            elif ((l_new >= self.First_qtr) and (h_new < self.Third_qtr)) :
                byte -= self.First_qtr
                l_new -= self.First_qtr
                h_new -= self.First_qtr
                # print(l_new, h_new, byte, '--First_qtr')
            elif h_new < self.Half:
                pass
            else:
                break
            l_new += l_new
            h_new += h_new + 1
            byte += byte
            if (bit := bitstream.read_bit()) is not None:
                # print("Зашел!!!")
                byte += int(bit)
            # print("in cycle", l_new, h_new, byte)
        # bitstream.write_byte(byte)
        self.l_last = l_new
        self.h_last = h_new
        return byte

    def end(self, bitstream: BitStream):
        self.bits_to_follow += 1 
        if self.l_last < self.First_qtr:
            bits_plus_follow(bitstream, 0, self.bits_to_follow)
        else:
            bits_plus_follow(bitstream, 1, self.bits_to_follow)

def findSimbol(table: list, byte: np.uint8):
    result = (-1, -1)
    last = 0
    for item in table:
        if item[0] == byte:
            result = (last, item[2])
            break
        last = item[2]
    return result

def findSimbol_decode(table: list, byte: int, l_last, h_last, delitel):
    # print(byte)
    byte -= l_last
    pos_last = 0
    # print(byte, delitel)
    for item in table:
        pos = item[2]
        low = pos_last * (h_last - l_last + 1)/delitel
        h = pos * (h_last - l_last + 1)/delitel - 1
        # print(low, h, byte)
        if (h > byte) and (low <= byte):
            # print(low, h, byte, chr(item[0]))
            return (low, h, item[0])
        pos_last = pos
    return(0, 65535, 0)


def bits_plus_follow (bitstream: BitStream, bit: np.uint8, bits_to_follow: int):
    bitstream.write_bit(bit)
    while bits_to_follow > 0:
        # print('зашел!', not bit)
        bitstream.write_bit(not bit)
        bits_to_follow-=1
    return bits_to_follow


def compress_ari(ifile : str, ofile : str):
    """PUT YOUR CODE HERE
       implement an arithmetic encoding algorithm for compression
    Parameters
    ----------
    ifile: str
        Name of input file
    ofile: str
        Name of output file
    """

    ### This is an implementation of simple copying
    ofile = "code.txt"
    bitstream = BitStream(ifile, ofile)
    table = FrequencyTable()
    byte = 0
    while (byte := bitstream.read_byte()).size:
        table.update(byte)
    table.createTable()
    ari = ArithmeticCompressor(bitstream, table)
    bitstream.close()

    bitstream = BitStream(ifile, ofile)
    byte = 0
    while (byte := bitstream.read_byte()).size:
        ari.encode_byte(byte, bitstream)
    ari.end(bitstream)
    bitstream.close()
    # print('---------------')
    # for item in ari.frequency_table.fullTable:
        # print(f"Символ: {chr(item[0])}, Вес: {item[1]}, Позиция: {item[2]}")
    # print('---------------')


def decompress_ari(ifile : str, ofile : str):
    """PUT YOUR CODE HERE
       implement an arithmetic decoding algorithm for decompression
    Parameters
    ----------
    ifile: str
        Name of input file
    ofile: str
        Name of output file
    """

    ### This is an implementation of simple copying
    ifile = 'code.txt'
    bitstream = BitStream(ifile, ofile)
    table = FrequencyTable()
    table.loadTable()
    ari = ArithmeticCompressor(bitstream, table)

    byte = 0
    bits_min = 16
    bits_first = bits_min
    if(bitstream.bits_count < bits_min):
        bits_first = bitstream.bits_count
    bits_other = bits_min - bits_first
    for i in range(bits_first):
        byte += byte + int(bitstream.read_bit())
        # print(f'иттерация {i}: {byte}')

    for i in range(bits_other):
        byte += byte
    # if
    for i in range(table.fullTable[-1][2]):
        byte = ari.decode_byte(byte, bitstream)
    # while (byte := bitstream.read_byte()).size:
    #     ari.decode_byte(byte, bitstream)
    bitstream.close()
