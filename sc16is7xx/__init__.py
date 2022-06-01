# Base driver for sc16is7xx
# Copyright (c) 2022 Petr Kracik

# Inspired by
# https://github.com/SandboxElectronics/UART_Bridge
# https://github.com/pbrunnen/SC16IS750


__version__ = "0.0.1"
__license__ = "MIT"
__author__ = "Petr Kracik"


class SC16IS7XX:
    DEFAULT_CRYSTAL_FREQ = 14_745_600
    DEFAULT_I2C_ADDRESS = 0x4D

    # Register, use 3 left shift ( << 3) to get actually value for i2c write
    # -- General Registers
    REG_RHR = 0x00  # Receive Holding Register (R)
    REG_THR = 0x00  # Transmit Holding Register (W)
    REG_IER = 0x01  # Interrupt Enable Register (R/W)
    REG_FCR = 0x02  # FIFO Control Register (W)
    REG_IIR = 0x02  # Interrupt Identification Register (R)
    REG_LCR = 0x03  # Line Control Register (R/W)
    REG_MCR = 0x04  # Modem Control Register (R/W)
    REG_LSR = 0x05  # Line Status Register (R)
    REG_MSR = 0x06  # Modem Status Register (R)
    REG_SPR = 0x07  # Scratchpad Register (R/W)
    REG_TCR = 0x06  # Transmission Control Register (R/W)
    REG_TLR = 0x07  # Trigger Level Register (R/W)
    REG_TXLVL = 0x08  # Transmit FIFO Level Register (R)
    REG_RXLVL = 0x09  # Receive FIFO Level Register (R)
    REG_IODIR = 0x0A  # I/O pin Direction Register (R/W)
    REG_IOSTATE = 0x0B  # I/O pin States Register (R)
    REG_IOINTENA = 0x0C  # I/O Interrupt Enable Register (R/W)
    REG_IOCONTROL = 0x0E  # I/O pins Control Register (R/W)
    REG_EFCR = 0x0F  # Extra Features Register (R/W)

    # -- Special Register Set (Requires LCR[7] = 1 & LCR != 0xBF to use)
    REG_LCR7_DLL = 0x00  # Divisor Latch LSB (R/W)
    REG_LCR7_DLH = 0x01  # Divisor Latch MSB (R/W)

    # -- Enhanced Register Set (Requires LCR = 0xBF to use)
    REG_LCR_0XBF_EFR = 0x02  # Enhanced Feature Register (R/W)
    REG_LCR_0XBF_XON1 = 0x04  # XOn Nr.1 Word (R/W)
    REG_LCR_0XBF_XON2 = 0x05  # XOff Nr.1 Word (R/W)
    REG_LCR_0XBF_XOFF1 = 0x06  # XOn Nr.2 Word (R/W)
    REG_LCR_0XBF_XOFF2 = 0x07  # XOff Nr.2 Word (R/W)


    # Parity
    PARITY_NONE = None
    PARITY_ODD = 0
    PARITY_EVEN = 1
    PARITY_FORCE_ONE = 2
    PARITY_FORCE_ZERO = 3


    def __init__(self, bus, address_or_ss=None, debug = False, crystalfreq = DEFAULT_CRYSTAL_FREQ):
        self._bus = bus
        self._crystalfreq = crystalfreq
        self._debug = debug

        if type(self._bus).__name__ == "I2C":
            self._read_reg = self._read_reg_i2c
            self._write_reg = self._write_reg_i2c
            self._addr = self.DEFAULT_I2C_ADDRESS if address_or_ss is None else address_or_ss
        elif type(self._bus).__name__ == "SPI":
            self._read_reg = self._read_reg_spi
            self._write_reg = self._write_reg_spi
            self._ss = address_or_ss
        else:
            raise ValueError("Unsupported bus {}".format(type(self._bus).__name__))


        self._reset()
        self._fifoenable(True)
        self.init()


    def _debugmsg(self, msg):
        if not self._debug:
            return

        print("{}: {}".format(type(self).__name__, msg))


    def _read_reg_i2c(self, reg, count = 1):
        val = self._bus.readfrom_mem(self._addr, reg << 3, count)
        self._debugmsg("Read i2c reg {}: value {} ({})".format(hex(reg), hex(val), bin(val)))
        return bytearray(val)


    def _write_reg_i2c(self, reg, val):
        self._debugmsg("Write i2c reg {}: value {} ({})".format(hex(reg), hex(val), bin(val)))
        self._bus.writeto_mem(self._addr, reg << 3, value)


    def _read_reg_spi(self, reg, count = 1):
        raise NotImplementedError()


    def _write_reg_spi(self, reg, value):
        raise NotImplementedError()


    def _fifoenable(self, enable):
        reg = self._read_reg(self.REG_FCR)

        if enable:
            reg[0] |= 0x01
        else:
            reg[0] &= 0xFE

        self._write_reg(self.REG_FCR, reg)


    def _reset(self):
        reg = self._read_reg(self.REG_IOCONTROL)
        reg[0] |= 0x08
        self._write_reg(self.REG_IOCONTROL, reg)


    def _setbaudrate(self, baudrate=115200):
        prescaler = None
        tmp = self._read_reg(self.REG_MCR)

        prescaler = 4 if tmp[0] & 0x80 else 1

        divisor = (self._crystalfreq / prescaler) / (baudrate * 16)

        tmplcr = self._read_reg(self.REG_LCR)

        tmplcr[0] |= 0x80  # Divisor Latch enable (bit 7) - Allow access to DLL and DHL registers
        self._write_reg(self.REG_LCR, tmplcr)

        # Write new baudrate
        self._write_reg(self.REG_LCR7_DLL, divisor & 0xFF)
        self._write_reg(self.REG_LCR7_DLH, (divisor >> 8) & 0xFF)

        tmplcr[0] &= ~0x80  # Divisor Latch disable (bit 7)
        self._write_reg(self.REG_LCR, tmplcr)

        actual_baudrate = (self._crystalfreq/prescaler)/(16*divisor)
        error = (actual_baudrate-baudrate)*1000.0/baudrate

        self._debugmsg("Desired baudrate: {}, Calculated divisor: {}, Actual baudrate: {}, Baudrate error: {}"
                .format(baudrate, divisor, actual_baudrate, error))

        return error


    def _setline(self, bits = 8, parity = PARITY_NONE, stopbits = 1):
        reg = self._read_reg(self.REG_LCR)
        reg[0] &= 0xC0  # Clear actual settings

        if bits < 5 or bits > 8:
            raise ValueError("Error data length {} is not supported. Supported 5, 6, 7, 8".format(bits))


        if stopbits not in [1, 1.5, 2]:
            raise ValueError("Stop bits {} is not supported. Supported 1, 1.5, 2".format(stopbits))


        if bits == 5:
            reg[0] |= 0x00
        elif bits == 6:
            reg[0] |= 0x01
        elif bits == 7:
            reg[0] |= 0x02
        elif bits == 8:
            reg[0] |= 0x03


        if stopbits > 1:
            reg[0] |= 0x04


        if parity == self.PARITY_NONE:
            reg[0] |= 0x00
        elif parity == self.PARITY_ODD:
            reg[0] |= 0x08
        elif parity == self.PARITY_EVEN:
            reg[0] |= 0x18
        elif parity == self.PARITY_FORCE_ONE:
            reg[0] |= 0x28
        elif parity == self.PARITY_FORCE_ZERO:
            reg[0] |= 0x38
        else:
            ValueError("Invalid parity {} Use {}.PARITY_*".format(parity, type(self).__name__))


        self._write_reg(self.REG_LCR, reg)


    def init(self, baudrate=9600, bits=8, parity=None, stop=1):
        self._setline(bits, parity, stop)
        self._setbaudrate(baudrate)


    def any(self):
        reg = self._read_reg(self.REG_RXLVL)
        return reg[0]


    def read(self, data):
        length = self.any()
        if length == 0:
            return None

        return bytes(self._read_reg(self.REG_RHR, length))


    def write(self, data):
        self._write_reg(self.REG_THR, data)
