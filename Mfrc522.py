import RPi.GPIO as GPIO
import spi


class Mfrc522:
    # Constantes
    NRSTPD = 37  # Pin 22 for SPI0 and Pin 37 for SPI1

    MAX_LEN = 16

    PCD_IDLE = 0x00
    PCD_AUTHENT = 0x0E
    PCD_RECEIVE = 0x08
    PCD_TRANSMIT = 0x04
    PCD_TRANSCEIVE = 0x0C
    PCD_RESETPHASE = 0x0F
    PCD_CALCCRC = 0x03

    PICC_REQIDL = 0x26
    PICC_REQALL = 0x52
    PICC_ANTICOLL = 0x93
    PICC_SELECTTAG = 0x93
    PICC_AUTHENT1A = 0x60
    PICC_AUTHENT1B = 0x61
    PICC_READ = 0x30
    PICC_WRITE = 0xA0
    PICC_DECREMENT = 0xC0
    PICC_INCREMENT = 0xC1
    PICC_RESTORE = 0xC2
    PICC_TRANSFER = 0xB0
    PICC_HALT = 0x50

    OK = 0
    NOTAGERR = 1
    ERROR = 2

    Reserved00 = 0x00
    CommandReg = 0x01
    CommIEnReg = 0x02
    DivlEnReg = 0x03
    CommIrqReg = 0x04
    DivIrqReg = 0x05
    ErrorReg = 0x06
    Status1Reg = 0x07
    Status2Reg = 0x08
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    WaterLevelReg = 0x0B
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    CollReg = 0x0E
    Reserved01 = 0x0F

    Reserved10 = 0x10
    ModeReg = 0x11
    TxModeReg = 0x12
    RxModeReg = 0x13
    TxControlReg = 0x14
    TxAutoReg = 0x15
    TxSelReg = 0x16
    RxSelReg = 0x17
    RxThresholdReg = 0x18
    DemodReg = 0x19
    Reserved11 = 0x1A
    Reserved12 = 0x1B
    MifareReg = 0x1C
    Reserved13 = 0x1D
    Reserved14 = 0x1E
    SerialSpeedReg = 0x1F

    Reserved20 = 0x20
    CRCResultRegM = 0x21
    CRCResultRegL = 0x22
    Reserved21 = 0x23
    ModWidthReg = 0x24
    Reserved22 = 0x25
    RFCfgReg = 0x26
    GsNReg = 0x27
    CWGsPReg = 0x28
    ModGsPReg = 0x29
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D
    TCounterValueRegH = 0x2E
    TCounterValueRegL = 0x2F

    Reserved30 = 0x30
    TestSel1Reg = 0x31
    TestSel2Reg = 0x32
    TestPinEnReg = 0x33
    TestPinValueReg = 0x34
    TestBusReg = 0x35
    AutoTestReg = 0x36
    VersionReg = 0x37
    AnalogTestReg = 0x38
    TestDAC1Reg = 0x39
    TestDAC2Reg = 0x3A
    TestADCReg = 0x3B
    Reserved31 = 0x3C
    Reserved32 = 0x3D
    Reserved33 = 0x3E
    Reserved34 = 0x3F

    serNum = []

    def __init__(self, dev='/dev/spidev1.0', spd=1000000):
        spi.openSPI(device=dev, speed=spd)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.NRSTPD, GPIO.OUT)
        GPIO.setup(self.NRSTPD, GPIO.OUT)
        GPIO.output(self.NRSTPD, 1)
        self.init(dev, spd)

    def init(self, device='/dev/spidev1.0', speed=1000000):
        GPIO.output(self.NRSTPD, 1)
        # Reset Device
        self.reset()

        # Start Device
        self.write(self.TModeReg, 0x8D)
        self.write(self.TPrescalerReg, 0x3E)
        self.write(self.TReloadRegL, 30)
        self.write(self.TReloadRegH, 0)

        self.write(self.TxAutoReg, 0x40)
        self.write(self.ModeReg, 0x3D)
        self.power_on_antenna()

    def reset(self):
        self.write(self.CommandReg, self.PCD_RESETPHASE)

    def write(self, address, value):
        spi.transfer(((address << 1) & 0x7E, value))

    def read(self, address):
        value = spi.transfer((((address << 1) & 0x7E) | 0x80, 0))
        return value[1]

    def set_bit_mask(self, reg, mask):
        temp = self.read(reg)
        self.write(reg, temp | mask)

    def clear_bit_mask(self, reg, mask):
        temp = self.read(reg)
        self.write(reg, temp & (~mask))

    def power_on_antenna(self):
        temp = self.read(self.TxControlReg)
        if (~(temp & 0x03)):
            self.set_bit_mask(self.TxControlReg, 0x03)

    def power_off_antenna(self):
        self.clear_bit_mask(self.TxControlReg, 0x03)

    # TODO verify
    def stop_crypto(self):
        self.clear_bit_mask(self.Status2Reg, 0x08)

    # TODO verify
    def dump_classic_1k(self, key, uid):
        i = 0
        while i < 64:
            status = self.auth(self.PICC_AUTHENT1A, i, key, uid)

            if status == self.OK:
                self.read_data(i)
            else:
                print("Authentication Error")
            i += 1

    def request(self, request_mode):
        status = None
        back_bits = None
        tag_type = []

        self.write(self.BitFramingReg, 0x07)

        tag_type.append(request_mode)
        (status, back_data, back_bits) = self.to_card(self.PCD_TRANSCEIVE, tag_type)

        if status != self.OK | back_bits != 0x10:
            status = self.ERROR
        return (status, back_bits)

    def anticollision(self):
        back_data = []
        ser_num_check = 0
        ser_num = []

        self.write(self.BitFramingReg, 0x00)

        ser_num.append(self.PICC_ANTICOLL)
        ser_num.append(0x20)

        (status, back_data, back_bits) = self.to_card(self.PCD_TRANSCEIVE, ser_num)

        if status == self.OK:
            i = 0
            if len(back_data) == 5:
                while i < 4:
                    ser_num_check = ser_num_check ^ back_data[i]
                    i += 1
                if ser_num_check != back_data[i]:
                    status = self.ERROR
            else:
                status = self.ERROR
        return (status, back_data)

    def calculate_crc(self, in_data):
        self.clear_bit_mask(self.DivIrqReg, 0x04)
        self.set_bit_mask(self.FIFOLevelReg, 0x80)

        i = 0
        while i < len(in_data):
            self.write(self.FIFODataReg, in_data[i])
            i += 1
        self.write(self.CommandReg, self.PCD_CALCCRC)

        i = 0xFF
        while True:
            n = self.read(self.DivIrqReg)
            i -= 1
            if not ((i != 0) and not (n & 0x04)):
                break
        out_data = [self.read(self.CRCResultRegL), self.read(self.CRCResultRegM)]
        return out_data

    def select_tag(self, ser_num):
        back_data = []
        buf = [self.PICC_SELECTTAG, 0x70]

        i = 0
        while i < 5:
            buf.append(ser_num[i])
            i += 1
        out = self.calculate_crc(buf)
        buf.append(out[0])
        buf.append(out[1])

        (status, back_data, back_len) = self.to_card(self.PCD_TRANSCEIVE, buf)

        if status == self.OK and back_len == 0x18:
            #print("Size: " + str(back_data[0]))
            return back_data[0]
        else:
            return 0

    def read_data(self, block_address):
        received_data = [self.PICC_READ, block_address]
        out = self.calculate_crc(received_data)
        received_data.append(out[0])
        received_data.append(out[1])
        (status, back_data, back_len) = self.to_card(self.PCD_TRANSCEIVE, received_data)
        if not status == self.OK:
            print("Error while reading!")

        if len(back_data) == 16:
            return back_data

    def auth(self, auth_mode, block_address, sector_key, ser_num):
        buff = [auth_mode, block_address]

        i = 0
        while i < len(sector_key):
            buff.append(sector_key[i])
            i += 1

        i = 0
        while i < 4:
            buff.append(ser_num[i])
            i += 1

        (status, back_data, back_len) = self.to_card(self.PCD_AUTHENT, buff)

        if not status == self.OK:
            print("Authentication Error!")
        if not (self.read(self.Status2Reg) and 0x08) != 0:
            print("Authentication Error! (status2reg and 0x08) != 0")

        return status

    def write_data(self, block_address, write_data):
        buff = [self.PICC_WRITE, block_address]
        crc = self.calculate_crc(buff)
        buff.append(crc[0])
        buff.append(crc[1])

        (status, back_data, back_len) = self.to_card(self.PCD_TRANSCEIVE, buff)
        if not status == self.OK or not back_len == 4 or not (back_data[0] & 0x0F) == 0x0A:
            status = self.ERROR

        print(str(back_len) + "back data &0x0F == 0x0A" + str(back_data[0] & 0x0F))

        if status == self.OK:
            i = 0
            temp_buf = []
            while i < 16:
                temp_buf.append(write_data[i])
                i += 1
            crc = self.calculate_crc(temp_buf)
            temp_buf.append(crc[0])
            temp_buf.append(crc[1])
            (status, back_data, back_len) = self.to_card(self.PCD_TRANSCEIVE, temp_buf)
            if not status == self.OK or not back_len == 4 or not (back_data[0] & 0x0F) == 0x0A:
                print("Error while writing")
            if status == self.OK:
                print("Data written")

    def to_card(self, command, send_data):
        back_data = []
        back_len = 0
        status = self.ERROR
        irq_en = 0x00
        wait_irq = 0x00
        last_bits = None

        if command == self.PCD_AUTHENT:
            irq_en = 0x12
            wait_irq = 0x10
        if command == self.PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_irq = 0x30

        self.write(self.CommIEnReg, irq_en | 0x80)
        self.clear_bit_mask(self.CommIrqReg, 0x80)
        self.set_bit_mask(self.FIFOLevelReg, 0x80)
        self.write(self.CommandReg, self.PCD_IDLE)

        i = 0
        while i < len(send_data):
            self.write(self.FIFODataReg, send_data[i])
            i += 1

        self.write(self.CommandReg, command)

        if command == self.PCD_TRANSCEIVE:
            self.set_bit_mask(self.BitFramingReg, 0x80)

        i = 2000
        n = 0
        while True:
            n = self.read(self.CommIrqReg)
            i -= 1
            if ~((i != 0) and ~(n & 0x01) and ~(n & wait_irq)):
                break

        self.clear_bit_mask(self.BitFramingReg, 0x80)

        if i != 0:
            if (self.read(self.ErrorReg) & 0x1B) == 0x00:
                status = self.OK

                if n & irq_en & 0x01:
                    status = self.NOTAGERR

                if command == self.PCD_TRANSCEIVE:
                    n = self.read(self.FIFOLevelReg)
                    last_bits = self.read(self.ControlReg) & 0x07

                    if last_bits != 0:
                        back_len = (n - 1) * 8 + last_bits
                    else:
                        back_len = n * 8

                    if n == 0:
                        n = 1
                    if n > self.MAX_LEN:
                        n = self.MAX_LEN

                    i = 0
                    while i < n:
                        back_data.append(self.read(self.FIFODataReg))
                        i += 1
            else:
                status = self.ERROR

        return (status, back_data, back_len)
