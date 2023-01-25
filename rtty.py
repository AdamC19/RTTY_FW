import serial
import struct
import time

class RTTY(object):
    MODE_IDLE = 0,
    MODE_RX = 1,
    MODE_TX = 2,
    MODE_CALIBRATE_VCO = 3

    FIELD_MODE = 0
    FIELD_FREQ_MHZ = 1
    FIELD_MARK_FREQ = 2
    FIELD_SPACE_FREQ = 3
    FIELD_BAUD_RATE = 4
    FIELD_TX_DATA = 5
    FIELD_RX_DATA_RDY = 6
    FIELD_RX_DATA = 7
    FIELD_RX_TONE = 8
    FIELD_VCO_DAC_VOLTAGE = 9
    FIELD_VCO_FREQ_CAL_VALUE = 10
    FIELD_PA_DAC_VOLTAGE = 11

    CMD_NONE = 0
    CMD_READ_FIELDS = 1
    CMD_SET_FIELDS = 2
    CMD_TEST_COMMS = 3

    def __init__(self, comport) -> None:
        self.ser = serial.Serial(comport)
        self.ser.baudrate = 115200
        self.ser.timeout = 0.5
    
    def read_fields(self, fields):
        arr_len = 2 + len(fields)
        arr = bytearray(arr_len)
        cmd = self.CMD_READ_FIELDS
        body_len = len(fields)
        struct.pack_into("BB", arr, 0, cmd, body_len)
        offset = 2
        for field in fields:
            struct.pack_into("B", arr, offset, field)
            offset += 1

        self.ser.write(arr)

        rpy_hddr = self.ser.read(2)
        if rpy_hddr[0] != self.CMD_READ_FIELDS:
            return None

        rpy_len = rpy_hddr[1]
        rpy_body = self.ser.read(rpy_len)
        
        retval = {}
        for chunk in struct.iter_unpack("<Bi", rpy_body):
            retval[chunk[0]] = chunk[1]

        return retval

    def set_fields(self, fields: dict):
        body_len = 5*len(fields.keys())
        cmd = self.CMD_SET_FIELDS
        hddr = bytearray(2)
        body = bytearray(body_len)
        hddr[0] = cmd
        hddr[1] = body_len

        offset = 0
        for key in fields.keys():
            if type(fields[key]) == int:
                struct.pack_into("<Bi", body, offset, key, fields[key])
            elif type(fields[key]) == float:
                struct.pack_into("<Bf", body, offset, key, fields[key])

            offset += 5
        
        self.ser.write(hddr)
        self.ser.write(body)

    def set_mode(self, mode):
        self.set_fields({self.FIELD_MODE: mode})

    def start_vco_cal_point(self, vco_dac_voltage):
        # set VCO DAC voltage
        # also set VCO calibration freq to 0.0
        self.set_fields({self.FIELD_VCO_DAC_VOLTAGE: vco_dac_voltage, self.FIELD_VCO_FREQ_CAL_VALUE: 0.0})

        # switch mode to VCO CAL mode
        self.set_mode(self.MODE_CALIBRATE_VCO)
    
    def finish_vco_cal_point(self, freq):
        self.set_fields({self.FIELD_VCO_FREQ_CAL_VALUE: freq})
    
    def enter_rx_mode(self):
        self.set_mode(self.MODE_RX)
    
    def poll_for_rx_data(self):
        rpy = self.read_fields([self.FIELD_RX_DATA_RDY])
        if rpy[self.FIELD_RX_DATA_RDY] == 1:
            rpy = self.read_fields([self.FIELD_RX_DATA])
            return rpy[self.FIELD_RX_DATA]
        else:
            return None

#------------------------------------------------------------------------------
def calibrate_vco(rtty: RTTY):
    dac_cal_points = 64
    dac_fs_voltage = 3.3
    step_size = dac_fs_voltage/(dac_cal_points - 1)
    for i in range(0, dac_cal_points):
        print("Setting VCO DAC voltage to {:.4f}...".format(i * step_size))
        rtty.start_vco_cal_point(i * step_size)
        freq = 1.0e6*float(input("Enter actual frequency in MHz: "))
        rtty.finish_vco_cal_point(freq)
        time.sleep(0.1)

#------------------------------------------------------------------------------
if __name__ == '__main__':
    comport = input("Comport: ")
    
    if 'COM' not in comport:
        comport = 'COM' + comport
    
    rtty = RTTY(comport)

    def display_help():
        print("=== Actions ===")
        print(" - h : help")
        print(" - c : calibrate")
        print(" - r : receive")
        print(" - t : transmit")
        print(" - q : quit")
        print(" - f : set frequency")
        print(" - m : set mark tone frequency")
        print(" - s : set space tone frequency")
        print("===============")

    display_help()

    quit = False
    while not quit:
        inp = input("Action: ")
        parts = inp.split(' ')
        act = parts[0]
        if act == 'h':
            display_help()
        elif act == 'q':
            print("Quitting...")
            quit = True
        elif act == 'c':
            print("Beginning VCO calibration...")
            calibrate_vco(rtty)
        elif act == 'f':
            try:
                freq = float(parts[1])
            except IndexError:
                print("No frequency specified.")
        elif act == 'm':
            try:
                tone = float(parts[1])
            except IndexError:
                print("No tone frequency specified.")
        elif act == 's':
            try:
                tone = float(parts[1])
            except IndexError:
                print("No tone frequency specified.")
        elif act == 'r':
            print("Entering receive mode (Ctrl+C to exit)...")
            rtty.enter_rx_mode()

            done = False
            while not done:
                try:
                    data = rtty.poll_for_rx_data()
                    if data != None:
                        print("0x{:x}".format(data), end=" ")
                except KeyboardInterrupt:
                    done = True
            print("Exiting receive mode...")
        elif act == 't':
            try:
                tx_msg = parts[1]
                print("Transmitting {}...".format(tx_msg))
            except IndexError:
                print("No message provided.")