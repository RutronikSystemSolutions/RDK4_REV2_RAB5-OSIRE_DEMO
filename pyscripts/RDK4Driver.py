# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 09:12:40 2024

@author: E1001997, GDR
"""

import struct
import time

import serial
import serial.tools.list_ports


class RDK4Driver:
    def __init__(self, timeout=1):
        self.ser = serial.Serial()
        pass

    def connect(self):
        self.port = self.AutoPortFinder()
        self.baud = 256000
        self.ser = serial.Serial(self.port, self.baud, serial.EIGHTBITS,
                                 serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=5)

    def connecctionstate(self):
        if self.ser.isOpen():
            print(self.ser.name, 'is open...')

        else:
            print('Not Opened')

    def close(self):
        self.ser.close()
        # print(self.ser.name, 'is closed.')

    def SendCommand(self, commandtowrite, len):
        self.ser.write(commandtowrite[:len])

    def ReceiveCommand(self, length):
        out = self.ser.read(length)
        return out

    def AutoPortFinder(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # print(p)

            if "Cypress" in p.manufacturer:
                return p.device

    def TimeoutCommand(self, seconds):
        self.ser.timeout = seconds


class RDK4Command:
    def __init__(self):
        self.drv = RDK4Driver()
        self.bytenumber = 265
        self.StructArray = {"DeviceID": [0, "<", "B"],
                            "Group": [1, "<", "B"],
                            "Length": [2, "<", "B"],
                            "Byte_3": [3, "<", "B"],
                            "Param1": [4, "<", "B"],
                            "Param2": [5, "<", "B"],
                            "Param3": [6, "<", "B"],
                            "Param4": [7, "<", "B"],
                            "Param5": [8, "<", "B"],
                            "Param6": [9, "<", "B"],
                            "Param7": [10, "<", "B"],
                            "Param8": [11, "<", "B"],
                            "Param9": [12, "<", "B"],
                            "Param10": [13, "<", "B"],
                            }
        self.Offset = 0
        self.Endian = 1
        self.Format = 2

    def initiliaze_structure(self):
        self.cmd = bytearray()
        for i in range(0, self.bytenumber):
            self.cmd.append(0)

    def StructPackInto(self, StructArrayCmd, CaseValue):
        # print(self.StructArray[StructArrayCmd][self.Endian])
        # print(self.StructArray[StructArrayCmd][self.Format])
        struct.pack_into(self.StructArray[StructArrayCmd][self.Endian] + self.StructArray[StructArrayCmd]
        [self.Format], self.cmd, self.StructArray[StructArrayCmd][self.Offset], CaseValue)

    def Send(self, len):
        # print(self.cmd)
        self.drv.SendCommand(self.cmd, len)

    def Receive(self, length, formattodecode="B", endian="<"):
        out = self.drv.ReceiveCommand(length)
        data = struct.unpack(endian + str(length) + formattodecode, bytearray(out))

        return data

    def ReceiveRaw(self, length):
        out = self.drv.ReceiveCommand(length)
        return out

    def Timeout(self, seconds):
        self.drv.TimeoutCommand(seconds)

    def crc8(self, datagram, initial_value):
        crc = initial_value
        # Iterate bytes in data
        for byte in datagram:
            crc = crc ^ byte
            # Iterate bits in byte
            for _ in range(0, 8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x2F) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                # Shift to next bit
                byte = byte >> 1
        return crc


class RDK4Functions(RDK4Command):
    def __init__(self):
        RDK4Command.__init__(self)
        self.FunctionCall = {"OSP_RESET": 0,
                             "OSP_INIT_BIDIR": 2,
                             "OSP_INIT_LOOP": 3,
                             "OSP_GO_SLEEP": 4,
                             "OSP_GO_ACTIVE": 5,
                             "OSP_GO_DEEP_SLEEP": 6,
                             "OSP_SET_PWM": 7,
                             "OSP_IDENTIFY": 10,
                             "OSP_READTEMP": 11,
                             "OSP_READADC": 12,
                             "OSP_SETADC": 13,
                             "OSP_READCURRCH": 14,
                             "OSP_SETCURRCH": 15,
                             "OSP_READSTATUS": 16,
                             "OSP_CLRERROR": 17,
                             "OSP_READADCDATA": 18,
                             "OSP_READTEMPSTATUS": 19,
                             "OSP_SET_PWM_OS": 32,

                             }

        self.GroupCall = {"LADDER_UP": 0,
                          "LADDER_DWN": 1,
                          "PAUSE": 2,

                          }

    def connect(self):
        self.drv.connect()

    def connectionstatus(self):
        self.drv.ser.isOpen()

    def close(self):
        self.drv.close()

    def set_uart_mode(self):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 0x01)
        self.StructPackInto("Group", 0x00)
        self.StructPackInto("Length", 0x01)
        self.StructPackInto("Byte_3", 0x00)

        crc = 0
        crc = self.crc8(self.cmd[:4], crc)
        self.StructPackInto("Param1", crc)

        self.Send(5)
        time.sleep(0.1)
        out = self.Receive(length)
        # print(out)
        return out[4]

    def osp_reset(self):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_RESET"])
        self.StructPackInto("Param1", 0x00)
        self.StructPackInto("Param2", 0x00)

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(1)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_clearerror function..")

        return out[4]

    def osp_clearerror(self, addr=0):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_CLRERROR"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.1)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_clearerror function..")

        return out[4]

    def osp_init_bidir(self, addr):
        length = 15
        state = {0: "UNINITIALIZED", 1: "SLEEP", 2: "ACTIVE", 3: "DEEP SLEEP"}
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_INIT_BIDIR"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))
        # self.StructPackInto("Param3", 0xE2)

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.1)
        out = self.Receive(length)
        # print(out)

        # if out[4] == 255:
        #     print("NACK received, something went wrong in function..") 
        #     return out[4]

        tempdeg = out[12] - 86

        data = {
            "LAST": out[10] << 8 | out[11],
            "TEMP (DegC)": tempdeg,
            "UV_FLAG": out[13] & 0x1,
            "OT_FLAG": (out[13] & 0x2) >> 1,
            "LOS_FLAG": (out[13] & 0x4) >> 2,
            "CE_FLAG": (out[13] & 0x8) >> 3,
            "OV_FLAG": (out[13] & 0x10) >> 4,
            "TEST_FLAG": (out[13] & 0x20) >> 5,
            "STATE": state[(out[13] & 0xC0) >> 6]

        }

        return data

    def osp_go_active(self, addr=0):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_GO_ACTIVE"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_go_active function..")

        return out[4]

    def osp_go_deep_sleep(self, addr=0):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_GO_DEEP_SLEEP"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        if out[4] == 255:
            print("NACK received, something went wrong in osp_go_deep_sleep function..")

        return out[4]

    def osp_go_sleep(self, addr=0):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_GO_SLEEP"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_go_sleep function..")

        return out[4]

    def osp_setpwm(self, addr, led, red, green, blue):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 10)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_SET_PWM"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))
        self.StructPackInto("Param3", led)
        self.StructPackInto("Param4", red >> 8)
        self.StructPackInto("Param5", int(red & 0xFF))
        self.StructPackInto("Param6", green >> 8)
        self.StructPackInto("Param7", int(green & 0xFF))
        self.StructPackInto("Param8", blue >> 8)
        self.StructPackInto("Param9", int(blue & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:13], crc)
        self.StructPackInto("Param10", crc)

        self.Send(14)
        time.sleep(0.7)
        # out = self.Receive(length)

        # if out[4] == 255:
        #      #print("NACK received, something went wrong in osp_setpwm function..")

        # return out[4] 

    def osp_setpwm_os(self, addr, red, green, blue, daymode):
        length = 6
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 10)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_SET_PWM_OS"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))
        self.StructPackInto("Param3", red >> 8)
        self.StructPackInto("Param4", int(red & 0xFF))
        self.StructPackInto("Param5", green >> 8)
        self.StructPackInto("Param6", int(green & 0xFF))
        self.StructPackInto("Param7", blue >> 8)
        self.StructPackInto("Param8", int(blue & 0xFF))
        self.StructPackInto("Param9", daymode)

        crc = 0
        crc = self.crc8(self.cmd[:13], crc)
        self.StructPackInto("Param10", crc)

        self.Send(14)
        time.sleep(0.7)

    def osp_identify(self, addr):
        length = 12
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_IDENTIFY"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_identify function..") 

        return out[10]

    def osp_readstatus(self, addr):
        length = 12
        state = {0: "UNINITIALIZED", 1: "SLEEP", 2: "ACTIVE", 3: "DEEP SLEEP"}
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_READSTATUS"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_readstatus function..") 
        return out[4]

        data = {
            "UV_FLAG": out[10] & 0x1,
            "OT_FLAG": (out[10] & 0x2) >> 1,
            "LOS_FLAG": (out[10] & 0x4) >> 2,
            "CE_FLAG": (out[10] & 0x8) >> 3,
            "OV_FLAG": (out[10] & 0x10) >> 4,
            "TEST_FLAG": (out[10] & 0x20) >> 5,
            "STATE": state[(out[10] & 0xC0) >> 6]
        }

        return data

    def osp_readtemp(self, addr):
        length = 12

        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_READTEMP"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_readtemp function..") 

        tempdeg = out[10] - 86
        return [out[10], tempdeg]

    def osp_readtempstat(self, addr):
        length = 13
        state = {0: "UNINITIALIZED", 1: "SLEEP", 2: "ACTIVE", 3: "DEEP SLEEP"}
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_READTEMPSTATUS"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_readstatus function..") 
        return out[4]

        tempdeg = out[10] - 86

        data = {
            "TEMP (DegC)": tempdeg,
            "UV_FLAG": out[11] & 0x1,
            "OT_FLAG": (out[11] & 0x2) >> 1,
            "LOS_FLAG": (out[11] & 0x4) >> 2,
            "CE_FLAG": (out[11] & 0x8) >> 3,
            "OV_FLAG": (out[11] & 0x10) >> 4,
            "TEST_FLAG": (out[11] & 0x20) >> 5,
            "STATE": state[(out[11] & 0xC0) >> 6]

        }

        return data

    def osp_readadc(self, addr):
        length = 12
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_READADC"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_readtemp function..") 

        return out[10]

    def osp_readadcdata(self, addr):
        length = 13
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 3)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_READADCDATA"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))

        crc = 0
        crc = self.crc8(self.cmd[:6], crc)
        self.StructPackInto("Param3", crc)

        self.Send(7)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        if out[4] == 255:
            print("NACK received, something went wrong in osp_readtemp function..")

        return (out[10] << 8) + out[11]

    def osp_setadc(self, addr, fusa_en, dis_sync, mux_ctrl):
        length = 6

        # if fusa_en > 1 or dis_sync > 1 or mux_ctrl > 15:
        #     print("Something went wrong in osp_setadc function: fusa_en and dis_sync are 2 bits, mux_ctrl is 4 bits")
        return 0xFF

        flag = (fusa_en << 5) + (dis_sync << 4) + mux_ctrl
        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 4)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_SETADC"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))
        self.StructPackInto("Param3", flag)

        crc = 0
        crc = self.crc8(self.cmd[:7], crc)
        self.StructPackInto("Param4", crc)

        self.Send(8)
        time.sleep(0.7)
        out = self.Receive(length)
        # print(out)
        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_setpwm function..")

        return out[4]

    def osp_read_current_channel(self, addr, led):
        length = 15

        curr_coding_led0 = {0: "3mA", 1: "6mA", 2: "12mA", 3: "24mA", 4: "48mA", 8: "50uA", 9: "100uA", 10: "500uA",
                            11: "1000uA"}
        curr_coding_led12 = {0: "1.5mA", 1: "3mA", 2: "6mA", 3: "12mA", 4: "24mA", 8: "50uA", 9: "100uA", 10: "500uA",
                             11: "1000uA"}

        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 4)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_READCURRCH"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))
        self.StructPackInto("Param3", led)

        crc = 0
        crc = self.crc8(self.cmd[:7], crc)
        self.StructPackInto("Param4", crc)

        self.Send(8)
        time.sleep(0.7)
        out = self.Receive(length)
        print(out)

        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_read_current_channel function..") 
        #  return out[4]

        if led == 0:
            rcurr = curr_coding_led0[out[11]]
            gcurr = curr_coding_led0[out[12]]
            bcurr = curr_coding_led0[out[13]]
        else:
            rcurr = curr_coding_led12[out[11]]
            gcurr = curr_coding_led12[out[12]]
            bcurr = curr_coding_led12[out[13]]

        data = {
            "SYNC_EN": (out[10] & 0x4) >> 2,
            "HYBRID_PWM": (out[10] & 0x2) >> 1,
            "DITHERING_EN": out[10] & 0x1,
            "PWMRED": rcurr,
            "PWMGREEN": gcurr,
            "PWMBLUE": bcurr
        }

        return data

    def osp_set_current_channel(self, addr, led, sync, hyb, dith, rcurr, gcurr, bcurr):
        length = 6
        flag = dith + (2 * hyb) + ((2 ** 2) * sync)
        curr_coding_led0 = {"3mA": 0, "6mA": 1, "12mA": 2, "24mA": 3, "48mA": 4, "50uA": 8, "100uA": 9, "500uA": 10,
                            "1000uA": 11}
        curr_coding_led12 = {"1.5mA": 0, "3mA": 1, "6mA": 2, "12mA": 3, "24mA": 4, "50uA": 8, "100uA": 9, "500uA": 10,
                             "1000uA": 11}

        self.initiliaze_structure()
        self.StructPackInto("DeviceID", 1)
        self.StructPackInto("Group", 2)
        self.StructPackInto("Length", 8)
        self.StructPackInto("Byte_3", self.FunctionCall["OSP_SETCURRCH"])
        self.StructPackInto("Param1", addr >> 8)
        self.StructPackInto("Param2", int(addr & 0xFF))
        self.StructPackInto("Param3", led)
        self.StructPackInto("Param4", flag)
        if led == 0:
            self.StructPackInto("Param5", curr_coding_led0[rcurr])
            self.StructPackInto("Param6", curr_coding_led0[gcurr])
            self.StructPackInto("Param7", curr_coding_led0[bcurr])
        else:
            self.StructPackInto("Param5", curr_coding_led12[rcurr])
            self.StructPackInto("Param6", curr_coding_led12[gcurr])
            self.StructPackInto("Param7", curr_coding_led12[bcurr])

        crc = 0
        crc = self.crc8(self.cmd[:11], crc)
        self.StructPackInto("Param8", crc)

        self.Send(12)
        time.sleep(0.7)
        out = self.Receive(length)

        # if out[4] == 255:
        #     print("NACK received, something went wrong in osp_set_current_channel function..")

        return out[4]
