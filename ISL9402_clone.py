#!/usr/bin/python
import os
from ssl import VerifyFlags
import subprocess
import time
import re
import platform
import os.path
import argparse
from math import log,exp
import hashlib

TEMPO = 2.45 # 2.45
RS = 22000
RP = 10000
RT = 10000
CGREEN  = '\33[32m'
CRESET = '\33[0m'

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r','--read', dest='saveFile', action='store_true',
                        help='save EEPROM to file')
    parser.add_argument('-w','--write', dest='saveEEPROM', action='store_true',
                        help='save file to EEPROM')
    parser.add_argument('-u', dest='Cell_V', action='store_true',
                        help='print Cells Voltage')
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                        help='verbose')                    
    parser.add_argument('-t','--temp', dest='Temp', action='store_true',
                        help='print Temperature')
    parser.add_argument('-s','--status', dest='Status', action='store_true',
                        help='print BMS Status')                                                                
    parser.add_argument('-i','--info', dest='BMSInfo', action='store_true',
                        help='print BSM info')
    parser.add_argument('-md5', dest='MD5', action='store_true',
                        help='read RAM and EEPROM checksum - MD5')                                                              
    parser.add_argument('-f','--filename', type=str, default='eeprom.txt', dest='filename',
                        help='file name for storage EEPROM --f eeprom.txt')
    parser.add_argument('-sh','--shunt', type=float, default=0.005, dest='shunt',
                        help='shunt resistor in BMS') 
    parser.add_argument('-set', type=str, dest='setReg', nargs=2,
                        help='set BMS register "DOT -5" or -set HELP for list of registers ')                                        
    return parser.parse_args()

def Get12bitVoltage(RegL,RegH):
    Reg = ((RegH<<8) & 0x0fff) | (RegL & 0xff)
    return (Reg*1.8*8)/(4095*3)

def GetVBATT(RegL,RegH):
    Reg = ((RegH<<8) & 0x0fff) | (RegL & 0xff)
    return (Reg*1.8*32)/4095

def GetVRGO(RegL,RegH):
    Reg = ((RegH<<8) & 0x0fff) | (RegL & 0xff)
    return (Reg*1.8*2)/4095

def GetDxCx(RegL,RegH):
    Reg = ((RegH<<8) & 0x0fff) | (RegL & 0xff)
    return (Reg*1.8)/4095

def ITemp(x):
    return((x*1000/1.8527)-273.15)

def temperature(res, R0=10000.0, T0=25.0, beta=3435.0):
    out = log(res / R0) / beta + 1 / (273.15 + T0)
    out = 1 / out - 273.15
    return out

def resistance(tem, R0=10000.0, T0=25.0, beta=3435.0):
    out =  beta* (1 / (tem + 273.15) - 1 / (T0 + 273.15))
    return R0 * exp(out)  

def ETempLog(gain,u):
    u = u/gain
    r = RS * u / (TEMPO - u)
    r = abs(1/((1/r)-(1/RP)))
    t = temperature(r)
    return t

def Get12bitITEMP(RegL,RegH):
    Reg = ((RegH<<8) & 0x0fff) | (RegL & 0xff)
    return ITemp((Reg*1.8)/4095)

def Get12bitETEMP(RegL,RegH):
    Reg = ((RegH<<8) & 0x0fff) | (RegL & 0xff)
    Vtemp = ((Reg*1.8)/4095)
    return ETempLog(2,Vtemp)

def convert(x, n_bytes=2, order='big'):
    msb, *lsb = x.to_bytes(n_bytes, byteorder=order)
    return (msb, *lsb)

def Set12bitReg(x):    
    return ((x*3*4095)/(1.8*8))

def Set12bitReg1(x):    
    return ((x*4095)/(1.8))

def Temp_to_ADC(x):
    RT = resistance(float(x))
    RR = 1/(1/RT+1/RP)
    vADC = (TEMPO * (RR/(RR+RS)))*2
    msb,lsb = convert(int(Set12bitReg1(vADC)))
    return msb,lsb

def Checksum_MD5(lst):
    md5_hash = hashlib.md5()
    lst = [(reg[1:2]) for reg in lst]
    lst = sum(lst, [])
    # print(lst)
    byte_array = bytearray(lst)
    md5_hash.update(byte_array)    
    return md5_hash.hexdigest()

__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))) + "/"

args = parse_args()
filename = args.filename
shunt = args.shunt

address = 0x28
where = platform.system()
if where == "Linux":
    p = subprocess.Popen(['i2cdetect', '-l'],stdout=subprocess.PIPE,)
    for i in range(0,25):
        line = str(p.stdout.readline())
        s = re.search("i2c-tiny-usb", line)
        if s:
            line = re.split(r'\W+', line)
            bus=int(line[2])
    import smbus
    ISL9402 = smbus.SMBus(bus)
elif where == "Windows":        
    from i2c_mp_usb import I2C_MP_USB as SMBus
    ISL9402 = SMBus()
    # ISL9402.set_baudrate(50)
else:
  print("Platform not supported")

regName = ('OV Threshold(L)', 'OV Threshold(H)', 'OV Recovery(L)', 'OV Recovery(H)', 'UV Threshold(L)', 'UV Threshold(H)', 'UV Recovery(L)', 'UV Recovery(H)', 'OV Lockout Threshold(L) ', 'OV Lockout Threshold(H)', 'UV Lockout Threshold(L)', 'UV Lockout Threshold(H)', 'EOC Threshold(L)', 'EOC Threshold(H)', 'Low Voltage Charge Level(L)', 'Low Voltage Charge Level(H)', 'OV Delay Time Out(L)', 'OV Delay Time Out(H)', 'UV Delay Time Out(L)', 'UV Delay Time Out(H)', 'OW Timing(L)', 'OW Timing(H)', 'DOC Time Out', 'DOC Thresh', 'COC Time Out', 'COC Thresh', 'DSC Time Out', 'DSC Thresh', 'CB Min Voltage(L)', 'CB Min Voltage(H)', 'CB Max Voltage(L)', 'CB Max Voltage(H)', 'CB Min dV(L)', 'CB Mni dV(H)', 'CB Max dV(L)', 'CB Max dV(H)', 'CB On Time(L)', 'CB On Time(H)', 'CB Off Time(L)', 'CB Off Time(H)', 'CBUTS(L)', 'CBUTS(H)', 'CBUTR(L)', 'CBUTR(H)', 'CBOTS(L)', 'CBOTS(H)', 'CBOTR(L)', 'CBOTR(H)', 'COT Voltage(L)', 'COT Voltage (H)', 'COT Recovery(L)', 'COT Recovery(H)', 'CUT Voltage(L)', 'CUT Voltage(H)', 'CUT Recovery(L)', 'CUT Recovery(H)', 'DOT Voltage(L)', 'DOT Voltage(H)', 'DOT Recov(L)', 'DOT Recov(H)', 'DUT Voltage(L)', 'DUT Voltage(H)', 'DUT Recov(L)', 'DUT Recov(H)', 'IOT Voltage (L)', 'IOT Voltage(H)', 'IOT Recovery(L)', 'IOT Recovery(H)', 'Sleep Level V(L)', 'Sleep Level V(H)', 'Sleep Delay(L)', 'Sleep Delay(H)', 'Mode Timer', 'Cell Config', 'Settings', 'CB Control')
regOperationsName = ('Fault Status 1', 'Fault Status 2', 'Fault Statu 3', 'Fault Status 4', 'FET Control Bits', 'Analog MUX', 'Additional Control', 'Micro Control', 'Mode Control', 'EEPROM Enable')
regDataName = ('Cell Minimum Voltage(L)', 'Cell Minimum Voltage(H)', 'Cell Maximum Voltage(L)', 'Cell Maximum Voltage(H)', 'Pack Current(L)', 'Pack Current(H)', 'Cell 1 Voltage(L)', 'Cell 1 Voltage(H)', 'Cell 2 Voltage(L)', 'Cell 2 Voltage(H)', 'Cell 3 Voltage(L)', 'Cell 3 Voltage(H)', 'Cell 4 Voltage(L)', 'Cell 4 Voltage(H)', 'Cell 5 Voltage(L)', 'Cell 5 Voltage(H)', 'Cell 6 Voltage(L)', 'Cell 6 Voltage(H)', 'Cell 7 Voltage(L)', 'Cell 7 Voltage(H)', 'Cell 8 Voltage(L)', 'Cell 8 Voltage(H)', 'Internal Temperature(L)', 'Internal Temperature(H)', 'External Temperature1(L)', 'External Temperature1(H)', 'External Temeperature2(L)', 'External Temeperature2(H)', 'VBATT Voltage(L)', 'VBATT Voltage(H)', 'VRGO Voltage(L)', 'VRGO Voltage(H)', '14-Bit ADC(L)', '14-Bit ADC(H)')
regAddress = (0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B)
regOperationsAddress = (0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89)
regDataAddress = (0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB)

reg_dict = dict(zip(regName,regAddress))
reg_dict_Operations = dict(zip(regOperationsName,regOperationsAddress))
reg_dict_Data = dict(zip(regDataName,regDataAddress))

DOC = (4,8,16,24,32,48,64,96)
COC = (1,2,4,6,8,12,16,24)
DSC = (16,24,32,48,64,96,128,256)

Cells = {'3 Cells':0b10000011,
         '4 Cells':0b11000011,
         '5 Cells':0b11000111,
         '6 Cells':0b11100111,
         '7 Cells':0b11101111,
         '8 Cells':0b11111111}

Status = {'IN_SLEEP' :0b01000000,
          'IN_DOZE'  :0b00100000,
          'IN_IDLE'  :0b00010000}

CellBalance = {'CBUV':0b00001000,
               'CBOV':0b00000100}

TempFault = {'CBUTF' :0b00000010,
             'CBOTF' :0b00000001}

def write_EEPROM(xxxxx):
    ISL9402.write_byte_data(address, reg_dict_Operations['Mode Control'][0], 1)
    if "IN_IDLE" == list(Status.keys())[list(Status.values()).index(ISL9402.read_byte_data(address, reg_dict_Operations['Fault Status 4'][0]) & 0x70)] :
        ISL9402.write_byte_data(address, reg_dict_Operations['Micro Control'][0], 4)
        if 4 != ISL9402.read_byte_data(address, reg_dict_Operations['Micro Control'][0]):
            print("Device not ready for EEPROM communication. Micro Control.")

        ISL9402.write_byte_data(address, reg_dict_Operations["EEPROM Enable"][0], 1)
        if 1 != ISL9402.read_byte_data(address, reg_dict_Operations["EEPROM Enable"][0]):
            print("Device not ready for EEPROM communication. EEPROM Enable.")

        for reg in xxxxx:
            ISL9402.write_byte_data(address,reg[0],reg[1])
            time.sleep(0.03)
        if args.verbose:
            print("write EEPROM - OK")
        ISL9402.write_byte_data(address, reg_dict_Operations["EEPROM Enable"][0], 0)
        ISL9402.write_byte_data(address, reg_dict_Operations['Micro Control'][0], 0)
    else:
        print("BMS is not in IDLE mode")

    ISL9402.write_byte_data(address, reg_dict_Operations['Mode Control'][0], 0)
    return

def read_EEPROM(xxxxx):
    read_list = []
    ISL9402.write_byte_data(address, reg_dict_Operations['Mode Control'][0], 1)
    if "IN_IDLE" == list(Status.keys())[list(Status.values()).index(ISL9402.read_byte_data(address, reg_dict_Operations['Fault Status 4'][0]) & 0x70)] :
        ISL9402.write_byte_data(address, reg_dict_Operations['Micro Control'][0], 4)
        if 4 != ISL9402.read_byte_data(address, reg_dict_Operations['Micro Control'][0]):
            print("Device not ready for EEPROM communication. Micro Control.")

        ISL9402.write_byte_data(address, reg_dict_Operations["EEPROM Enable"][0], 1)
        if 1 != ISL9402.read_byte_data(address, reg_dict_Operations["EEPROM Enable"][0]):
            print("Device not ready for EEPROM communication. EEPROM Enable.")

        for reg in xxxxx:
            if reg[0] % 2 == 0:
                ISL9402.read_byte_data(address,reg[0])
            read_list.append([reg[0],ISL9402.read_byte_data(address,reg[0])])
        if args.verbose:
            print("read EEPROM - OK")
        ISL9402.write_byte_data(address, reg_dict_Operations["EEPROM Enable"][0], 0)
        ISL9402.write_byte_data(address, reg_dict_Operations['Micro Control'][0], 0)
    else:
        print("BMS is not in IDLE mode")

    ISL9402.write_byte_data(address, reg_dict_Operations['Mode Control'][0], 0)
    return read_list

def write_RAM(xxxxx):
    for reg in xxxxx:
        ISL9402.write_byte_data(address,reg[0],reg[1])
    return

# init values
for key in reg_dict:
    reg_dict[key] = list((reg_dict[key],1))

for key in reg_dict_Data:
    reg_dict_Data[key] = list((reg_dict_Data[key],1))

for key in reg_dict_Operations:
    reg_dict_Operations[key] = list((reg_dict_Operations[key],1))

if args.saveFile:
    for key in reg_dict:
        reg_dict[key][1] = ISL9402.read_byte_data(address, reg_dict[key][0])

    fout = open(__location__ + filename, 'w')

    for _,value in reg_dict.values():
        fout.writelines(str.format('{:02X}', value)+"\n")
    fout.close()
    if args.verbose:          
        print("MD5 = ",Checksum_MD5(reg_dict.values()))

if args.saveEEPROM:
    if not os.path.isfile(filename):
        print('File does not exist.')
    else:
        with open(filename) as f:
            lines_n = f.read().splitlines()
        k=0
        for key in reg_dict:
            reg_dict[key][1] = int(lines_n[k],16)
            k=k+1

        lst = [reg for reg in reg_dict.values()]
        md5 = Checksum_MD5(reg_dict.values())
        write_EEPROM(lst)
        verify = read_EEPROM(lst)
        md5v = Checksum_MD5(verify)
     
    if args.verbose:
        if md5 == md5v:
            print("verify - OK")
        else:
            print("verify - BAD")
        print("MD5 write = ",md5)
        print("MD5 read = ",md5v)


if args.BMSInfo or args.Cell_V or args.Temp or args.Status or args.MD5:
    for key in reg_dict:
        reg_dict[key][1] = ISL9402.read_byte_data(address, reg_dict[key][0])

    for key in reg_dict_Data:
        reg_dict_Data[key][1] = ISL9402.read_byte_data(address, reg_dict_Data[key][0])

    for key in reg_dict_Operations:
        reg_dict_Operations[key][1] = ISL9402.read_byte_data(address, reg_dict_Operations[key][0])    

if args.MD5:
    md5 = Checksum_MD5(reg_dict.values())
    lst = [reg for reg in reg_dict.values()]
    verify = read_EEPROM(lst)
    md5v = Checksum_MD5(verify)
    print("RAM MD5 = ",md5)
    print("EEPROM MD5 = ",md5v)
    if md5 == md5v:
        print("RAM and EEPROM are the " + CGREEN + "same." + CRESET)

if args.BMSInfo:
    print("OV Threshold = ",Get12bitVoltage(reg_dict["OV Threshold(L)"][1],reg_dict["OV Threshold(H)"][1]))
    print("OV Recovery = ",Get12bitVoltage(reg_dict["OV Recovery(L)"][1],reg_dict["OV Recovery(H)"][1]))
    print("UV Threshold = ",Get12bitVoltage(reg_dict["UV Threshold(L)"][1],reg_dict["UV Threshold(H)"][1]))
    print("Discharge Overcurrent = ", DOC[(reg_dict['DOC Thresh'][1] & 0x70)>>4]*0.001/shunt)    
    print("Charge Overcurrent = ", COC[(reg_dict['COC Thresh'][1] & 0x70)>>4]*0.001/shunt)
    print("Discharge Short-Circuit = ", DSC[(reg_dict['DSC Thresh'][1] & 0x70)>>4]*0.001/shunt)
    print("Cell Config = ",list(Cells.keys())[list(Cells.values()).index(reg_dict['Cell Config'][1])])
    # print('{0:b}'.format(reg_dict_Operations['Fault Status 4'][1]))
    try:
        print("BMS Status = ",list(Status.keys())[list(Status.values()).index(reg_dict_Operations['Fault Status 4'][1] & 0x70)])
    except ValueError:    
        print("BMS Status = unknown, try again in 20 seconds or press the button WakeUp ..or..or..")    

if args.Cell_V:
    # print("Cell Minimum Voltage = ",Get12bitVoltage(reg_dict_Data['Cell Minimum Voltage(L)'][1],reg_dict_Data['Cell Minimum Voltage(H)'][1]))
    # print("Cell Maximum Voltage = ",Get12bitVoltage(reg_dict_Data['Cell Maximum Voltage(L)'][1],reg_dict_Data['Cell Maximum Voltage(H)'][1]))
    print("Cell 1 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 1 Voltage(L)'][1],reg_dict_Data['Cell 1 Voltage(H)'][1]))
    print("Cell 2 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 2 Voltage(L)'][1],reg_dict_Data['Cell 2 Voltage(H)'][1]))
    print("Cell 3 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 3 Voltage(L)'][1],reg_dict_Data['Cell 3 Voltage(H)'][1]))
    print("Cell 4 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 4 Voltage(L)'][1],reg_dict_Data['Cell 4 Voltage(H)'][1]))
    print("Cell 5 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 5 Voltage(L)'][1],reg_dict_Data['Cell 5 Voltage(H)'][1]))
    print("Cell 6 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 6 Voltage(L)'][1],reg_dict_Data['Cell 6 Voltage(H)'][1]))
    print("Cell 7 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 7 Voltage(L)'][1],reg_dict_Data['Cell 7 Voltage(H)'][1]))
    print("Cell 8 Voltage = ",Get12bitVoltage(reg_dict_Data['Cell 8 Voltage(L)'][1],reg_dict_Data['Cell 8 Voltage(H)'][1]))
    print("VBATT = ",GetVBATT(reg_dict_Data['VBATT Voltage(L)'][1],reg_dict_Data['VBATT Voltage(H)'][1]))
    print("VRGO = ",GetVRGO(reg_dict_Data['VRGO Voltage(L)'][1],reg_dict_Data['VRGO Voltage(H)'][1]))

if args.Temp:    
    print("Internal Temperature = ",Get12bitITEMP(reg_dict_Data['Internal Temperature(L)'][1],reg_dict_Data['Internal Temperature(H)'][1]))
    print("External Temperature1 = ",Get12bitETEMP(reg_dict_Data['External Temperature1(L)'][1],reg_dict_Data['External Temperature1(H)'][1]))
    print("External Temperature2 = ",Get12bitETEMP(reg_dict_Data['External Temeperature2(L)'][1],reg_dict_Data['External Temeperature2(H)'][1]))
    print("Discharge Over-Temperature = ",ETempLog(2,GetDxCx(reg_dict['DOT Voltage(L)'][1],reg_dict['DOT Voltage(H)'][1])))
    print("Discharge Over-Temperature Recovery = ",ETempLog(2,GetDxCx(reg_dict['DOT Recov(L)'][1],reg_dict['DOT Recov(H)'][1])))
    print("Discharge Under-Temperature = ",ETempLog(2,GetDxCx(reg_dict['DUT Voltage(L)'][1],reg_dict['DUT Voltage(H)'][1])))
    print("Discharge Under-Temperature Recovery = ",ETempLog(2,GetDxCx(reg_dict['DUT Recov(L)'][1],reg_dict['DUT Recov(H)'][1])))

def DUT():
    msb,lsb = Temp_to_ADC(args.setReg[1])
    reg_dict['DUT Voltage(L)'][1] = lsb
    reg_dict['DUT Voltage(H)'][1] = msb
    lst = [reg_dict['DUT Voltage(L)'],reg_dict['DUT Voltage(H)']]
    write_EEPROM(lst)
    write_RAM(lst)
    return msb,lsb

def DUTR():
    msb,lsb = Temp_to_ADC(args.setReg[1])
    reg_dict['DUT Recov(L)'][1] = lsb
    reg_dict['DUT Recov(H)'][1] = msb
    lst = [reg_dict['DUT Recov(L)'],reg_dict['DUT Recov(H)']]
    write_EEPROM(lst)
    write_RAM(lst)
    return msb,lsb

def DOT():
    msb,lsb = Temp_to_ADC(args.setReg[1])
    reg_dict['DOT Voltage(L)'][1] = lsb
    reg_dict['DOT Voltage(H)'][1] = msb
    lst = [reg_dict['DOT Voltage(L)'],reg_dict['DOT Voltage(H)']]
    write_EEPROM(lst)
    write_RAM(lst)
    return msb,lsb

def DOTR():
    msb,lsb = Temp_to_ADC(args.setReg[1])
    reg_dict['DOT Recov(L)'][1] = lsb
    reg_dict['DOT Recov(H)'][1] = msb
    lst = [reg_dict['DOT Recov(L)'],reg_dict['DOT Recov(H)']]
    write_EEPROM(lst)
    write_RAM(lst)
    return msb,lsb

def HELP():
    return print(', '.join(str(key) for key in switcher.keys()))

switcher = {
    "DUT" : DUT,
    "DUTR": DUTR,
    "DOT" : DOT,
    "DOTR": DOTR,
    "HELP": HELP
    }

def sw_to_func(argument):
    func = switcher.get(argument, "nothing")
    return func()

if args.setReg:
    setReg = args.setReg
    sw_to_func(setReg[0])    

if args.Status:
    # print(reg_dict_Operations)
    # print(reg_dict_Data)
    print(reg_dict)
