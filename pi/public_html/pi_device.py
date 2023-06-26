DEVICE_ID = "4W15631IMYD9Q5MHKBX1"
URI = "wss://innomati.com:8443/"

# General Import
import json
import asyncio
import websockets
import threading
import datetime        
from time import sleep
import subprocess
import requests
import pathlib
import sys

######################################################################
# Check internat connection START
######################################################################
try:
    import httplib
except:
    import http.client as httplib
def have_internet():
    conn = httplib.HTTPConnection("www.google.com", timeout=5)
    try:
        conn.request("HEAD", "/")
        conn.close()
        return True
    except:
        conn.close()
        return False

def script_is_running(filename):
    cmd = ['pgrep -f .*python.*' + filename]
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    my_pid, err = process.communicate()
    return len(my_pid.splitlines()) > 2

# avoid running second file by ssh 
if script_is_running("pi_device.py"):
    sys.exit()
else:
    while not have_internet():
        sleep(5)
######################################################################
# Check internat connection END
######################################################################

#############################
# Sensors START
#############################

def get_ttyUSB_port(name):
    p = subprocess.Popen("dmesg | grep ttyUSB", stdout=subprocess.PIPE, shell=True)
    result_arr = p.communicate()[0].decode("utf-8").split("\n")
    ttyUSBS = []
    for i in range(len(result_arr)):
        detail = result_arr[i]
        if name in detail:
            detail_arr = detail.split(" ")
            ttyUSBS.append(detail_arr[len(detail_arr) - 1])
    return ttyUSBS

# GPS
# from gps3 import gps3
# gps_socket = gps3.GPSDSocket()
# data_stream = gps3.DataStream()
# gps_socket.connect()
# gps_socket.watch()
            # python3
            # count = 0
            # for new_data in gps_socket:
            #     if new_data:
            #         data_stream.unpack(new_data)
            #         alt = data_stream.TPV['alt']
            #         lat = data_stream.TPV['lat']
            #         lon = data_stream.TPV['lon']
            #         if count == 5:
            #             break
            #         elif alt == 'n/a' and lat == 'n/a' and lon == 'n/a':
            #             await asyncio.sleep(0.5)
            #             count += 1
            #         else:
            #             sensor_data["mode"] = str(data_stream.TPV['mode']) + "D" 
            #             sensor_data["lat"] = str(data_stream.TPV['lat'])
            #             sensor_data["lat_95_confidence_in_m"] = str(data_stream.TPV['epy'])
            #             sensor_data["lon"] = str(data_stream.TPV['lon'])
            #             sensor_data["lon_95_confidence_in_m"] = str(data_stream.TPV['epx'])
            #             sensor_data["alt"] = str(data_stream.TPV['alt'])
            #             sensor_data["alt_95_confidence_in_m"] = str(data_stream.TPV['epv'])
            #             sensor_data["H_speed"] = str(data_stream.TPV['speed'])
            #             sensor_data["H_speed_95_confidence_in_ms"] = str(data_stream.TPV['eps'])
            #             sensor_data["V_speed"] = str(data_stream.TPV['climb'])
            #             sensor_data["V_speed_95_confidence_in_ms"] = str(data_stream.TPV['epc'])
            #             break
import RPi.GPIO as GPIO
import serial

GPS_DATA = None
LAST_GPS_Data = None

def send_at(ser, command, back, timeout):
    rec_buff = ''
    ser.write((command+'\r\n').encode())
    sleep(timeout)
    if ser.inWaiting():
        sleep(0.01)
        rec_buff = ser.read(ser.inWaiting())
    if rec_buff != '':
        if back not in rec_buff.decode():
            return {"success": False, "error": {"message": str(rec_buff.decode())}}
        else:
            return {"success": True, "data": rec_buff.decode()}
    else:
        return {"success": False, "error": {"message": "Empty result"}}

def get_gps_position(ser):
    result = send_at(ser, 'AT+CGPSINFO','+CGPSINFO: ',1)
    if result["success"]: 
        if ',,,,,,' in result["data"]:
            return {"success": False, "error": {"message": "GPS is not ready"}} 
        data = result["data"].replace("\r", "").split('\n')
        gps_data = data[3].replace("+CGPSINFO: ", "").split(",")
        gps_data[0], gps_data[2] = convert_coordinate_from_dd_to_dms(gps_data[0], gps_data[2])
        return {"success": True, "data": gps_data}
    return result

def convert_coordinate_from_dd_to_dms(lat_str, lot_str):
    new_lat = round(float(lat_str[:2]) + float(lat_str[2:]) / 60, 6)
    new_lot = round(float(lot_str[:3]) + float(lot_str[3:]) / 60, 6)
    return (str(new_lat), str(new_lot))

def power_on(ser, power_key):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(power_key,GPIO.OUT)
    sleep(0.1)
    GPIO.output(power_key,GPIO.HIGH)
    sleep(2)
    GPIO.output(power_key,GPIO.LOW)
    sleep(20)
    ser.flushInput()

def run_gps():
    global GPS_DATA, LAST_GPS_Data

    ser = serial.Serial('/dev/' + get_ttyUSB_port("GSM modem")[2],115200)
    ser.flushInput()

    # Start GPS (takes ~2 seconds)
    power_key = 6
    power_on(ser, power_key)
    send_at(ser, 'AT+CGPS=1,1','OK',1)
    sleep(2)

    while True:
        result = get_gps_position(ser)
        if result["success"]:
            GPS_DATA = result["data"]
            LAST_GPS_Data = result["data"]
            update_device_data()
        else:
            GPS_DATA = None

# Sense Hat
SENSE_HAT_DATA = {}
ACCData=[0.0]*8
GYROData=[0.0]*8
AngleData=[0.0]*8          
FrameState = 0            #通过0x后面的值判断属于哪一种情况
Bytenum = 0               #读取到这一段的第几位
CheckSum = 0              #求和校验位         
 
a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3
def DueData(inputdata):   #新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
    global  FrameState    #在局部修改全局变量，要进行global的定义
    global  Bytenum
    global  CheckSum
    global  a
    global  w
    global  Angle
    ORIENTATION_DATA = {}
    for data in inputdata:  #在输入的数据进行遍历
        #Python2软件版本这里需要插入 data = ord(data)*****************************************************************************************************
        if FrameState==0:   #当未确定状态的时候，进入以下判断
            if data==0x55 and Bytenum==0: #0x55位于第一位时候，开始读取数据，增大bytenum
                CheckSum=data
                Bytenum=1
                continue
            elif data==0x51 and Bytenum==1:#在byte不为0 且 识别到 0x51 的时候，改变frame
                CheckSum+=data
                FrameState=1
                Bytenum=2
            elif data==0x52 and Bytenum==1: #同理
                CheckSum+=data
                FrameState=2
                Bytenum=2
            elif data==0x53 and Bytenum==1:
                CheckSum+=data
                FrameState=3
                Bytenum=2
        elif FrameState==1: # acc    #已确定数据代表加速度
            
            if Bytenum<10:            # 读取8个数据
                ACCData[Bytenum-2]=data # 从0开始
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):  #假如校验位正确
                    a = get_acc(ACCData)
                CheckSum=0                  #各数据归零，进行新的循环判断
                Bytenum=0
                FrameState=0
        elif FrameState==2: # gyro
            
            if Bytenum<10:
                GYROData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    w = get_gyro(GYROData)
                CheckSum=0
                Bytenum=0
                FrameState=0
        elif FrameState==3: # angle
            
            if Bytenum<10:
                AngleData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    Angle = get_angle(AngleData)
                    ORIENTATION_DATA = {"pitch_accl": round(a[0], 2), 
                                        "roll_accl": round(a[1], 2), 
                                        "yaw_accl": round(a[2], 2), 
                                        "pitch_gyro": round(w[0], 2), 
                                        "roll_gyro": round(w[0], 2), 
                                        "yaw_gyro": round(w[0], 2), 
                                        "pitch": round(Angle[0], 2), 
                                        "roll": round(Angle[1], 2), 
                                        "yaw": round(abs(Angle[2]) if Angle[2] < 0 else 360 - Angle[2], 2)}
                    return {"success": True, "data": ORIENTATION_DATA}
                CheckSum=0
                Bytenum=0
                FrameState=0

    return {"success": False}
 
def get_acc(datahex):  
    axl = datahex[0]                                        
    axh = datahex[1]
    ayl = datahex[2]                                        
    ayh = datahex[3]
    azl = datahex[4]                                        
    azh = datahex[5]
    
    k_acc = 16.0
 
    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z-= 2 * k_acc
    
    return [acc_x,acc_y,acc_z]
 
def get_gyro(datahex):                                      
    wxl = datahex[0]                                        
    wxh = datahex[1]
    wyl = datahex[2]                                        
    wyh = datahex[3]
    wzl = datahex[4]                                        
    wzh = datahex[5]
    k_gyro = 2000.0
 
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >=k_gyro:
        gyro_z-= 2 * k_gyro
    return [gyro_x,gyro_y,gyro_z]
 
def get_angle(datahex):                                 
    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
 
    return [angle_x,angle_y,angle_z]
 
def run_orientation():
    global SENSE_HAT_DATA
    ser = serial.Serial('/dev/' + get_ttyUSB_port("pl2303")[0], baudrate = 9600, timeout = 2)
    # print(ser.is_open)
    count = 0
    while(1):
        datahex = ser.read(33)
        result = DueData(datahex)
        if result["success"]:
            SENSE_HAT_DATA = result["data"]
            update_device_data()

        if result["success"]:
            count = 0
        else:
            count += 1
        if count == 3:
            count = 0
            ser.close()
            sleep(1)
            ser = serial.Serial('/dev/' + get_ttyUSB_port("pl2303")[0], baudrate = 9600, timeout = 2)

# battery
import re

class UPS2:
    def __init__(self,port):
        self.ser  = serial.Serial(port,9600)        
        
    def get_data(self,nums):
        while True:
            self.count = self.ser.inWaiting()
            if self.count !=0:
                self.recv = self.ser.read(nums)
                return self.recv
    
    def decode_uart(self):
        self.uart_string = self.get_data(100)
        self.data = self.uart_string.decode('ascii','ignore')
        self.pattern = r'\$ (.*?) \$'
        self.result = re.findall(self.pattern,self.data,re.S)
        self.tmp = self.result[0]
        self.pattern = r'SmartUPS (.*?),'
        self.version = re.findall(self.pattern,self.tmp)
        self.pattern = r',Vin (.*?),'
        self.vin = re.findall(self.pattern,self.tmp)
        self.pattern = r'BATCAP (.*?),'
        self.batcap = re.findall(self.pattern,self.tmp)
        self.pattern = r',Vout (.*)'
        self.vout = re.findall(self.pattern,self.tmp)
        return self.version[0],self.vin[0],self.batcap[0],self.vout[0]

UPS_SERIAL = UPS2("/dev/ttyAMA0")
BATTERY_DATA = {}
def battery_info_read():
    global UPS_SERIAL, BATTERY_DATA
    try:
        version, vin, batcap, vout = UPS_SERIAL.decode_uart()
        connected = vin != "NG"
        BATTERY_DATA = {"voltage":          {"amount": vout, "unit": "mV"},
                        "battery_level":    {"amount": batcap, "unit": "%"},
                        "charging":         connected}
    except:
        print("Battery read went wrong")

# from ina219 import INA219
# from ina219 import DeviceRangeError
# SHUNT_OHMS = 0.1
# BATTERY_DATA = {}
# def battery_info_read():
#     global BATTERY_DATA
#     ina = INA219(SHUNT_OHMS, busnum=1)
#     ina.configure()
#     try:
#         battery_level = (float(ina.voltage()) - 10.8) / (12.044 - 10.8)
#         BATTERY_DATA = {"voltage":          {"amount": ina.voltage(), "unit": "V"},
#                         "current":          {"amount": ina.current(), "unit": "mA"},
#                         "power":            {"amount": ina.power(), "unit": "mW"},
#                         "shunt_voltage":    {"amount": ina.shunt_voltage(), "unit": "mV"},
#                         "battery_level":    {"amount": battery_level, "unit": "%"}}
#     except DeviceRangeError as e:
#         BATTERY_DATA = {}

def run_battery():
    while True:
        battery_info_read()
        update_device_data()
        sleep(1)

# run sensor
DEVICE_DATA = {}
def update_device_data():
    global DEVICE_DATA, SENSE_HAT_DATA, LAST_GPS_Data, BATTERY_DATA

    DEVICE_DATA = SENSE_HAT_DATA.copy()
    if LAST_GPS_Data is not None:
        DEVICE_DATA["lat"] = float(LAST_GPS_Data[0])
        DEVICE_DATA["lon"] = float(LAST_GPS_Data[2])
        DEVICE_DATA["alt"] = float(LAST_GPS_Data[6])
        DEVICE_DATA["H_speed"] = float(LAST_GPS_Data[7])
    DEVICE_DATA["time"] = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f%Z")
    DEVICE_DATA["battery_info"] = BATTERY_DATA

threading.Thread(target=run_orientation).start()
threading.Thread(target=run_gps).start()
threading.Thread(target=run_battery).start()

#############################
# Sensors END
#############################

#############################
# Controls START
#############################

# Motor
import os
os.system("sudo pigpiod")
sleep(1)
import pigpio

ESC = 24
max_value = 2000 
min_value = 1000
mid_value = 1500

# calibration and arm esc
esc_pi = pigpio.pi();
# esc_pi.set_servo_pulsewidth(ESC, 0) 
# sleep(2)
# esc_pi.set_servo_pulsewidth(ESC, max_value)
# sleep(2)
# esc_pi.set_servo_pulsewidth(ESC, min_value)
# sleep(4)
# esc_pi.set_servo_pulsewidth(ESC, mid_value)
# sleep(5)

def move_motor(percentage):
    global esc_pi, ESC, max_value, min_value, mid_value
    diff = 0
    if percentage > 0 and percentage <= 1:
        diff = (max_value - mid_value) * percentage
    if percentage < 0 and percentage >= -1:
        diff = (mid_value - min_value) * percentage
    esc_pi.set_servo_pulsewidth(ESC, int(diff + mid_value))

# Servo 
# subprocess.Popen(["sudo", "pigpiod"])
import gpiozero
# from gpiozero.pins.pigpio import PiGPIOFactory
in_servo = 23
servo = gpiozero.Servo(in_servo) # , pin_factory=PiGPIOFactory()

#############################
# Controls END
#############################

print("DEVICE SET UP READY")

#############################
# From viewer START
#############################

from math import sin, cos, sqrt, atan2, radians, asin, degrees, tan, pi

class AutoSailing:

    COMMON_RUDDER_ANGLE = 20    # degree
    ERROR_OF_TOLERANT = 5       # meter

    COORDINATES = []
    LAST_COORDINATE = None
    REMAIN_COORDINATES = []

    def __init__(self, vessel_length):
        self.VESSEL_LENGTH = vessel_length           # meter

    def register(self, coordinates, vessel_coord):

        # x is rudder angle, y is diameter in meter
        coefficients = self.calc_parabola_vertex(35, 3.5 * self.VESSEL_LENGTH, 20, 3.5 * self.VESSEL_LENGTH * 555 / 358, 10, 3.5 * self.VESSEL_LENGTH  * 794 / 358)
        diameter = coefficients[0] * (abs(self.COMMON_RUDDER_ANGLE) ** 2) + coefficients[1] * abs(self.COMMON_RUDDER_ANGLE) + coefficients[2]

        distance_limit = diameter * 2

        coords = [vessel_coord] + coordinates
        for i in range(1, len(coords)):
            distance = self.get_data_between_two_coordinates(coords[i - 1], coords[i])["distance_in_KM"]
            if distance * 1000 < distance_limit:
                return {"success": False, "error": {"code": 2003, "message": "Distance between two coordinates are less than " + str(distance_limit) + "m", "data": {"distanceInMeter": distance_limit}}}

        self.COORDINATES = coordinates
        self.LAST_COORDINATE = vessel_coord
        self.REMAIN_COORDINATES = coordinates

        return {"success": True}

    def update(self, vessel_coord, vessel_heading):

        next_coordinate = self.REMAIN_COORDINATES[0]
        if self.get_data_between_two_coordinates(vessel_coord, next_coordinate)["distance_in_KM"] * 1000 < self.ERROR_OF_TOLERANT:
            self.LAST_COORDINATE = self.REMAIN_COORDINATES.pop(0)
            if len(self.REMAIN_COORDINATES) == 0:
                return {"success": True, "data": {"done": True, "power": 0, "angle": 0}}
            next_coordinate = self.REMAIN_COORDINATES[0]

        # offset_distance = self.get_distance_perpendicular_to_two_coordinates(vessel_coord, self.LAST_COORDINATE, next_coordinate)

        data = self.get_data_between_two_coordinates(vessel_coord, next_coordinate)
        bank_angle = self.get_bank_angle(vessel_heading, data["bearing"])
        # if abs(bank_angle) > 35:
        #     return {"success": True, "data": {"done": False, "power": 0.2, "angle": 35 if bank_angle > 0 else -35}}
        # elif abs(bank_angle) > 20:
        #     return {"success": True, "data": {"done": False, "power": 0.4, "angle": bank_angle}}
        # elif abs(bank_angle) > 10:
        #     return {"success": True, "data": {"done": False, "power": 0.6, "angle": bank_angle}}
        # elif abs(bank_angle) > 5:
        #     return {"success": True, "data": {"done": False, "power": 0.8, "angle": bank_angle}}
        # else:
        #     return {"success": True, "data": {"done": False, "power": 1, "angle": bank_angle}}
        if abs(bank_angle) > 30:
            return {"success": True, "data": {"done": False, "power": 0.2, "angle": 30 if bank_angle > 0 else -30}}
        elif abs(bank_angle) > 20:
            return {"success": True, "data": {"done": False, "power": 0.2, "angle": int(bank_angle / 2)}}
        elif abs(bank_angle) > 10:
            return {"success": True, "data": {"done": False, "power": 0.2, "angle": int(bank_angle / 3)}}
        else:
            return {"success": True, "data": {"done": False, "power": 0.2, "angle": int(bank_angle / 4)}}

    # def get_sideway_distance_in_meter(self, rudder_angle, bank_angle):

    #     # x is rudder angle, y is diameter
    #     coefficients = self.calc_parabola_vertex(35, 3.5 * self.VESSEL_LENGTH, 20, 3.5 * self.VESSEL_LENGTH * 555 / 358, 10, 3.5 * self.VESSEL_LENGTH  * 794 / 358)
    #     diameter = coefficients[0] * (abs(rudder_angle) ** 2) + coefficients[1] * abs(rudder_angle) + coefficients[2]
    #     radius = diameter / 2

    #     distance = (radius ** 2 + radius ** 2 - 2 * radius * radius * cos(radians(abs(bank_angle)))) ** 0.5
    #     sideway_distance = ((distance ** 2) / (2 * (1 - cos(radians(180 - bank_angle))))) ** 0.5
        
    #     return sideway_distance

    # def get_intersection(self, coordA, headingA, coordB, headingB):

    #     distance = self.get_data_between_two_coordinates(coordA, coordB)["distance_in_KM"]
    #     bank_angle = self.get_bank_angle(headingA, headingB)
    #     side = ((distance ** 2) / (2 * (1 - cos(radians(180 - abs(bank_angle)))))) ** 0.5
    #     target_coordinate = self.get_target_coordinate(coordA, side, headingA)

    #     return target_coordinate

    # def get_target_coordinate(self, coord, distance_in_KM, bearing):

    #     R = 6372.8                  # approximate radius of earth in km
    #     lat1 = radians(coord[0])
    #     lon1 = radians(coord[1])

    #     lat2 = asin(sin(lat1) * cos(distance_in_KM / R) + cos(lat1) * sin(distance_in_KM / R) * cos(radians(bearing)))
    #     lon2 = lon1 + atan2(sin(radians(bearing)) * sin(distance_in_KM / R) * cos(lat1), cos(distance_in_KM / R) - sin(lat1) * sin(lat2))
    #     lat2 = degrees(lat2)
    #     lon2 = degrees(lon2)

    #     return [lat2, lon2]

    # def get_distance_perpendicular_to_two_coordinates(self, target_coord, coordA, coordB):

    #     b_to_a_bearing = self.get_data_between_two_coordinates(coordB, coordA)["bearing"]
    #     b_to_target_data = self.get_data_between_two_coordinates(coordB, target_coord)
    #     b_to_target_bearing = b_to_target_data["bearing"]
    #     abs_bank_angle = abs(self.get_bank_angle(b_to_target_bearing, b_to_a_bearing))
    #     if abs_bank_angle > 90:
    #         abs_bank_angle = 180 - abs_bank_angle
    #     distance = b_to_target_data["distance_in_KM"] * sin(radians(abs_bank_angle))

    #     return distance

    def get_bank_angle(self, headingA, headingB):

        bank_angle = headingB - headingA
        if bank_angle < 0:
            bank_angle = 360 + bank_angle
        if bank_angle > 180:
            bank_angle = bank_angle - 360

        return bank_angle

    def get_data_between_two_coordinates(self, coordA, coordB):

        R = 6372.8                  # approximate radius of earth in km
        lat1 = radians(coordA[0])
        lon1 = radians(coordA[1])
        lat2 = radians(coordB[0])
        lon2 = radians(coordB[1])

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance_in_KM = R * c

        bearing = atan2(sin(lon2 - lon1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1))
        bearing = (degrees(bearing) + 360) % 360

        return {"distance_in_KM": distance_in_KM, "distance_in_NM": distance_in_KM / 1.852, "bearing": bearing}

    def calc_parabola_vertex(self, x1, y1, x2, y2, x3, y3):

        denom = (x1-x2) * (x1-x3) * (x2-x3);
        A     = (x3 * (y2-y1) + x2 * (y1-y3) + x1 * (y3-y2)) / denom;
        B     = (x3*x3 * (y1-y2) + x2*x2 * (y3-y1) + x1*x1 * (y2-y3)) / denom;
        C     = (x2 * x3 * (x2-x3) * y1+x3 * x1 * (x3-x1) * y2+x1 * x2 * (x1-x2) * y3) / denom;

        return A,B,C

class Device:

    SELF_UPDATE_TIME = 2        # sec
    AUTO_SAILING = None
    websocket = None

    CONTROL_POWER = 0
    MIN_CONTROL_POWER = 0
    MAX_CONTROL_POWER = 1 

    CONTROL_ANGLE = 0
    CALIBRATION_CONTROL_ANGLE = 0
    IS_OPPOSITE_CONTROL_ANGLE = False
    ANGLE_LIMIT = 35

    def __init__(self, uid, session):
        self.uid = uid
        self.send_data_is_running = False
        self.send_data_is_waiting = False
        self.session = session

    def __str__(self):
        printout = "ID: " + self.uid + "\n" + \
                    "AUTO_SAILING: " + str(self.AUTO_SAILING is not None) + "\n" + \
                    "CONTROL_POWER: " + str(self.CONTROL_POWER) + "\n" + \
                    "MIN_CONTROL_POWER: " + str(self.MIN_CONTROL_POWER) + "\n" + \
                    "MAX_CONTROL_POWER: " + str(self.MAX_CONTROL_POWER) + "\n" + \
                    "CONTROL_ANGLE: " + str(self.CONTROL_ANGLE) + "\n" + \
                    "CALIBRATION_CONTROL_ANGLE: " + str(self.CALIBRATION_CONTROL_ANGLE) + "\n" + \
                    "IS_OPPOSITE_CONTROL_ANGLE: " + str(self.IS_OPPOSITE_CONTROL_ANGLE)
        return printout

    def adjust_update_time(self, time):
        self.SELF_UPDATE_TIME = time

    async def send_data(self, on):
        if on:
            # send data to websocket
            if not self.send_data_is_waiting:
                asyncio.create_task(self.send_device_data())
            self.send_data_is_running = True
        else:
            # stop send data to websocket
            self.send_data_is_running = False

    async def send_device_data(self):
        while self.send_data_is_running:
            self.send_data_is_waiting = True
            await self.send_device_data_to_viewer()
            await asyncio.sleep(self.SELF_UPDATE_TIME)
            self.send_data_is_waiting = False

    async def send_device_data_to_viewer(self):
        # global SERVER_LATENCY
        auto_sailing_data = {"on": self.AUTO_SAILING is not None}
        if self.AUTO_SAILING is not None:
            auto_sailing_data["coordinates"] = self.AUTO_SAILING.COORDINATES

        global DEVICE_DATA
        data = {
                    "sensor_data": DEVICE_DATA, 
                    "control_data": 
                        {
                            "power": self.CONTROL_POWER, 
                            "min_power": self.MIN_CONTROL_POWER,
                            "max_power": self.MAX_CONTROL_POWER,
                            "power_in_percentage": self.get_adjusted_power(),
                            "angle": self.CONTROL_ANGLE, 
                            "calibration_angle": self.CALIBRATION_CONTROL_ANGLE,
                            "is_opposite_angle": self.IS_OPPOSITE_CONTROL_ANGLE,
                            "autoSailing": auto_sailing_data
                        },
                    "viewers": len(self.session.viewers)
                    # "camera_host": CAMERA_HOST, 
                    # "ssh_host": SSH_HOST,
                    # "latency": SERVER_LATENCY
                }
        await self.session.send_to_viewers({"action": "device_data"}, {"success": True, "data": data})

    async def action(self, name, data):
        if name == "control":
            if self.AUTO_SAILING is None:
                return await self.control(data["power"], data["angle"])   
            else:
                return {"success": False, "error": {"code": 2002, "message": "Auto sailing is enabled"}}
        elif name == "control_auto":
            if "on" in data and data["on"]:
                if self.AUTO_SAILING is None:
                    return self.control_auto(data["coordinates"]) 
                else:
                    return {"success": False, "error": {"code": 2002, "message": "Auto sailing is enabled"}}
            else:
                self.control_auto_stop()
                return await self.control(0, 0)
        elif name == "power_angle_setting":
            response = await self.adjust_power_angle_setting(data["min_power"], data["max_power"], data["calibration_angle"], data["is_opposite_angle"])
            if response["success"]:
                await SERVER.save_device_params(data)
            return response

    async def ini_power_angle_setting(self, min_power, max_power, calibration_angle, is_opposite_angle):
        ini_min_power = self.MIN_CONTROL_POWER if min_power is None else min_power
        ini_max_power = self.MAX_CONTROL_POWER if max_power is None else max_power
        ini_calibration_angle = self.CALIBRATION_CONTROL_ANGLE if calibration_angle is None else calibration_angle
        ini_is_opposite_angle = self.IS_OPPOSITE_CONTROL_ANGLE if is_opposite_angle is None else is_opposite_angle
        return await self.adjust_power_angle_setting(ini_min_power, ini_max_power, ini_calibration_angle, ini_is_opposite_angle)

    async def adjust_power_angle_setting(self, min_power, max_power, calibration_angle, is_opposite_angle):
        if min_power is None or not (isinstance(min_power, float) or isinstance(min_power, int)) or min_power < 0 or min_power > 0.99 \
            or max_power is None or not (isinstance(max_power, float) or isinstance(max_power, int)) or max_power < 0.01 or max_power > 1 \
            or min_power >= max_power \
            or calibration_angle is None or not isinstance(calibration_angle, int) or calibration_angle < -45 or calibration_angle > 45 \
            or is_opposite_angle is None or not isinstance(is_opposite_angle, bool):
            return {"success": False, "error": {"code": 2005, "message": "Format for power or angle setting is not correct"}}
        
        self.MIN_CONTROL_POWER = min_power
        self.MAX_CONTROL_POWER = max_power 
        self.CALIBRATION_CONTROL_ANGLE = calibration_angle
        self.IS_OPPOSITE_CONTROL_ANGLE = is_opposite_angle
        asyncio.create_task(self.control_servo())
        return {"success": True}

    async def control_auto_completed(self):
        self.control_auto_stop()
        await self.session.send_to_viewers({"action": "control_auto_completed"}, {"success": True})

    async def auto_sailing_running(self):
        global DEVICE_DATA
        while self.AUTO_SAILING is not None:
            if "lat" in DEVICE_DATA and "lon" in DEVICE_DATA and "yaw" in DEVICE_DATA:
                response = self.AUTO_SAILING.update([DEVICE_DATA["lat"], DEVICE_DATA["lon"]], DEVICE_DATA["yaw"])
                if response["success"]:
                    data = response["data"]
                    self.adjust_update_time(1 if data["power"] == 0 else 1 / (6 - abs(int(data["power"] * 5))))
                    await self.control(data["power"], round(data["angle"] / self.ANGLE_LIMIT, 2))
                    if data["done"]:
                        await self.control_auto_completed()

            await asyncio.sleep(self.SELF_UPDATE_TIME)

    def control_auto_stop(self):
        self.AUTO_SAILING = None
        self.adjust_update_time(2)

    def control_auto(self, coordinates):
        global DEVICE_DATA
        if "lat" not in DEVICE_DATA or "lon" not in DEVICE_DATA:
            # self.DEVICE_DATA["lat"] = 22.416792
            # self.DEVICE_DATA["lon"] = 114.222437
            return {"success": False, "error": {"code": 2004, "message": "GPS is not active"}}

        self.AUTO_SAILING = AutoSailing(1.3)
        result = self.AUTO_SAILING.register(coordinates, [DEVICE_DATA["lat"], DEVICE_DATA["lon"]])
        if result["success"]:
            self.adjust_update_time(0.2)
            asyncio.create_task(self.auto_sailing_running())
        else:
            self.AUTO_SAILING = None
        return result

    async def control(self, power, angle):

        if power < -1 or power > 1 or not (isinstance(power, float) or isinstance(power, int)) \
            or angle < -1 or angle > 1 or not (isinstance(angle, float) or isinstance(angle, int)):
            return {"success": False, "error": {"code": 2001, "message": "Input is not correct"}} 

        if self.CONTROL_POWER != power:
            self.CONTROL_POWER = power
            adjusted_power = self.get_adjusted_power()
            move_motor(adjusted_power)

        if self.CONTROL_ANGLE != angle:
            self.CONTROL_ANGLE = angle
            asyncio.create_task(self.control_servo())

        # print("CONTROL_POWER: " + str(self.CONTROL_POWER))
        # print("CONTROL_ANGLE: " + str(self.CONTROL_ANGLE))

        # power_was_on = self.CONTROL_POWER != 0
        # if power == 0:
        #     move_motor(0)
        # elif not power_was_on:
        #     asyncio.create_task(self.control_engine())

        return {"success": True}

    # turning (negative is left turn and positive is right turn)
    async def control_servo(self):
        global servo
        angle = (round(self.CONTROL_ANGLE * self.ANGLE_LIMIT) + self.CALIBRATION_CONTROL_ANGLE) / 80 # 80 is the physcial motor max angle
        servo.value = -1 * angle if self.IS_OPPOSITE_CONTROL_ANGLE else angle
        await asyncio.sleep(0.5)

    def get_adjusted_power(self):
        if self.CONTROL_POWER == 0:
            return 0
        adjusted_power = self.MIN_CONTROL_POWER + (self.MAX_CONTROL_POWER - self.MIN_CONTROL_POWER) * abs(self.CONTROL_POWER)
        adjusted_power = adjusted_power if self.CONTROL_POWER >= 0 else adjusted_power * -1
        # segment = (self.MAX_CONTROL_POWER - self.MIN_CONTROL_POWER) * 100 / 5
        # adjusted_power = (self.MIN_CONTROL_POWER * 100 + abs(self.CONTROL_POWER) * segment) / 100
        # adjusted_power = adjusted_power if self.CONTROL_POWER >= 0 else adjusted_power * -1
        return adjusted_power

    # async def control_engine(self):
    #     while self.CONTROL_POWER != 0:
    #         adjusted_power = self.get_adjusted_power()
    #         move_motor(adjusted_power)
    #         await asyncio.sleep(1)

class Viewer:

    def __init__(self, uid):
        self.uid = uid

    def __str__(self):
        return self.uid

    def __eq__(self, other):
        if (isinstance(other, Viewer)):
            return self.uid == other.uid
        return False

    def update_data(self, websocket):
        self.websocket = websocket

class Session:

    def __init__(self, device_id):
        self.device = Device(device_id, self)
        self.viewers = []

    def __str__(self):
        for viewer in self.viewers:
            print("Session: " + str(viewer))
        return "Number of Viewers: " + str(len(self.viewers))

    def get_viewer(self, uid):
        for viewer in self.viewers:
            if viewer.uid == uid:
                return viewer
        return None

    def get_viewer_by_websocket(self, websocket):
        for viewer in self.viewers:
            if viewer.websocket == websocket:
                return viewer
        return None

    async def save_viewer(self, websocket, viewer_uid):
        viewer = self.get_viewer(viewer_uid)
        if viewer is None:
            viewer = Viewer(viewer_uid)
            self.viewers.append(viewer)
        viewer.update_data(websocket)
        await self.number_of_viewer_changed()
        return {"success": True}

    async def remove_viewer_by_socket(self, websocket):
        viewer = self.get_viewer_by_websocket(websocket)
        if viewer is not None:
            print("remove_viewer_by_socket found: " + viewer.uid)
            index = None
            for i in range(len(self.viewers)):
                if self.viewers[i].uid == viewer.uid:
                    index = i
            self.viewers.pop(index)
            await self.number_of_viewer_changed()

    async def number_of_viewer_changed(self):
        await self.device.send_data(len(self.viewers) > 0)
        if len(self.viewers) == 0 and self.device.AUTO_SAILING is None:
            await self.device.control(0, 0)

    async def register_viewer(self, websocket, data):
        viewer_uid = data["viewer_uid"] if "viewer_uid" in data else None
        if viewer_uid is not None:
            return await self.save_viewer(websocket, viewer_uid)
        else:
            return {"success": False, "error": {"code": 1002, "message": "Incorrect parameters"}}

    async def send_to_device(self, action, data, session_uid, frm = None):
        viewers = self.viewers
        if session_uid is not None:
            session = self.get_viewer(session_uid)
            if session is not None:
                viewers = [session]
        for session in viewers:
            await session.send_to_device(action, data, frm)

    async def send_to_viewers(self, request, response, viewer_uid = None):
        found_viewers = []
        if viewer_uid is None:
            found_viewers = self.viewers
        else:
            viewer = self.get_viewer(viewer_uid)
            if viewer is not None:
                found_viewers = [viewer]
        for viewer in found_viewers:
            await send_message_to_socket(viewer.websocket, request ,response)

async def send_message_to_socket(websocket, request, response):
    request["response"] = response
    if request["action"] == "device_data" or request["action"] == "get_latency":
        print("SEND DATA (VIEWER SIDE): " + request["action"])
    else:
        print("SEND DATA (VIEWER SIDE): " + json.dumps(request))
    await websocket.send(json.dumps(request))

async def request_data_from_user_to_device(websocket, request):
    action = request["action"]
    data = request["data"] if "data" in request else None

    if action == "register":
        response = await SESSION.register_viewer(websocket, data)
        print("-----REGISTER (VIEWER SIDE)-----")
        print(SESSION)
        print("------------------")
        await send_message_to_socket(websocket, request, response)
    elif action == "control" or action == "control_auto" or action == "power_angle_setting":
        response = await SESSION.device.action(action, data)
        if action == "control_auto" and not ("on" in data and data["on"]):
            await SESSION.send_to_viewers(request, response)
        else:
            await send_message_to_socket(websocket, request, response)
    else:
        await send_message_to_socket(websocket, request, {"success": False, "error": {"code": 1000, "message": "Incorrect action request"}})

async def unregister(websocket):
    await SESSION.remove_viewer_by_socket(websocket)
    print("----UNREGISTER (VIEWER SIDE)----")
    print(SESSION)
    print("------------------")
    # # Servo
    # # subprocess.Popen(["sudo", "killall", "pigpiod"])

async def init_wss_as_server(websocket, path):
    try:
        async for message in websocket:
            request = json.loads(message)
            print("RECEIVE DATA (VIEWER SIDE): " + message)
            if "action" in request:
                await request_data_from_user_to_device(websocket, request)
            else:
                await send_message_to_socket(websocket, request, {"success": False, "error": {"code": 1000, "message": "Incorrect action request"}})
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        await unregister(websocket)

SESSION = Session(DEVICE_ID)
# start_server = websockets.serve(init_wss_as_server, "0.0.0.0", 8443)
# asyncio.get_event_loop().run_until_complete(start_server)
# asyncio.get_event_loop().run_forever()

#############################
# From viewer END
#############################

#############################
# To Server START
#############################

class Server:

    CAMERA_HOST = {}
    SSH_HOST = {}
    WSS_HOST = {}

    def __init__(self, uid):
        self.uid = uid

    def get_host(self):
        # start camera server and Reverse SSH Tunneling
        path = str(pathlib.Path(__file__).parent.resolve())
        camera_reverse_tunneling_process = subprocess.Popen("exec " + path + "/ngrok start --all > /dev/null &", shell=True)
        camera_ws_server_process = subprocess.Popen(["node", path + "/raspivid-broadcaster/ws-server.js"])
        while len(self.CAMERA_HOST) == 0:
            try:
                reverse_tunneling_requests = requests.get('http://127.0.0.1:4040/api/tunnels')
                reverse_tunnels = reverse_tunneling_requests.json()["tunnels"]
                for reverse_tunnel in reverse_tunnels:
                    tunneling_url = reverse_tunnel["public_url"].split("://")[1]
                    if reverse_tunnel["uri"] == "/api/tunnels/first":
                        self.CAMERA_HOST = {"hostname": tunneling_url.split(":")[0], "port": None}#tunneling_url.split(":")[1]}
                    if reverse_tunnel["uri"] == "/api/tunnels/second":
                        self.SSH_HOST = {"hostname": tunneling_url.split(":")[0], "port": tunneling_url.split(":")[1]}
                    if reverse_tunnel["uri"] == "/api/tunnels/third":
                        self.WSS_HOST = {"hostname": tunneling_url.split(":")[0], "port": None}#tunneling_url.split(":")[1]}
            except:
                sleep(3)

    async def register(self, websocket):
        self.websocket = websocket
        await self.send_message_to_socket("register", {"device_uid": self.uid, "ssh_host": self.SSH_HOST, "camera_host": self.CAMERA_HOST, "control_host": self.WSS_HOST})

    async def save_device_params(self, data):
        data["device_uid"] = self.uid
        await self.send_message_to_socket("save_device_params", data)

    def deregister(self):
        self.websocket = None
        # # stop camera server and Reverse SSH Tunneling
        # camera_reverse_tunneling_process.kill()
        # # ngrok still run outside this process, so call below to stop all ngrok processes
        # subprocess.Popen("exec " + "killall ngrok", shell=True)
        # camera_ws_server_process.kill()
        # CAMERA_HOST = {}
        # SSH_HOST = {}
        # print("clean up")

    async def send_message_to_socket(self, action, data):
        dictionary = {"action": action, "data": data}
        print("SEND DATA (SERVER SIDE): " + json.dumps(dictionary))
        await self.websocket.send(json.dumps(dictionary))

    async def do(self, data):
        if data["response"]["success"]:
            if data["action"] == "register":
                response_data = data["response"]["data"]
                if "data" in response_data:
                    device_data = response_data["data"]
                    min_power = device_data["min_power"] if "min_power" in device_data else None
                    max_power = device_data["max_power"] if "max_power" in device_data else None
                    calibration_angle = device_data["calibration_angle"] if "calibration_angle" in device_data else None
                    is_opposite_angle = device_data["is_opposite_angle"] if "is_opposite_angle" in device_data else None
                    ini_result = await SESSION.device.ini_power_angle_setting(min_power, max_power, calibration_angle, is_opposite_angle)

async def init_wss_as_client():
    global SERVER
    try:
        async with websockets.connect(URI) as websocket:
            SERVER.get_host()
            await SERVER.register(websocket)

            while True:
                message = await websocket.recv()
                print('RECEIVE DATA (SERVER SIDE):' + message)
                dictionary = json.loads(message)
                await SERVER.do(dictionary)
    finally:
        SERVER.deregister()
        print("DISCONNECTED (SERVER SIDE)")

        sleep(5)
        await init_wss_as_client()

SERVER = Server(DEVICE_ID)
# asyncio.run(init_wss_as_client())

#############################
# To Server END
#############################

# sleep(30)

loop = asyncio.get_event_loop()
loop.create_task(init_wss_as_client())
loop.run_until_complete(websockets.serve(init_wss_as_server, "0.0.0.0", 8443))
loop.run_forever()




