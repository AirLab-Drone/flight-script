import serial
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading

global dis 
def sensor(H,L):
    Dis = int(H)*256 + int(L)
    return Dis
def readus(port1):
    '''
    @param port1 串口
    @return fs1,fs2,fs3,fs4 4個傳感器的距離值,單位mm
    '''
    in_byte = [0,0,0,0,0,0,0,0,0,0,0,0]
    #0X55 0XAA 0X01 0X01 checksum
    command = [0x55,0xAA,0x01,0x01,0x01]
    command = bytearray(command)
    # while 1 :
    # port1.flushInput()
    port1.write(command)
    time.sleep(0.3)
    rcv1 = port1.read(1)
    print(rcv1)
    head = ord(rcv1)
    print(head)
    if(head == 85):
        for i in range(len(in_byte)-2):
            rcv1 = port1.read(1)
            in_byte[i] = ord(rcv1)
            # print(in_byte[i])
        #print("received = ", in_byte2)
        checksum = 0x55
        for i in range(0,11):
            checksum += in_byte[i]
        checksum=checksum&0x00FF
        #print(checksum)
        fs1 = sensor(in_byte[3], in_byte[4])
        if fs1 == 0:fs1=4500
        fs2 = sensor(in_byte[5], in_byte[6])
        if fs2 == 0:fs2=4500
        fs3 = sensor(in_byte[7], in_byte[8])
        if fs3 == 0:fs3=4500
        fs4 = sensor(in_byte[9], in_byte[10])
        if fs4 == 0:fs4=4500
        print("S = ", fs1, ",", fs2 ,",", fs3 ,",", fs4)
        # period = 250 ms
        # time.sleep(0.25)
        print(in_byte)
        return fs1,fs2,fs3,fs4
    return 0,0,0,0
#單個讀取
def readusOne(port1,read=1):#read=1~4
    in_byte1 = [0,0,0,0,0,0]
    #0X55 0XAA 0X01 0X01 checksum
    a = 0x10 + read-1
    checksum = sum([0x55 , 0xAA , 0x01 , a]) & 0x00FF
    command = [0x55,0xAA,0x01,a,checksum]
    command = bytearray(command)
    port1.write(command)
    # period = 90 ms
    time.sleep(0.1)#單個讀取的響應時間比較快
    rcv1 = port1.read(1)
    head = ord(rcv1)
    if(head == 85):
        checksum = 0x55
        for i in range(len(in_byte1)):
            rcv1 = port1.read(1)
            in_byte1[i] = ord(rcv1)
            if i < 5:
                checksum += in_byte1[i]
        checksum = checksum & 0x00FF
        print('check sum: ',checksum)
        if checksum == in_byte1[5]:#package not loss
            s = sensor(in_byte1[3], in_byte1[4])
            print(f'raw s: {s}')
            if s == 0:s=4500 #超出量程

        else:#package loss
            s = 9999
    else:
        s = 9999
    return s
def sendRangeFinder(distance):
    # 创建一个测距仪消息
    msg = vehicle.message_factory.distance_sensor_encode(
        0,  # 时间微秒（0表示当前时间）
        25,                         # 最小距离
        450,                       # 最大距离
        distance,                  # 当前距离
        1,                         # 类型（0表示超声波）
        0,                         # ID
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # 方向（0表示向下）
        0                          # 信号强度
    )
    # 发送测距仪消息
    # print("sendRangeFinder: ",distance,"cm")
    vehicle.send_mavlink(msg)
def send_distance():
    global dis
    while True:
        try:
            if(dis == 9999):
                continue
            sendRangeFinder(int(dis/10))
            time.sleep(0.05)
        except:
            continue
def read_distance():
    global dis
    while True:
        try:
            dis = readusOne(port1,1)
            print("dis=",dis)
        except Exception as e: 
            print(e)
            continue
if __name__ == '__main__':
    global dis
    port1 = serial.Serial("/dev/serial0", baudrate = 9600, timeout = 2)
    vehicle = connect('/dev/ttyACM0',baud=9600)
    dis = 0
    thread1 = threading.Thread(target=read_distance)
    thread2 = threading.Thread(target=send_distance)
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
