import serial
import time
# import matplotlib.pyplot as plt

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
    port1.write(command)
    time.sleep(0.5)
    rcv1 = port1.read(1)
    time.sleep(0.5)
    print(rcv1)
    head = ord(rcv1)
    #print(head)
    if(head == 85):
        for i in range(len(in_byte)):
            rcv1 = port1.read(1)
            in_byte[i] = ord(rcv1)
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
    return fs1,fs2,fs3,fs4


if __name__ == '__main__':
    print('hello')
    port1 = serial.Serial("/dev/tty.usbserial-W2100850", baudrate = 9600, timeout = 2)
    plt.ion() #开启interactive mode 成功的关键函数
    plt.figure(1)
    t = []
    t_now = 0
    ds=[ds1,ds2,ds3,ds4] = [[],[],[],[]]
    i=0
    # plt.clf()#清空画布上的所有内容
    for i in range(0,40):
        #print(readus(port1))
        s1,s2,s3,s4 = readus(port1)
        print("num=",i,",s1=",s1,",s2=",s2,",s3=",s3,",s4=",s4)
        t_now = i*0.5
        t.append(t_now)#模拟数据增量流入，保存历史数据
        ds1.append(s1)
        ds2.append(s2)
        ds3.append(s3)
        ds4.append(s4)
        # plt.figure(1)
        # for i in range(1,5):
        #     plt.subplot(2, 4, i)
        #     plt.title("fs"+str(i))
        #     plt.ylim(250,4500)
        #     plt.ylabel('(mm)')
        #     plt.xlabel('                time')
        #     plt.plot(t,ds[i-1],'-r')
        #plt.figure(2)
        # for i in range(5,9):
        #     plt.subplot(2, 4, i)
        #     plt.title("bs"+str(i-4))
        #     plt.ylim(250,4500)
        #     plt.ylabel('(mm)')
        #     plt.xlabel('time')
        #     plt.plot(t,ds[i-1],'-b')
        plt.pause(0.01)

    # plt.savefig('senor1.png')#儲存圖片
    # plt.show()




