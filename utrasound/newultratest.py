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
        #print(checksum)
        if checksum == in_byte1[5]:#package not loss
            s = sensor(in_byte1[3], in_byte1[4])
            if s == 0:s=4500
        else:#package loss
            s = 9999
    else:
        s = 9999
    return s