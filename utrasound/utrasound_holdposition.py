import time
import serial
import ultrasound_sample as UtraDistance
import threading


class Distance:
    value = 0.0


def fakeData():
    s1 = 0
    s2 = 0
    s3 = 0
    s4 = 0
    return s1, s2, s3, s4


def getDataLoop(port, output, show_data=False):
    # ds = [ds1, ds2, ds3, ds4] = [[], [], [], []]
    while True:
        s1, s2, s3, s4 = fakeData()
        output[0] = s1
        output[1] = s2
        output[2] = s3
        output[3] = s4
        if show_data:
            print(",s1=", s1, ",s2=", s2, ",s3=", s3, ",s4=", s4)


def hold(ds:list):
    '''
    @param ds 四個感測器的距離數值，單位mm
    @output x,y軸的移動速度
    此函數會回傳平面兩軸的移動速度，調整無人機保持懸停位置。
    '''
    ds_init = ds.copy()
    ds_diff = [0,0,0,0]
    while True:
        ds_diff = [ds[0]-ds_init[0], ds[1]-ds_init[1], ds[2]-ds_init[2], ds[3]-ds_init[3]]



if __name__ == "__main__":
    # port = serial.Serial("/dev/tty.usbserial-W2100850", baudrate=9600, timeout=2)
    port = serial.Serial()
    ds = [ds1, ds2, ds3, ds4] = [[], [], [], []]
    thread = threading.Thread(
        target=getDataLoop,
        args=(
            port,
            ds,
        ),
    )
    thread.start()
    while True:
        print(ds)
        time.sleep(0.5)
