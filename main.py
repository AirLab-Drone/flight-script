import time
from dronekit import Vehicle, connect, VehicleMode, LocationLocal, LocationGlobal
import info

def location_info():
    vehicle = connect("/dev/tty.usbserial-D30GLXI0", wait_ready=True, baud=57600)
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    info.connectCheck(vehicle)
    start_time = time.time()
    while time.time() - start_time < 5:
        info.NEDLocation(vehicle)
        time.sleep(1)
def speed_info():
    vehicle = connect("/dev/tty.usbserial-D30GLXI0", wait_ready=True, baud=57600)
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    info.connectCheck(vehicle)
    start_time = time.time()

if __name__ == "__main__":
    vehicle = connect("/dev/tty.usbserial-D30GLXI0", wait_ready=False, baud=57600)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.parameters['EK2_ENABLE'] = 1
    vehicle.home_location = LocationGlobal(0, 0, 0)
    info.connectCheck(vehicle)
    # vehicle.arm()
    while True:
        print("location: " + str(vehicle.location.local_frame))
        time.sleep(0.5)