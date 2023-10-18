import time
from dronekit import Vehicle, connect, VehicleMode, LocationLocal, LocationGlobal
import info
import base_flight

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
    vehicle = connect("127.0.0.1:14550", wait_ready=False, baud=57600)
    vehicle.mode = VehicleMode("GUIDED")
    # vehicle.parameters['EK2_ENABLE'] = 1
    # vehicle.home_location = LocationGlobal(0, 0, 0)
    info.connectCheck(vehicle)
    high = input("take off high: ")
    vehicle.arm()
    vehicle.simple_takeoff(high)
    while True:
        dir = input("Enter direction: ")
        if dir == "w":
            base_flight.send_global_velocity(vehicle, 0, 1, 0, 1)
        elif dir == "s":
            base_flight.send_global_velocity(vehicle, 0, -1, 0, 1)
        elif dir == "a":
            base_flight.send_global_velocity(vehicle, -1, 0, 0, 1)
        elif dir == "d":
            base_flight.send_global_velocity(vehicle, 1, 0, 0, 1)
        elif dir == "q":
            break
    vehicle.mode = VehicleMode("LAND")
