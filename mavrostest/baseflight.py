# Python
import time
import rclpy
import flight_control
from mission import Mission

def wait(controller):
    controller.setZeroVelocity()
    time.sleep(5)

def velocityTest():
    rclpy.init()
    node = rclpy.create_node("flight_control")
    flightController = flight_control.FlightControl(node)
    time.sleep(5)
    isTakeoffSuccess = False
    while(isTakeoffSuccess == False):
        isTakeoffSuccess = flightController.armAndTakeoff()
    # flightController.simpleFlight(1.0, 0.0, 0.0, 5)
    # wait(flightController)
    flightController.simpleFlight(-1.0, 0.0, 0.0, 5)
    wait(flightController)
    # flightController.simpleFlight(0.0, 0.5, 0.0, 5)
    # wait(flightController)
    # flightController.simpleFlight(0.0, -0.5, 0.0, 5)
    # wait(flightController)
    isLandSuccess = False
    while(isLandSuccess == False):
        isLandSuccess = flightController.land()
    flightController.destroy()
def arucoLandingTest():
    rclpy.init()
    node = rclpy.create_node("flight_control")
    flightController = flight_control.FlightControl(node)
    time.sleep(5)
    mission = Mission(flightController)
    mission.landedOnPlatform()
    flightController.destroy()
if __name__ == '__main__':
    arucoLandingTest()