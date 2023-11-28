# Python
import threading
import time
import rclpy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList, PositionTarget
import flight_control
from mission import Mission

def wait(controller):
    controller.setZeroVelocity()
    time.sleep(5)

def velocityTest():
    rclpy.init()
    flightController = flight_control.FlightControl()
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
    flightController = flight_control.FlightControl()
    time.sleep(5)
    isTakeoffSuccess = False
    while(isTakeoffSuccess == False):
        isTakeoffSuccess = flightController.armAndTakeoff()
    mission = Mission(flightController)
    mission.landedOnPlatform()
    flightController.destroy()
if __name__ == '__main__':
    arucoLandingTest()