from flight_control import FlightControl, FlightInfo
from visual import ArucoDetector
import math
import rclpy

class Mission:
    def __init__(self, controller:FlightControl, flight_info:FlightInfo) -> None:
        self.controller = controller
        self.flight_info = flight_info

    def landedOnPlatform(self):
        lowest_high = 0.3 # 最低可看到aruco的高度 單位:公尺
        speed = 0.1 # 速度 單位:公尺/秒
        aruco_detector = ArucoDetector()
        while True: #當無人機與平台的差距大於0.1公尺時
            rclpy.spin_once(self.flight_info.node)
            closest_aruco = aruco_detector.closestAruco()
            if closest_aruco is None:
                self.controller.setZeroVelocity()
                continue
            x,y,z,_,_,_ = closest_aruco.getCoordinate()
            if x is None or y is None or z is None:
                self.controller.setZeroVelocity()
                continue
            diffrent_distance = math.sqrt(x**2 + y**2)
            if(diffrent_distance < 0.1):
                break
            x_speed = x/diffrent_distance * speed #todo 需調整aruco和無人機飛行方向一致
            y_speed = y/diffrent_distance * speed
            if(self.flight_info.rangefinder_alt < lowest_high):
                self.controller.sendPositionTarget(y_speed, x_speed, -0.1)
            else:
                self.controller.sendPositionTarget(y_speed, x_speed, 0.0)
        self.controller.setZeroVelocity()
        is_land_success = False
        while(is_land_success is False):
            is_land_success = self.controller.land()
    
        
