from flight_control import FlightControl
from visual import ArucoDetector
import math

class Mission:
    def __init__(self, controller:FlightControl) -> None:
        self.controller = controller

    def landedOnPlatform(self):
        lowest_high = 0.1 # 最低可看到aruco的高度 單位:公尺
        speed = 0.1 # 速度 單位:公尺/秒
        aruco_detector = ArucoDetector()
        while True: #當無人機與平台的差距大於0.1公尺時
            closest_aruco = aruco_detector.closestAruco()
            if closest_aruco == None:
                continue
            x,y,z,yaw,pitch,roll = closest_aruco.getCoordinate()
            if x==None or y==None:
                continue
            diffrent_distance = math.sqrt(x**2 + y**2)
            if(diffrent_distance < 0.1):
                break
            x_speed = x/diffrent_distance * speed #todo 需調整aruco和無人機飛行方向一致
            y_speed = y/diffrent_distance * speed
            self.controller.sendPositionTarget(y_speed, x_speed, -0.1)
        self.controller.setZeroVelocity()
        isLandSuccess = False
        while(isLandSuccess == False):
            isLandSuccess = self.controller.land()
    
        
