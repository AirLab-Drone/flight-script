from mavrostest.flight_control import FlightControl
from mavrostest.visual import Visual
import math

class Mission:
    def __init__(self, controller:FlightControl) -> None:
        self.controller = controller

    def landedOnPlatform(self):
        lowest_high = 0.1 # 最低可看到aruco的高度 單位:公尺
        speed = 0.5
        visual = Visual()
        aruco = visual.getArucoPositionById(0)
        diffrent_distance = math.sqrt(aruco[0]**2 + aruco[1]**2)
        while diffrent_distance > 0.1: #當無人機與平台的差距大於0.1公尺時
            x_speed = aruco[0]/diffrent_distance * speed #todo 需調整aruco和無人機飛行方向一致
            y_speed = aruco[1]/diffrent_distance * speed
            self.controller.setVelocity(x_speed, y_speed, -0.1)
            aruco = visual.getArucoPositionById(0)
            diffrent_distance = math.sqrt(aruco[0]**2 + aruco[1]**2)
        self.controller.setZeroVelocity()
        self.controller.land()
    
        
