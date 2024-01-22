import math
import time
import rclpy
from visual import ArucoDetector
from flight_control import FlightControl, FlightInfo
import json


class Mission:
    def __init__(self, controller: FlightControl, flight_info: FlightInfo) -> None:
        self.controller = controller
        self.flight_info = flight_info

    # def landedOnPlatform(self):
    #     lowest_high = 0.3  # 最低可看到aruco的高度 單位:公尺
    #     max_speed = 0.1  # 速度 單位:公尺/秒
    #     downWard_speed = -0.1  # 下降速度 單位:公尺/秒
    #     aruco_detector = ArucoDetector()
    #     while True:
    #         rclpy.spin_once(self.flight_info.node)
    #         closest_aruco = aruco_detector.closestAruco()
    #         if closest_aruco is None:
    #             self.controller.setZeroVelocity()
    #             continue
    #         x, y, z, _, _, _ = closest_aruco.getCoordinate()
    #         if x is None or y is None or z is None:
    #             self.controller.setZeroVelocity()
    #             continue
    #         diffrent_distance = math.sqrt(x**2 + y**2)
    #         if diffrent_distance < 0.1:  # 當無人機與平台的差距大於0.1公尺時停止
    #             break
    #         x_speed = x / diffrent_distance * max_speed  # todo 需調整aruco和無人機飛行方向一致
    #         y_speed = y / diffrent_distance * max_speed
    #         if self.flight_info.rangefinder_alt < lowest_high:
    #             self.controller.sendPositionTargetVelocity(y_speed, x_speed, downWard_speed)
    #         else:
    #             # when high is lower than lowest_high, stop moving down
    #             self.controller.sendPositionTargetVelocity(y_speed, x_speed, 0.0)
    #     self.controller.setZeroVelocity()
    #     is_land_success = False
    #     while is_land_success is False:
    #         is_land_success = self.controller.land()

    def landedOnPlatform(self):
        """
        Function to control the drone to land on a platform using Aruco markers.

        This function continuously checks the closest Aruco marker detected by the ArucoDetector.
        It calculates the distance and orientation between the drone and the marker.
        The drone moves towards the marker until the distance is less than 0.1 meters.
        If the drone's altitude is lower than the specified lowest_high value, it moves down towards the platform.
        Once the drone has landed on the platform, it stops moving and lands completely.

        Returns:
            None
        """
        lowest_high = 0.3  # 最低可看到aruco的高度 單位:公尺
        max_distance = 0.2
        max_yaw = 0.174  # 10度
        downward_distance = -0.2  #
        with open("aruco_setup.json", "r", encoding='utf-8') as f:
            aruco_setup = json.load(f)
        aruco_detector = ArucoDetector()
        while True:
            rclpy.spin_once(self.flight_info.node)
            closest_aruco = aruco_detector.closestAruco()
            if closest_aruco is None:
                self.controller.setZeroVelocity()
                continue
            id = closest_aruco.id
            x, y, z, yaw, _, _ = closest_aruco.getCoordinate()
            print(f"x:{x}, y:{y}, z:{z}, yaw:{yaw}")
            if x is None or y is None or z is None or yaw is None:
                self.controller.setZeroVelocity()
                continue
            x_offset = aruco_setup[str(id)]["coordinate"]["x"]
            y_offset = aruco_setup[str(id)]["coordinate"]["y"]
            diffrent_distance = math.sqrt(x**2 + y**2)
            move_x = min(max(x-x_offset, -max_distance), max_distance)
            move_y = min(max(y-y_offset, -max_distance), max_distance)
            move_yaw = min(max(yaw*3.14159/180, -max_yaw), max_yaw)
            if diffrent_distance < 0.1 and self.flight_info.rangefinder_alt<0.5:  # 當無人機與平台的差距大於0.1公尺時停止
                break
            if self.flight_info.rangefinder_alt > lowest_high:
                self.controller.sendPositionTargetPosition(
                    move_y,
                    move_x,
                    downward_distance,
                    -move_yaw,
                )
            else:
                # when high is lower than lowest_high, stop moving down
                self.controller.sendPositionTargetPosition(
                    move_y,
                    move_x,
                    0,
                    -move_yaw,
                )
            time.sleep(2)
        self.controller.setZeroVelocity()
        aruco_detector.stop()
        while not self.controller.land():
            print("landing")
