import math
import time
import rclpy
from visual import ArucoDetector
from flight_control import FlightControl, FlightInfo
from tool.PID import PID
import threading


class Mission:
    def __init__(self, controller: FlightControl, flight_info: FlightInfo) -> None:
        self.controller = controller
        self.flight_info = flight_info

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
        lowest_high = 1.2  # 最低可看到aruco的高度 單位:公尺
        max_distance = 0.2
        max_speed = 0.8  # 速度 單位:公尺/秒
        max_yaw = 0.174  # 10度
        downWard_distance = -0.2  #
        aruco_detector = ArucoDetector(video_source=1)
        count = 0
        max_count = 50
        pid_x = PID(
            4, 0, 3, 0, time=self.controller.node.get_clock().now().nanoseconds * 1e-9
        )
        pid_y = PID(
            4, 0, 3, 0, time=self.controller.node.get_clock().now().nanoseconds * 1e-9
        )
        while True:
            rclpy.spin_once(self.flight_info.node)
            closest_aruco = aruco_detector.closestAruco()
            if closest_aruco is None:
                # count += 1
                # if count > max_count:
                #     self.controller.sendPositionTargetPosition(0, 0, 0.2, 0)
                #     count = 0
                self.controller.setZeroVelocity()
                continue
            x, y, z, yaw, _, _ = closest_aruco.getCoordinate()
            if x is None or y is None or z is None or yaw is None:
                count += 1
                # if count > max_count:
                #     self.controller.sendPositionTargetPosition(0, 0, 0.2, 0)
                #     count = 0
                self.controller.setZeroVelocity()
                continue
            count = 0
            # -------------------------------- PID control ------------------------------- #
            # print(f'time:{self.controller.node.get_clock().now().nanoseconds}*1e-9')
            move_x = -pid_x.PID(
                x, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            move_y = -pid_y.PID(
                y, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            # print(f"move_pid_x:{move_pid_x}, move_pid_y:{move_pid_y}")

            diffrent_distance = math.sqrt(x**2 + y**2)

            # if diffrent_distance > 0.5:
            #     move_x = x/diffrent_distance*max_speed
            #     move_y = y/diffrent_distance*max_speed
            # else:
            #     move_x = x
            #     move_y = y
            move_x = min(max(x, -max_speed), max_speed)
            move_y = min(max(y, -max_speed), max_speed)
            move_yaw = min(max(yaw * 3.14159 / 180, -max_yaw), max_yaw)
            if (
                diffrent_distance < 0.05
                and self.flight_info.rangefinder_alt < lowest_high
            ):  # 當無人機與平台的差距大於0.1公尺時停止
                break
            # move_x = min(max(move_x, -max_speed), max_speed)
            # move_y = min(max(move_y, -max_speed), max_speed)
            if self.flight_info.rangefinder_alt > lowest_high:
                self.controller.sendPositionTargetVelocity(
                    -move_y,
                    -move_x,
                    downWard_distance,
                    -move_yaw,
                )
            else:
                # when height is lower than lowest_high, stop moving down
                self.controller.sendPositionTargetVelocity(
                    -move_y,
                    -move_x,
                    0,
                    -move_yaw,
                )
            print(f"move_x:{move_x}, move_y:{move_y}, move_yaw:{move_yaw}")
            time.sleep(1)
        self.controller.setZeroVelocity()
        aruco_detector.stop()
        print('now i want to land=================================')
        while not self.controller.land():
            print("landing")
