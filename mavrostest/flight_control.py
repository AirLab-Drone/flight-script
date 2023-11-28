import threading
import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

class FlightControl:
    def __init__(self):
        self.node = rclpy.create_node('flight_control')
        self.publisher = self.node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.msg = self.setInitPositionTarget()

    # def startTimer(self):
    #     self.timer_tread = threading.Thread(target=self.newTimer)
    #     self.timer_tread.start()
    # def newTimer(self):
    #     timer_period = 0.1 # 1 cycle per 0.1 second
    #     self.timer = self.node.create_timer(timer_period, self.sendPositionTarget)
    #     try:
    #         rclpy.spin(self.node)
    #     except KeyboardInterrupt:
    #         pass
    
    def simpleFlight(self,x,y,z,duration):
        '''
        @param x: x-axis velocity in m/s
        @param y: y-axis velocity in m/s
        @param z: z-axis velocity in m/s
        @param duration: duration in second
        使無人機飛行x,y,z軸的速度，單位為m/s，持續duration秒。
        '''
        start_time = time.time()
        while(time.time() - start_time < duration):
            self.sendPositionTarget(x,y,z)
            time.sleep(0.1)
        self.sendPositionTarget(0,0,0)
    def sendPositionTarget(self, x, y, z):
        '''
        @param x: x-axis velocity in m/s
        @param y: y-axis velocity in m/s
        @param z: z-axis velocity in m/s
        傳送x,y,z軸的速度給飛控，單位為m/s。
        '''
        self.msg.velocity.x = float(x)
        self.msg.velocity.y = float(y)
        self.msg.velocity.z = float(z)
        print(f'x: {self.msg.velocity.x}, y: {self.msg.velocity.y}, z: {self.msg.velocity.z}')
        self.publisher.publish(self.msg)
    def setInitPositionTarget(self):
        '''
        @return: mavros_msgs.msg.PositionTarget
        設定PositionTarget的初始值，定義飛行加速度與起始速度0。
        '''
        msg_obj = PositionTarget()
        msg_obj.coordinate_frame = 8
        msg_obj.type_mask = 128
        msg_obj.position.x = 0.0
        msg_obj.position.y = 0.0
        msg_obj.position.z = 0.0
        msg_obj.velocity.x = 0.0
        msg_obj.velocity.y = 0.0
        msg_obj.velocity.z = 0.0
        msg_obj.acceleration_or_force.x = 0.5
        msg_obj.acceleration_or_force.y = 0.5
        msg_obj.acceleration_or_force.z = 0.5
        msg_obj.yaw = 0.0
        msg_obj.yaw_rate = 0.0
        return msg_obj
    
    def setZeroVelocity(self):
        '''使無人機速度歸零，懸停於空中。'''
        self.sendPositionTarget(0,0,0)

    def armAndTakeoff(self, alt=1.2):
        '''
        @param alt: altitude in meter
        啟動馬達並起飛。
        '''
        print("armAndTakeoff")
        set_mode_client = self.node.create_client(SetMode, '/mavros/set_mode')
        arming_client = self.node.create_client(CommandBool, '/mavros/cmd/arming')
        takeoff_client = self.node.create_client(CommandTOL, '/mavros/cmd/takeoff')

        set_mode_request = SetMode.Request(base_mode=0, custom_mode='4')
        arming_request = CommandBool.Request(value=True)
        takeoff_request = CommandTOL.Request(altitude=alt)

        future_set_mode = set_mode_client.call_async(set_mode_request)
        print("future_set_mode")
        time.sleep(5)
        future_arming = arming_client.call_async(arming_request)
        print("future_arming")
        time.sleep(5)
        future_takeoff = takeoff_client.call_async(takeoff_request)
        print("future_takeoff")
        time.sleep(6)

        rclpy.spin_until_future_complete(self.node, future_set_mode)
        print(future_set_mode.result())
        if(future_set_mode.result() == None):
            return False
        if(future_set_mode.result().mode_sent == False):
            return False

        rclpy.spin_until_future_complete(self.node, future_arming)
        print(future_arming.result())
        if(future_arming.result() == None):
            return False
        if(future_arming.result().success == False):
            return False

        rclpy.spin_until_future_complete(self.node, future_takeoff)
        print(future_takeoff.result())
        if(future_takeoff.result() == None):
            return False
        if(future_takeoff.result().success == False):
            return False
        return True

    def land(self):
        land_client = self.node.create_client(CommandTOL, '/mavros/cmd/land')
        land_request = CommandTOL.Request()
        future_land = land_client.call_async(land_request)
        rclpy.spin_until_future_complete(self.node, future_land)
        if(future_land.result() == None):
            return False
        if(future_land.result().success == False):
            return False
        return True

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    controler = FlightControl()
    print(controler.land())
    controler.destroy()