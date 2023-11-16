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

    def initTimer(self):
        timer_period = 0.1 # 1 cycle per 0.1 second
        self.node.create_timer(timer_period, self.sendPositionTarget)
        rclpy.spin(self.node)
    
    def sendPositionTarget(self):
        print(f'x: {self.msg.velocity.x}, y: {self.msg.velocity.y}, z: {self.msg.velocity.z}')
        self.publisher.publish(self.msg)

    def setInitPositionTarget(self):
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
    
    def setVelocity(self, x, y, z):
        '''
        @param x: x-axis velocity in m/s
        @param y: y-axis velocity in m/s
        @param z: z-axis velocity in m/s
        '''
        self.msg.velocity.x = float(x) * 2.0
        self.msg.velocity.y = float(y) * 2.0
        self.msg.velocity.z = float(z) * 2.0

    def setZeroVelocity(self):
        self.msg.velocity.x = 0.0
        self.msg.velocity.y = 0.0
        self.msg.velocity.z = 0.0

    def armAndTakeoff(self):
        print("armAndTakeoff")
        set_mode_client = self.node.create_client(SetMode, '/mavros/set_mode')
        arming_client = self.node.create_client(CommandBool, '/mavros/cmd/arming')
        takeoff_client = self.node.create_client(CommandTOL, '/mavros/cmd/takeoff')

        set_mode_request = SetMode.Request(base_mode=0, custom_mode='4')
        arming_request = CommandBool.Request(value=True)
        takeoff_request = CommandTOL.Request(altitude=2.0)

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
        future_land = land_client.call(land_request)
        # rclpy.spin_until_future_complete(self.node, future_land)
        print(future_land)

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()