import rclpy

class Visual:
    def __init__(self) -> None:
        self.node = rclpy.Node("visual")

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def getImage(self):
        pass

    def arucoDetect(self) :
        '''
        @return: list([id, x, y, z, yaw, pitch, roll]) or None, return a list of aruco marker detected, each element is a list of [id, x, y, z, yaw, pitch, roll]
        '''
        pass

    def getArucoPositionById(self, id):
        '''
        @param id: int, the id of the aruco marker
        @return: [x, y, z, yaw, pitch, roll] or None, return the position of the aruco marker, [x, y, z, yaw, pitch, roll] is a list of float
        '''
        pass