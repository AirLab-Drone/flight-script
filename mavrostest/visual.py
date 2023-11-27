import cv2
import datetime 
import numpy as np
import json
import math
import os
import threading

class LimitedList:
    def __init__(self, initial_size=20):
        self.items = []
        self.initial_size = initial_size

    def add_element(self, element):
        if len(self.items) == self.initial_size:
            self.items.pop(0)  # 删除最早的元素
        self.items.append(element)
    def pop_element_and_getmedian(self):
        if len(self.items) == 0:
            return 'null'
        else:
            self.items.pop(0)
            return np.median(self.items)
    def calculate_median(self):
        return np.median(self.items)
    def calculate_std(self):
        return np.std(self.items)
    
class Aruco:
    mtx = np.array([[776.31000562, 0, 327.96638128], [0, 775.07173891, 179.57551958], [0, 0, 1]])
    dist = np.array([[8.29378271e-02, 1.26989092e-01, 3.86532147e-03, 1.18462078e-03, -1.87627090e+00]])
    markerLength = 0.0585 # unit: meter
    limit_list_size = 5
    # markerLength = 0.0385
    def __init__(self,id) -> None:
        self.id = id
        self.x_list = LimitedList(self.limit_list_size)
        self.y_list = LimitedList(self.limit_list_size)
        self.z_list = LimitedList(self.limit_list_size)
        self.yaw_list = LimitedList(self.limit_list_size)
        self.pitch_list = LimitedList(self.limit_list_size)
        self.roll_list = LimitedList(self.limit_list_size)
    def checkInList(self,ids):
        if ids is None or self.id not in ids:
            self.x_list.pop_element_and_getmedian()
            self.y_list.pop_element_and_getmedian()
            self.z_list.pop_element_and_getmedian()
            self.yaw_list.pop_element_and_getmedian()
            self.pitch_list.pop_element_and_getmedian()
            self.roll_list.pop_element_and_getmedian()
            return False
        else:
            return True
            
    def estimatePoseSingleMarkers(self,corner):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corner, self.markerLength, self.mtx, self.dist)
        self.rvec = rvec
        self.tvec = tvec
        yaw, pitch, roll = self.rvec_to_euler_angles(rvec)
        x = tvec[0][0][0]
        y = tvec[0][0][1]
        z = tvec[0][0][2]
        return x,y,z,yaw,pitch,roll
    def update(self,id,corner):
        if(self.id != id):
            return
        x,y,z,yaw,pitch,roll = self.estimatePoseSingleMarkers(corner)
        self.x_list.add_element(x)
        self.y_list.add_element(y)
        self.z_list.add_element(z)
        self.yaw_list.add_element(math.degrees(yaw))
        self.pitch_list.add_element(math.degrees(pitch))
        self.roll_list.add_element(math.degrees(roll))
        return self.getCoordinate()
    def getCoordinate(self):
        if(self.checkStd() == False):
            return None,None,None,None,None,None
        x = self.x_list.calculate_median()
        y = self.y_list.calculate_median()
        z = self.z_list.calculate_median()
        yaw = self.yaw_list.calculate_median()
        pitch = self.pitch_list.calculate_median()
        roll = self.roll_list.calculate_median()
        return x,y,z,yaw,pitch,roll
    def checkStd(self):
        x_std = self.x_list.calculate_std()
        y_std = self.y_list.calculate_std()
        z_std = self.z_list.calculate_std()
        yaw_std = self.yaw_list.calculate_std()
        if x_std > 0.02 or y_std > 0.02 or z_std > 0.02 or yaw_std > 2:
            return False
        return True
    def rvec_to_euler_angles(self,rvec):
        rvec_flipped = rvec[0][0]*-1 
        R_mat, jacobian=cv2.Rodrigues(rvec_flipped)        
        pitch = math.atan2(R_mat[2,1],R_mat[2,2])
        sy = math.sqrt((R_mat[0][0]*R_mat[0][0])+ (R_mat[1][0]*R_mat[1][0]))
        singular = sy<1e-6
        if not singular:
            pitch = math.atan2(R_mat[2,1],R_mat[2,2])
            roll = math.atan2(-R_mat[2,0],sy)
            yaw = math.atan2(R_mat[1,0],R_mat[0,0])
        else:
            pitch = math.atan2(-R_mat[1,2],R_mat[1,1])
            roll = math.atan2(-R_mat[2,0],sy)
            yaw = 0
        return yaw, pitch, roll
    def drawAruco(self, frame):
        frame = cv2.drawFrameAxes(frame, self.mtx,self.dist,self.rvec,self.tvec, 0.05)
        return frame

class ArucoDetector:
    # #-----------------setting-----------------
    # OUTVEDIO = False
    # DRAWTEXT = True
    # SHOWIMAGE = False
    # DRAWARUCO = True
    is_running = True
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.arucoList = []
        self.count = 0
        self.start_time = cv2.getTickCount()
        #-----------------output vedio-----------------
        # if(self.OUTVEDIO):
        #     outputvediofolder = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        #     fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        #     if(not os.path.exists('output_vedio/')):
        #         os.makedirs('output_vedio/')
        #     self.out = cv2.VideoWriter('output_vedio/'+outputvediofolder+'.mp4', fourcc, 20.0, (640,  480))
        # ------------------------------- start detect ------------------------------- #
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        rec, self.frame = self.cap.read()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
    
    def run(self):
        while self.is_running:
            ret, self.frame = self.cap.read()
            #-----------------find aruco-----------------
            if self.frame is None:
                continue
            (corners, ids, rejected) = self.detector.detectMarkers(self.frame)
            self.frame = cv2.aruco.drawDetectedMarkers(self.frame, corners, ids, (0,255,0))

            if len(corners) > 0:
                for i in range(len(ids)):
                    self.addNewAruco(ids[i][0],corners[i])
            for aruco in self.arucoList:
                if aruco.checkInList(ids) and len(corners) > 0:
                    id = aruco.id
                    aruco.update(id,corners[np.where(ids == aruco.id)[0][0]])
            self.debug()
    def stop(self):
        self.is_running = False
        self.cap.release()
        cv2.destroyAllWindows()
    def debug(self):
        #-------------print fps-----------------
        self.count += 1
        if(self.count%30 == 0):
            end_time = cv2.getTickCount()
            print('FPS: ', 30/((end_time - self.start_time)/cv2.getTickFrequency()))
            # for aruco in arucoList:
            #     print(aruco.id,aruco.getCoordinate())
            self.start_time = cv2.getTickCount()

    def closestAruco(self):
        send_data = []
        closest_aruco = None
        for aruco in self.arucoList:
            id = aruco.id
            x,y,z,yaw,pitch,roll = aruco.getCoordinate()
            if(x == None or y == None or z == None or yaw == None or pitch == None or roll == None):
                continue
            if(math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(yaw) or math.isnan(pitch) or math.isnan(roll)):
                continue
            json_data = {
                'id':int(id),
                'x':format(x,'.2f'),
                'y':format(y,'.2f'),
                'z':format(z,'.2f'),
                'yaw':format(yaw,'.2f'),
                'pitch':format(pitch,'.2f'),
                'roll':format(roll,'.2f')
            }
            if(closest_aruco == None):
                closest_aruco = json_data
            else:
                aruco_distance = math.sqrt(math.pow(x,2)+math.pow(y,2))
                closest_aruco_distance = math.sqrt(math.pow(float(closest_aruco['x']),2)+math.pow(float(closest_aruco['y']),2))
                if(aruco_distance < closest_aruco_distance):
                    closest_aruco = json_data
            send_data.append(json_data)
        if(closest_aruco != None):
            send_data.append(closest_aruco)
        if(len(send_data) == 0):
            send_data_json = json.dumps(send_data)
        else:
            send_data_json = json.dumps([send_data[0]])
        # print(send_data_json)

    def addNewAruco(self, id, corner):
        for aruco in self.arucoList:
            if aruco.id == id:
                return False
        aruco = Aruco(id)
        aruco.update(id,corner)
        self.arucoList.append(aruco)
    def draw(self, x, y, z, yaw, id, frame, image_y):
        text = 'id: {}, x: {:.2f}, y: {:.2f}, z: {:.2f}, yaw: {:.2f}'.format(id, x, y, z, yaw)
        cv2.putText(frame, text, (20,image_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    while True:
        if aruco_detector.frame is None:
            continue
        cv2.imshow('frame', aruco_detector.frame)
        if cv2.waitKey(1) == ord('q'):
            aruco_detector.stop()
            break