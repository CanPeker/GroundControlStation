import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import threading
import cv2
import PIL
from PIL import Image as Img
from PIL import ImageTk
import PIL
import cv2
import numpy as np
from threading import Thread
import imutils
import math
import threading
import serial
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import base64
from pymavlink import mavutil

#veri yollayan sleep veri alan sleep'siz

class Drone:

    def __init__(self,connection_string):


        self.ip  = '127.0.0.1'

        self.connection_string = connection_string
        self.vehicle = connect(self.connection_string, wait_ready=True, baud=57600)
        self.mode = "Nan"
        self.h = 0
        self.headingDegree = 0
        self.grndSpeed=0
        self.airSpeed=0
        self.armDisarm=False
        #self.vehicle.parameters['WP_LOITER_RAD'] = 100
        self.lat=0
        self.lon=0
        self.battery = 0
        self.ekfStatus = True
        self.gps_status = ""


        self.roll = 980
        self.pitch = 980
        self.thrt = 980
        self.yaw = 1500
        self.arm = 980
        self.modestc = 980
        self.yawA=0
        self.yawE=0
        self.pitchAngle = 0
        self.rollAngle = 0

        self.OnOff=True
        #print("saDrone")

        #print("saDrone")

        self.connection = True



    def information(self):
        while True:
            self.mode = self.vehicle.mode.name
            self.h = round(self.vehicle.location.global_relative_frame.alt,1)
            self.headingDegree = self.vehicle.heading
            self.grndSpeed=round(self.vehicle.groundspeed,1)
            self.airSpeed = round(self.vehicle.airspeed,1)
            self.armDisarm = self.vehicle.armed
            self.lat = float(self.vehicle.location.global_frame.lat)
            self.lon = float(self.vehicle.location.global_frame.lon)
            self.battery = self.vehicle.battery.voltage
            self.ekfStatus = self.vehicle.ekf_ok
            self.gps_status = self.vehicle.gps_0.satellites_visible
            self.pitchAngle = self.vehicle.attitude.pitch
            self.rollAngle = self.vehicle.attitude.roll
            #print("information:",self.battery,"stats:",self.gps_status)
            time.sleep(0.3)

    def KmdClientVeriAl(self):

        self.KumandaClientVeriAl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.KumandaClientVeriAl.connect((self.ip, 12349))


        while True:

            try:
                data = self.KumandaClientVeriAl.recv(4096).decode('utf-8')
                dataSplit = data.split("/")
                #print(dataSplit)

                self.OnOfftx=str(dataSplit[0])
                self.roll = float(dataSplit[1])
                self.pitch = float(dataSplit[2])
                self.thrt = float(dataSplit[3])
                self.arm = float(dataSplit[5])
                self.modestc = float(dataSplit[6])
                self.yawA = float(dataSplit[7])
                self.yawE = float(dataSplit[8].strip())

                print("thrst=", self.thrt, "pitch:", self.pitch, "roll:", self.roll, "yawA:", self.yawA, "yawE:",
                          self.yawE,
                         "arm:", self.arm, "mode:", self.modestc)

                if(self.OnOfftx=="False"):
                    print("tx lost")
                    self.OnOff=False
                if (self.OnOfftx == "True"):
                    self.OnOff = True

            except socket.error:
                print("HATA !! KumandaVeriAl")
            print("e")





    def runDrone(self):

        while True:

            while self.OnOff:

                self.vehicle.channels.overrides['4'] = 1500

                self.vehicle.channels.overrides['1'] = 1500+int(self.roll)

                self.vehicle.channels.overrides['2'] = 1500+int(self.pitch)
                self.vehicle.channels.overrides['3'] = int(self.thrt)

                if(int(self.yawA)==10):
                    self.vehicle.channels.overrides['4'] = 1550
                if (int(self.yawE) == 10):
                    self.vehicle.channels.overrides['4'] = 1450

                self.vehicle.channels.overrides['5'] = int(self.modestc)
                self.vehicle.channels.overrides['6'] = int(self.arm)
                time.sleep(0.1)

            time.sleep(0.1)
            print("tx disabled")



class Telemetri:

    def __init__(self):

        self.ip = '127.0.0.1'
        self.kumandaIP = '127.0.0.1'

        self.connectionString = "tcp:127.0.0.1:5763"

        self.drone = Drone(self.connectionString)

        self.drone.ip = self.kumandaIP

        self.tDrone = threading.Thread(target=self.drone.information)


        self.connection2=True

        self.tDrone.start()

        t_kmd = threading.Thread(target=self.drone.KmdClientVeriAl)
        t_kmd.start()

        t_run = threading.Thread(target=self.drone.runDrone)
        t_run.start()

        self.TelemClientVeriAl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TelemClientVeriAl.connect((self.ip, 12346))

        self.TelemClientVeriYolla = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TelemClientVeriYolla.connect((self.ip, 12345))



    def TlmClientVeriAl(self):

        while True:

            try:

                self.data = self.TelemClientVeriAl.recv(4096).decode('utf-8')
                self.dataStrip = self.data.split("/")
                if(self.dataStrip[0]=="Mode"):
                    if(str(self.dataStrip[1])=="GUIDED" or str(self.dataStrip[1])=="STABILIZE" or str(self.dataStrip[1])=="AUTO" or
                               str(self.dataStrip[1])=="LAND"):
                        self.drone.vehicle.mode=str(self.dataStrip[1])
                        print("Mode Changed")
                    else:
                        print("modeEror")

                if (self.dataStrip[0] == "ArmDisarm"):

                    if(str(self.dataStrip[1])=="False"):
                        self.drone.vehicle.armed=False
                    else:
                        self.drone.vehicle.armed = True

                if(self.dataStrip[0] == "Radius"):
                    self.drone.vehicle.parameters['WP_LOITER_RAD']=int(self.dataStrip[1])

            except socket.error:
                print("HATA !! TlmClientVeriAl")





    def TlmClientVeriYolla(self):

        while True:
            try:
                msg = str(self.drone.mode) + "/" + str(self.drone.headingDegree) + "/" + str(self.drone.h) + "/" + str(
                    self.drone.grndSpeed) + "/" + str(self.drone.airSpeed) + "/" + str(self.drone.armDisarm) + "/" + str(self.drone.lat) + "/" + str(self.drone.lon) + "/" + str(
                     self.drone.battery)+ "/"+ str(self.drone.gps_status) + "/"
                #print(msg)
                self.TelemClientVeriYolla.send(msg.encode('utf-8'))
                time.sleep(0.3)
            except socket.error:
                #print("HATA !! TlmClientVeriYolla")
                pass



class Video:

    def __init__(self):

        self.ip='127.0.0.1'

        self.Vclient_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.Vclient_socket.connect((self.ip, 5000))

        self.cameraIndex=0



    def VideoAktar(self):
        connection = self.Vclient_socket.makefile('wb')

        cam = cv2.VideoCapture(self.cameraIndex)

        cam.set(3, 1080)
        cam.set(4, 720)

        # addr = "127.0.0.1:12345"

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        img_counter = 0

        while True:


            ret, frame = cam.read()
            frame = cv2.resize(frame,(1080,720))
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            #    data = zlib.compress(pickle.dumps(frame, 0))
            data = pickle.dumps(frame, 0)
            size = len(data)

            # print("{}: {}".format(img_counter, size))
            self.Vclient_socket.sendall(struct.pack(">L", size) + data)
            # client_socket.send("sa".encode('utf-8'))
            img_counter += 1





class Task:

    def __init__(self):

        self.ip='127.0.0.1'

        self.TaskClientVeriAl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TaskClientVeriAl.connect((self.ip,12348))

        self.TaskClientVeriYolla = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TaskClientVeriYolla.connect((self.ip, 12347))

        self.TaskMsg = "Nan"

    def TskClientVeriAl(self):

        #print("sa1")

        while True:
            #print("(Client)Data From Task Server: ",self.TaskClientVeriAl.recv(64).decode('utf-8'))
            time.sleep(0.3)

    def TskClientVeriYolla(self):

        #print("sa2")


        while True:

            self.TaskClientVeriYolla.send(self.TaskMsg.encode('utf-8'))
            time.sleep(0.3)




telemetri = Telemetri()
vid = Video()
#task = Task()


vid.cameraIndex=0
telemetri.ip='127.0.0.1'
telemetri.kumandaIP = '127.0.0.1'
telemetri.connectionString="tcp:127.0.0.1:5763"
vid.ip='127.0.0.1'
#task.ip='127.0.0.1'



tVer = threading.Thread(target=telemetri.TlmClientVeriAl)
tAl = threading.Thread(target=telemetri.TlmClientVeriYolla)
tvid = threading.Thread(target=vid.VideoAktar)


#tTaskAl = threading.Thread(target=task.TskClientVeriAl)
#tTaskVer = threading.Thread(target=task.TskClientVeriYolla)




tVer.start()
tAl.start()
tvid.start()

#tTaskAl.start()
#tTaskVer.start()



























