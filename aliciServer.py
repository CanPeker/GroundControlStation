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
from tkinter import *
import base64
import pygame

class Task:

    def __init__(self,ip='127.0.0.1'):

        self.ip=ip



        self.TaskServerVeriAl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TaskServerVeriAl.bind((self.ip, 12347))
        self.TaskServerVeriAl.listen(4)

        self.TaskServerVeriYolla = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TaskServerVeriYolla.bind((self.ip, 12348))
        self.TaskServerVeriYolla.listen(4)



        self.TaskMsg = "Nan"

    def TskServerVeriAl(self):

        #print("TskServerVeriAl")

        conn, adrr = self.TaskServerVeriAl.accept()

        while True:

            #print("(Server)Data From Client Task: ",conn.recv(64).decode('utf-8'))
            time.sleep(0.3)

    def TskServerVeriYolla(self):

        #print("TskServerVeriYolla")

        conn,adrr = self.TaskServerVeriYolla.accept()

        while True:

            conn.send(self.TaskMsg.encode('utf-8'))
            time.sleep(0.3)


class Video:

    def __init__(self,ip='127.0.0.1'):
        self.frame = cv2.imread("load.jpeg")
        self.ip = ip
        self.OnOff=False
        self.conOnOff=False





    def grntAl(self):

        HOST = self.ip
        PORT = 5000
        PORT2 = 54321
        DISCONNECT_MESSAGE = "!DISCONNECT"

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('Socket created')

        s.bind((HOST, PORT))
        print('Socket bind complete')
        s.listen(10)
        print('Socket now listening')

        conn, addr = s.accept()

        self.conOnOff=True


        ss = 0

        data = b""
        payload_size = struct.calcsize(">L")
        # print("payload_size: {}".format(payload_size))


        while True:

            try:
                ss = ss + 1

                while len(data) < payload_size:
                    # print("Recv: {}".format(len(data)))
                    data += conn.recv(4096)

                # print("Done Recv: {}".format(len(data)))
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack(">L", packed_msg_size)[0]
                # print("msg_size: {}".format(msg_size))

                while len(data) < msg_size:
                    data += conn.recv(4096)

                frame_data = data[:msg_size]
                data = data[msg_size:]

                frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                self.frame = frame
            except socket.error:
                print("serverVideoConnection trying to connect...")
                self.conOnOff=False
                conn, addr = s.accept()
                self.conOnOff=True
                print("serverVideoConnection reconnect Ok")



        #try to connect




class Telemetri:

    def __init__(self,ip='127.0.0.1'):

        self.ipp = ip


        self.ip = str(self.ipp)

        self.port = str(12345)

        self.TelemServerVeriAl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TelemServerVeriAl.bind((self.ipp, 12345))
        self.TelemServerVeriAl.listen(4)

        self.TelemServerVeriYolla = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TelemServerVeriYolla.bind((self.ipp, 12346))
        self.TelemServerVeriYolla.listen(4)

        self.TelemMsg = "Nan/Nan"

        self.x = 0

        self.Cmode = "UNKNOWN"
        self.Ch = 0
        self.Chead = 0
        self.Cgrndspd = 0
        self.Cairspd = 0
        self.armedd = "Disarm"
        self.lat = 0
        self.lon = 0

        self.battery = 0
        self.gpsStatus = 0

        self.connection=True

        self.pitchAngle = 0
        self.rollAngle = 0

        self.OnOff = False

        self.conOnOff1=False
        self.conOnOff2=False


    def TlmServerVeriAl(self):

        print("TelemServerVeriAl")
        conn, adr = self.TelemServerVeriAl.accept()
        self.conOnOff1=True
        self.x = self.x + 1

        while True:
            try:
                x1=time.time()
                self.msg2 = conn.recv(4096).decode('utf-8')
                x2=time.time()
                #print("fpsData",1/(x2-x1))
                strr = self.msg2.split("/")
                #print("Telemetri Data:", strr, "length of data:", len(strr))
                self.Cmode = strr[0]
                self.Chead = strr[1]
                self.Ch = strr[2]
                self.Cgrndspd = strr[3]
                self.Cairspd = strr[4]
                self.armedd = strr[5]
                self.lat = float(strr[6])
                self.lon = float(strr[7])
                self.battery = strr[8]
                self.gpsStatus = strr[9].strip()

            except socket.error:
                print(" TelemServerVeriAl Trying To Connect..")
                self.conOnOff1=False
                conn, adr = self.TelemServerVeriAl.accept()
                print("TlmServerVeriAl Trying reconnect ok")
                self.conOnOff1=True



    def TlmServerVeriYolla(self):
        print("TelemServerVeriYolla")
        conn, adr = self.TelemServerVeriYolla.accept()
        self.conOnOff2=True
        while True:


            try:
                #print(self.TelemMsg)
                conn.send(self.TelemMsg.encode('utf-8'))
            except socket.error:
                self.conOnOff2=False
                print("HATA !! TlmServerVeriYolla Trying to connect... ")
                conn, adr = self.TelemServerVeriYolla.accept()
                self.conOnOff2=True
                print("TlmServerVeriYolla Trying reconnect ok")
            time.sleep(0.3)



class Kumanda:

    def __init__(self,ip='127.0.0.1'):


        self.ip = ip

        self.KumandaServerVeriYolla = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.KumandaServerVeriYolla.bind((self.ip, 12349))
        self.KumandaServerVeriYolla.listen(4)

        self.x = 0
        self.y = 0
        self.z = 0
        self.k = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrt = 0

        self.arm = 980
        self.stc9 = 980
        self.stc11 = 980
        self.stc13 = 980

        self.modestc = 980

        self.yawA=0
        self.yawE=0

        self.zoomCons=1
        self.relay = 1500
        self.r=1

        self.OnOff=False

        self.conOk=False


        pygame.display.init()
        pygame.joystick.init()
        pygame.joystick.Joystick(1).init()


    def mapp(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



    def run(self):

        i=0

        while True:
            #print("in")
            pygame.event.pump()

            self.roll = round(self.mapp(pygame.joystick.Joystick(1).get_axis(0), -1, 1, -100, 100), 0)

            self.pitch = round(self.mapp(pygame.joystick.Joystick(1).get_axis(1), -1, 1, -100,100), 0)

            self.thrt = round(self.mapp(pygame.joystick.Joystick(1).get_axis(2), -1, 1, 2000, 980), 0)


            if (self.thrt < 995):
                self.thrt = 980
            if (self.thrt > 2000):
                self.thrt = 1500

            if (pygame.joystick.Joystick(1).get_button(1) == 1):
                self.relay = 980

            if (pygame.joystick.Joystick(1).get_button(4) == 1):
                self.relay = 1500



            if(pygame.joystick.Joystick(1).get_button(2)==1):
                self.arm = 2000
            if (pygame.joystick.Joystick(1).get_button(3) == 1):
                self.arm = 980

            if(pygame.joystick.Joystick(1).get_button(20) == 1):
                self.yawA=10

            if (pygame.joystick.Joystick(1).get_button(22) == 1):
                self.yawE = 10


            if (pygame.joystick.Joystick(1).get_button(9) == 1):
                self.modestc = 1000

            if (pygame.joystick.Joystick(1).get_button(11) == 1):
                self.modestc = 1400

            if (pygame.joystick.Joystick(1).get_button(13) == 1):
                self.modestc = 1900

            if (pygame.joystick.Joystick(1).get_button(14) == 1):
                print("pressed")
                self.zoomCons = self.zoomCons + 1
            if (pygame.joystick.Joystick(1).get_button(5) == 1):
                print("pressed")
                self.zoomCons = self.zoomCons - 1
                if (self.zoomCons == 0):
                    self.zoomCons = 1


            time.sleep(0.1)

            self.yawE=0
            self.yawA=0

    def KmdServerVeriYolla(self):

        conn, adr = self.KumandaServerVeriYolla.accept()
        self.conOk=True

        while True:

            try:

                TelemMsg = str(self.OnOff)+"/"+str(self.roll) + "/" + str(self.pitch) + "/" + str(self.thrt) + "/" + str(
                    self.yaw) + "/" + str(
                    self.arm) + "/" + str(self.modestc) + "/" + str(self.yawA) + "/" + str(self.yawE) + "/" + str(self.relay) + "/"
                dT = TelemMsg.split("/")
                #print(len(dT))
                if(len(dT)==11):

                    conn.send(TelemMsg.encode('utf-8'))
                else:
                    print("KmndDataError")
                time.sleep(0.1)

            except socket.error:

                print("kmnd server trying to connect...")
                self.conOk=False
                conn, adr = self.KumandaServerVeriYolla.accept()
                self.conOk = True
                print("kmnd server reconnect ok")







class GUI:

    realIP = '192.168.1.10'
    simuIP = '127.0.0.1'

    def __init__(self):


        #self.mapFrame = cv2.imread("sonar2.JPG")

        self.hedefPic = cv2.imread("hedefYok.jpg")

        self.visionLogo = cv2.imread("VISION.jpg")

        self.uhsstLogo = cv2.imread("uhsst.png")

        self.mapFrame = cv2.imread("mimarsinanharita.JPG")
        self.mapTarget1 = cv2.imread("redD2.PNG")

        self.zoomKat=1

        self.telem = Telemetri(ip=self.simuIP)
        self.vid = Video(ip=self.simuIP)
        #self.task = Task()
        self.kmd = Kumanda(ip=self.simuIP)


        self.cap = cv2.VideoCapture(0)
        self.win = Tk()

        width = self.win.winfo_screenwidth()
        height = self.win.winfo_screenheight()
        # setting tkinter window size
        self.win.geometry("%dx%d" % (width, height))

        #self.win.geometry("1920x1080")
        #self.win.attributes('-fullscreen',True)

        self.win.title("VISION R&D TEAM USER INTERFACE")



        self.label_1 = Label(self.win, text="hi everyone",bg="white")
        self.label_2 = Label(self.win, text="hi2 everyone", bg="black")
        self.label_3 = Label(self.win, text="hi3 everyone", bg="red")
        self.label_4 = Label(self.win, text="hi3 everyone", bg="white")
        self.label_5 = Label(self.win, text="hi3 everyone", bg="white")
        self.label_6 = Label(self.win, text="hi3 everyone", bg="white")

        self.label_2.place(x=0,y=405)
        self.label_3.place(x=0, y=405)
        self.label_4.place(x=680, y=80)
        self.label_5.place(x=915, y=80)
        self.label_6.place(x=1150, y=80)


        """
        self.btnCamAc = Button(self.win, text="Kamera", font="italic 10 bold", bg="Yellow", fg="black",
                                  command=self.videoAc)

        self.btnCamAc.place(x=660, y=30, width=90, height=25)

        self.btnTelemAc = Button(self.win, text="Telemetri", font="italic 10 bold", bg="Yellow", fg="black",
                               command=self.telemAc)

        self.btnTelemAc.place(x=760, y=30, width=90, height=25)
        """
        self.btnKmndAc = Button(self.win, text="TX", font="italic 10 bold", bg="Yellow", fg="black",
                                 command=self.kmndAc)

        self.btnKmndAc.place(x=1400, y=115, width=110, height=25)





        self.win.configure(bg='black')

        self.labelBaglantiBas = Label(self.win,text="IP/Port",font="italic 15 bold",bg="black",fg="white")
        self.labelBaglantiBas.place(x=1115,y=27,width=105,height=20)

        self.labelBaglantiVal = Label(self.win, text=self.telem.ip+str("/")+self.telem.port,font="italic 15 bold",bg="black",fg="white")
        self.labelBaglantiVal.place(x=1202, y=27, width=200, height=20)

        self.btnBaglanti = Button(self.win,text="Bağlantı Kur",font="italic 10 bold",bg="Green",fg="White",command=self.connect)
        self.btnBaglanti.place(x=1402,y=25,width=110,height=25)

        self.btnBaglantiCancel = Button(self.win, text="KAPAT", font="italic 10 bold", bg="Red", fg="White",command=self.killAllThings)
        #self.btnBaglantiCancel.place(x=1382, y=25, width=110, height=25)


        self.labelModeBas = Label(self.win,text="Mode",font="italic 15 bold",bg="black",fg="white")
        self.labelModeBas.place(x=792,y=410,width=100,height=50)

        self.labelModeVal = Label(self.win, text="GUIDED", font="italic 15 bold",bg="black",fg="white")
        self.labelModeVal.place(x=792, y=460, width=100, height=50)


        self.labelHeadingBas = Label(self.win, text="Heading", font="italic 15 bold",bg="black",fg="white")
        self.labelHeadingBas.place(x=985, y=410, width=100, height=50)

        self.labelHeadingVal = Label(self.win, text="360", font="italic 15 bold",bg="black",fg="white")
        self.labelHeadingVal.place(x=985, y=460, width=100, height=50)

        self.labelHBas = Label(self.win, text="Yükseklik", font="italic 15 bold",bg="black",fg="white")
        self.labelHBas.place(x=1188, y=410, width=100, height=50)

        self.labelHVal = Label(self.win, text="0", font="italic 15 bold",bg="black",fg="white")
        self.labelHVal.place(x=1188, y=460, width=100, height=50)



        self.labelGrndSpdBas = Label(self.win, text="GroundSpeed", font="italic 15 bold", bg="black", fg="white")
        self.labelGrndSpdBas.place(x=772, y=540, width=140, height=50)

        self.labelGrndSpdVal = Label(self.win, text="0", font="italic 15 bold", bg="black", fg="white")
        self.labelGrndSpdVal.place(x=820, y=585, width=40, height=50)

        self.labelairSpdBas = Label(self.win, text="AirSpeed", font="italic 15 bold", bg="black", fg="white")
        self.labelairSpdBas.place(x=965, y=540, width=140, height=50)

        self.labelairSpdVal = Label(self.win, text="0", font="italic 15 bold", bg="black", fg="white")
        self.labelairSpdVal.place(x=1020, y=585, width=40, height=50)

        self.labelarmBas = Label(self.win, text="Arm/Disarm", font="italic 15 bold", bg="black", fg="white")
        self.labelarmBas.place(x=1168, y=540, width=140, height=50)

        self.labelarmVal = Label(self.win, text="Disarm", font="italic 15 bold", bg="black", fg="white")
        self.labelarmVal.place(x=1195, y=585, width=80, height=50)

        self.labelBatteryBas = Label(self.win, text="Battery Voltage", font="italic 15 bold", bg="black", fg="white")
        self.labelBatteryBas.place(x=772, y=650, width=160, height=50)

        self.labelBatteryVal = Label(self.win, text="0", font="italic 15 bold", bg="black", fg="white")
        self.labelBatteryVal.place(x=772, y=695, width=160, height=50)

        self.labelEKFBas = Label(self.win, text="GPS Satellite", font="italic 15 bold", bg="black", fg="white")
        self.labelEKFBas.place(x=965, y=650, width=160, height=50)

        self.labelEKFVal = Label(self.win, text="0", font="italic 15 bold", bg="black", fg="white")
        self.labelEKFVal.place(x=965, y=695, width=160, height=50)






        self.btnModeFBWA = Button(self.win, text="GUIDED", font="italic 10 bold", bg="Yellow", fg="black",
                                  command=self.modeFBWB)

        self.btnModeFBWA.place(x=1400, y=470, width=110, height=25)

        self.btnModeRTL = Button(self.win, text="LAND", font="italic 10 bold", bg="Yellow", fg="black",
                                 command=self.modeRTL
                                 )

        self.btnModeRTL.place(x=1400, y=420, width=110, height=25)

        self.btnModeAUTO = Button(self.win, text="AUTO", font="italic 10 bold", bg="Yellow", fg="black",
                                  command=self.modeAuto
                                  )

        self.btnModeAUTO.place(x=1400, y=370, width=110, height=25)

        self.btnModeArm = Button(self.win, text="ARM", font="italic 10 bold", bg="Red", fg="black",
                                  command=self.arm
                                  )

        self.btnModeArm.place(x=1400, y=160, width=110, height=25)

        self.btnModeDisarm = Button(self.win, text="DISARM", font="italic 10 bold", bg="Green", fg="black",command=self.disarm)

        self.btnModeDisarm.place(x=1400, y=205, width=110, height=25)

        self.btnModeStab = Button(self.win, text="STABILIZE", font="italic 10 bold", bg="Yellow", fg="black",
                                    command=self.modeStab)

        self.btnModeStab.place(x=1400, y=325, width=110, height=25)



        self.labelRadBas = Label(self.win, text="Loiter Radius", font="italic 15 bold", bg="black", fg="white")
        self.labelRadBas.place(x=1382, y=520, width=140, height=50)


        self.w = Scale(self.win, from_=0, to=130,orient='horizontal')
        self.w.place(x=1400,y=565)

        self.btnRadius = Button(self.win, text="SET", font="italic 10 bold", bg="Yellow", fg="black",
                                command=self.radius)

        self.btnRadius.place(x=1405, y=625, width=95, height=25)

        self.btntarget1Vur = Button(self.win, text="VUR", font="italic 10 bold", bg="red", fg="black")

        self.btntarget1Vur.place(x=710,y=310,width=65, height=25)

        self.btntarget1ipt = Button(self.win, text="İPT", font="italic 10 bold", bg="green", fg="black")

        self.btntarget1ipt.place(x=790, y=310, width=65, height=25)



        self.btntarget2Vur = Button(self.win, text="VUR", font="italic 10 bold", bg="red", fg="black")

        self.btntarget2Vur.place(x=946, y=310, width=65, height=25)

        self.btntarget2ipt = Button(self.win, text="İPT", font="italic 10 bold", bg="green", fg="black")

        self.btntarget2ipt.place(x=1026, y=310, width=65, height=25)





        self.btntarget3Vur = Button(self.win, text="VUR", font="italic 10 bold", bg="red", fg="black")

        self.btntarget3Vur.place(x=1182, y=310, width=65, height=25)

        self.btntarget3ipt = Button(self.win, text="İPT", font="italic 10 bold", bg="green", fg="black")

        self.btntarget3ipt.place(x=1262, y=310, width=65, height=25)



        self.labellogo = Label(self.win, text="hi everyone", bg="black")
        self.labellogo.place(x=1340, y=700)

        self.labelUlogo = Label(self.win, text="hi everyone", bg="black")
        self.labelUlogo.place(x=1220, y=680)


        self.tlabel = threading.Thread(target=self.LabelUpdate)


        self.tlabelStop = False

        self.guiStop = False



        self.hedef = "Pasif"
        self.color = "Green"

        self.i=0

        self.kill=False


    def nothing(self):

        pass


    def killAllThings(self):

        self.kill=True

    def telemAc(self):

        self.telem.OnOff=True
        self.btnTelemAc.config(command=self.telemKapa,bg='Green')


    def telemKapa(self):
        self.telem.OnOff=False
        self.btnTelemAc.config(command=self.telemAc,bg='Red')


    def videoAc(self):

        self.vid.OnOff=True
        self.btnCamAc.config(command=self.videoKapa,bg='Green')


    def videoKapa(self):
        self.vid.OnOff = False
        self.btnCamAc.config(command=self.videoAc, bg='Red')


    def kmndAc(self):

        self.kmd.OnOff=True
        self.btnKmndAc.config(command=self.kmndKapa,bg='Green')


    def kmndKapa(self):
        self.kmd.OnOff=False
        self.btnKmndAc.config(command=self.kmndAc,bg='Red')


    def controlCon(self):
        mapUpdate = threading.Thread(target=self.upt)
        while True:

            if(self.telem.x==1):
                self.btnBaglanti.config(text="Başarılı Bağlantı", bg="Green", fg="Black", font="italic 8 bold")
                mapUpdate.start()
                break



    def connectionSignalControl(self):

        while True:

            if(self.kmd.conOk==False and self.telem.conOnOff1==False and self.telem.conOnOff2==False and self.vid.conOnOff==False ):
                self.btnBaglanti.config(text="Client Bekleniyor", bg="Red", fg="Black", font="italic 8 bold")

            else:
                self.btnBaglanti.config(text="Başarılı Bağlantı", bg="Green", fg="Black", font="italic 8 bold",command=self.nothing)

            time.sleep(0.1)



    def connect(self):

        print("connect cliked")
        self.btnBaglanti.config(text="Client Bekleniyor",bg="Yellow",fg="Black",font="italic 8 bold")
        print("connect cliked2")

        self.tControl = threading.Thread(target=self.controlCon)

        self.tControl.start()

        self.tTelemSvrAl = threading.Thread(target=self.telem.TlmServerVeriAl)

        self.tTelemSvrYolla = threading.Thread(target=self.telem.TlmServerVeriYolla)

        self.tVideoAl = threading.Thread(target=self.vid.grntAl)



        #self.tTaskVeriAl = threading.Thread(target=self.task.TskServerVeriAl)

        #self.tTaskVeriYolla = threading.Thread(target=self.task.TskServerVeriYolla)



        self.tTelemSvrAl.start()

        self.tVideoAl.start()



        self.tTelemSvrYolla.start()

        #self.tTaskVeriAl.start()

        #self.tTaskVeriYolla.start()

        self.tlabel.start()



        self.t_kmnd = threading.Thread(target=self.kmd.run)
        self.t_kmnd_veri_yolla = threading.Thread(target=self.kmd.KmdServerVeriYolla)

        self.t_kmnd.start()
        self.t_kmnd_veri_yolla.start()


        self.signalCon = threading.Thread(target=self.connectionSignalControl)
        self.signalCon.start()


        




    def ibo(self):
        print("iboli")


    def hedefVur(self):

        self.task.TaskMsg="Hedef"+"/"+"Vur"
        time.sleep(0.3)
        self.task.TaskMsg = "Nan" + "/" + "Nan"

    def hedefIptal(self):

        self.task.TaskMsg="Hedef"+"/"+"İptal"
        time.sleep(0.3)
        self.task.TaskMsg = "Nan" + "/" + "Nan"




    def takeoff(self):
        self.telem.TelemMsg = "Takeoff" + "/" + str("Takeoff")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"

    def takeoffIpt(self):
        self.telem.TelemMsg = "TkfIpt" + "/" + str("Takeoff")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"




    def arm(self):
        self.telem.TelemMsg = "ArmDisarm" + "/" + str("True")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"


    def disarm(self):
        self.telem.TelemMsg = "ArmDisarm" + "/" + str("False")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"

    def modeStab(self):


        self.telem.TelemMsg = "Mode"+"/"+str("STABILIZE")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg ="Nan/Nan"


    def modeAuto(self):


        self.telem.TelemMsg = "Mode"+"/"+str("AUTO")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg ="Nan/Nan"

    def modeRTL(self):

        self.telem.TelemMsg = "Mode" + "/" + str("LAND")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"

    def modeFBWB(self):

        self.telem.TelemMsg = "Mode" + "/" + str("GUIDED")
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"


    def radius(self):

        self.telem.TelemMsg = "Radius" + "/" + str(self.w.get())
        time.sleep(0.3)
        print(self.telem.TelemMsg)
        self.telem.TelemMsg = "Nan/Nan"




    def upt(self):

        while True:



            lat = self.telem.lat
            lon = self.telem.lon
            xx, yy = self.saniyeTopixel(x=lon, y=lat)

            self.label_3.place(x=xx)
            self.label_3.place(y=405+yy)

            time.sleep(0.2)


    def mapp(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def dd2dms(self,deg):
        d = int(deg)
        md = abs(deg - d) * 60
        m = int(md)
        sd = (md - m) * 60
        saniye = m*60+sd
        return saniye

    def saniyeTopixel(self,x,y):

        """

        latMin = 1807.39116
        latMax = 1892.07288

        lonMin = 275.49576
        lonMax = 473.7906

        pixelXmin=0
        pixelXmax=640
        pixelYmin=0
        pixelYmax=350

        """

        latMin = 671.9767
        latMax = 677.4058

        lonMin = 460.129
        lonMax = 470.6344

        pixelXmin = 0
        pixelXmax = 640
        pixelYmin = 0
        pixelYmax = 350



        xsaniye = self.dd2dms(x)
        ysaniye = self.dd2dms(y)

        #print("saniye:",xsaniye,ysaniye)

        pixelX = self.mapp(xsaniye,lonMin,lonMax,pixelXmin,pixelXmax)

        pixelY = self.mapp(ysaniye,latMax,latMin,pixelYmin,pixelYmax)

        #print("pixel:",round(pixelX,0),round(pixelY,0))

        return round(pixelX,0),round(pixelY,0)



    def to_pil(self, img, label, x, y, w, h):
        img = cv2.resize(img, (w, h))

        #img = cv2.flip(img, 1)

        image = Img.fromarray(img)
        pic = ImageTk.PhotoImage(image)
        label.configure(image=pic)
        label.image = pic
        label.place(x=x, y=y)

    def to_pil2(self, img, label,w, h):
        img = cv2.resize(img, (w, h))

        #img = cv2.flip(img, 1)

        image = Img.fromarray(img)
        pic = ImageTk.PhotoImage(image)
        label.configure(image=pic)
        label.image = pic
        #label.place(x=x, y=y)

    def zoom(self,img, zoom_factor=5):
        y_size = img.shape[0]
        x_size = img.shape[1]

        # define new boundaries
        x1 = int(0.5 * x_size * (1 - 1 / zoom_factor))
        x2 = int(x_size - 0.5 * x_size * (1 - 1 / zoom_factor))
        y1 = int(0.5 * y_size * (1 - 1 / zoom_factor))
        y2 = int(y_size - 0.5 * y_size * (1 - 1 / zoom_factor))

        # first crop image then scale
        img_cropped = img[y1:y2, x1:x2]
        return cv2.resize(img_cropped, None, fx=zoom_factor, fy=zoom_factor)



    # img =cv2.imread('shape.jpg')


    def HUD(self,frame):

        h = frame.shape[0]
        w = frame.shape[1]

        if(w!=300):

            x =  int(w/3)
            hy = int(h/2)
            genislik = h

            #print(w, h, x)

            rgb = cv2.putText(frame, str(self.telem.Cmode), (30, int((h / 3) / 3)), cv2.FONT_HERSHEY_DUPLEX, 1,
                              (255, 0, 0), 2)

            rgb = cv2.putText(rgb, str(self.kmd.zoomCons)+str("X"), (int(w/2)-30, int((h/3)/3)), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 4)
            h = int(float(self.telem.Ch))

            rgb = cv2.putText(rgb, str(self.telem.armedd), (int(w/2)+int(w/3),int((genislik/3)/3)),
                              cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)
            h = int(float(self.telem.Ch))

            hpixel = int(self.kmd.mapp(h,0,50,0,hy))

            rgb = cv2.line(rgb, (x, hy), (x * 2, int(hy)), (0, 0, 0), 3)

            rgb = cv2.line(rgb,(x,hy-hpixel),(x*2,hy-hpixel),(0,255,0),3)
            #print(hy-hpixel)
            rgb = cv2.putText(rgb, str(self.telem.Ch), (int(1.4*x+15), hy-hpixel), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)#yükseklik gösterge
            #print(h)




        else:
            rgb = frame
        return rgb





    def display(self):

        #_, img = self.cap.read()
        ft = self.vid.frame
        rgb = cv2.cvtColor(ft, cv2.COLOR_BGR2RGB)

        rgb = self.zoom(rgb,zoom_factor=self.kmd.zoomCons)

        rgb = self.HUD(rgb)


        self.to_pil(rgb,self.label_1, x=0,y=0,w=640,h=400)


        rgb2 = cv2.cvtColor(self.mapFrame, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgb2, self.label_2, w=640, h=350)

        rgb3 = cv2.cvtColor(self.mapTarget1, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgb3, self.label_3, w=8, h=7)

        rgb4 = cv2.cvtColor(self.hedefPic, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgb4, self.label_4, w=200, h=200)

        rgb5 = cv2.cvtColor(self.hedefPic, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgb5, self.label_5, w=200, h=200)

        rgb6 = cv2.cvtColor(self.hedefPic, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgb6, self.label_6, w=200, h=200)

        rgbVLogo = cv2.cvtColor(self.visionLogo, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgbVLogo, self.labellogo, w=177, h=67)

        rgbULogo = cv2.cvtColor(self.uhsstLogo, cv2.COLOR_BGR2RGB)
        self.to_pil2(rgbULogo, self.labelUlogo, w=120, h=120)


        self.label_1.after(20, self.display)



        #self.label_2.after(20, self.display)
        #print("sa")

        #print(self.svr.msg2)


    def display2(self):

        rgb2 = cv2.cvtColor(self.mapFrame, cv2.COLOR_BGR2RGB)
        self.to_pil(rgb2, self.label_2, x=0, y=405, w=640, h=350)
        self.label_2.after(20, self.display)
        #print("sa")


    def dis(self):

        while self.kill==False:
            print("display")
            self.display()
            #self.display2()
            self.win.mainloop()
            self.win.mainloop()
        print("killed")




    def LabelUpdate(self):
        while True:
            #print("saUp")
            self.labelModeVal.config(text=self.telem.Cmode)
            self.labelHeadingVal.config(text=self.telem.Chead)
            self.labelHVal.config(text=self.telem.Ch)
            self.labelGrndSpdVal.config(text=self.telem.Cgrndspd)
            self.labelairSpdVal.config(text=self.telem.Cairspd)
            self.labelarmVal.config(text=self.telem.armedd)
            self.labelBatteryVal.config(text=self.telem.battery)
            self.labelEKFVal.config(text=self.telem.gpsStatus)
            time.sleep(0.1)




gui = GUI()
gui.dis()




































































