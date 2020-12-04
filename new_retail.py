import sys
from threading import Thread
import time
import serial
from robot_arm_ik import robot_arm_ik

class Robot():
    def __init__(self, transport=None):
        self.ik = robot_arm_ik() # support postion to angle

        self.transport = transport
        self.anglePerStep = 360/(64*200*10) # 64细分，1.8度步进角，10:1减速器

        self.mState = 0 # stop
        self.nState = 0 # stop
        self.oState = 0 # stop

        self.curX = 0
        self.curY = 0
        self.curZ = 0

        self.targetX = 0
        self.targetY = 0
        self.targetZ = 0

        self.m1Max = 90
        self.m1Min = -90
        self.m2Max = 90
        self.m2Min = 17.0
        self.m3Max = 60 #40
        self.m3Min = -78.0
        self.deltM2M3Min = 2
        self.deltM2M3Max = 130
    
    def rxTask(self):
        while True:
            if self.transport.in_waiting:
                data = self.transport.readline()
                str_data = str(data)
                if "M.STATUS" in str_data:
                    if "STOPPED" in str_data:
                        self.mState = 0
                        self.curX = self.targetX
                elif "N.STATUS" in str_data:
                    if "STOPPED" in str_data:
                        self.nState = 0
                        self.curY = self.targetY
                elif "O.STATUS" in str_data:
                    if "STOPPED" in str_data:
                        self.oState = 0
                        self.curZ = self.targetZ
                else:
                    print(str(data))
            time.sleep(0.1)

    def cmdSend(self, data):
        # print(">>:", data)
        self.transport.write(data.encode('UTF-8'))

    def angleToStep(self, m1Angle, m2Angle, m3Angle):
        mSteps = m1Angle / self.anglePerStep
        nSteps = m2Angle / self.anglePerStep
        oSteps = m3Angle / self.anglePerStep
        return mSteps, nSteps, oSteps

    def xyzToStep(self,x,y,z): # unit: mm
        armAngle = [0,0,0]
        if False == self.ik.postion_to_angle([x,y,z], armAngle):
            print("Error: xyzToStep postion1")
            return (0,0,0)

        print(armAngle)
        m1, m2, m3 = self.ik.armAngle_to_motorAngle(armAngle)
        print(m1,m2,m3)

        # if m1 < self.m1Min or m1 > self.m1Max:
        #     print("Error: xyzToStep postion2")
        #     return (0,0,0)
        # if m2 < self.m2Min or m2 > self.m2Max:
        #     print("Error: xyzToStep postion3")
        #     return (0,0,0)
        # if m3 < self.m3Min or m3 > self.m3Max:
        #     print("Error: xyzToStep postion4")
        #     return (0,0,0)
        # if (m2 - m3) < self.deltM2M3Min or (m2 - m3) > self.deltM2M3Min:
        #     print("Error: xyzToStep postion5")
        #     return (0,0,0)

        return self.angleToStep(m1, m2, m3)

    def goPostion(self, xyz):
        mSteps, nSteps, oSteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        if mSteps == 0 and nSteps == 0 and oSteps == 0:
            return

        self.targetX = xyz[0]
        self.targetY = xyz[1]
        self.targetZ = xyz[2]
        self.mState = 1
        self.nState = 1
        self.oState = 1
        self.cmdSend("setM:{:}setN:{:}setO:{:}\n".format(mSteps, nSteps, oSteps))

    def mvPostion(self, xyz):
        self.goPostion([xyz[0] + self.curX, xyz[1] + self.curY, xyz[2] + self.curZ])

    def mvZ(self, z):
        self.goPostion([self.curX, self.curY, z + self.curZ])
    
    def mvY(self, y):
        self.goPostion([self.curX, y + self.curY, self.curZ])

    def mvX(self, x):
        self.goPostion([x + self.curX, self.curY, self.curZ])

    # moto = 'M', 'N', 'O'
    # angle = -180~180
    def mvAngle(self, moto, angle):
        self.cmdSend("add{:}:{:}\n".format(moto, angle / self.anglePerStep))
   
    def setX(self, x):
        self.goPostion([x, self.curY, self.curZ])

    def pick(self):
        self.cmdSend("setP.pump:0\n")
    
    def release(self):
        self.cmdSend("setP.pump:1\n")

    def reset(self):
        self.release()
        # 原理限位器移动一点距离，防止已经运动到限位器位置
        self.mvAngle('M', -3)
        self.mState = 1
        self.mvAngle('N', -5)
        self.nState = 1
        self.mvAngle('O', -3)
        self.oState = 1
        while not self.isStop():
            time.sleep(0.5)

        self.mvAngle('M', 360)
        self.mState = 1
        self.mvAngle('N', 360)
        self.nState = 1
        self.mvAngle('O', 360)
        self.oState = 1
        while not self.isStop():
            time.sleep(0.5)

        self.cmdSend("setM.currentPosition:{:}\n".format(74 / self.anglePerStep))
        self.cmdSend("setN.currentPosition:{:}\n".format(90 / self.anglePerStep))
        self.cmdSend("setO.currentPosition:{:}\n".format(40 / self.anglePerStep))
        time.sleep(0.5)
        self.mvAngle('M', -74)
        self.mState = 1
        self.mvAngle('N', -1)
        self.nState = 1
        self.mvAngle('O', -40)
        self.oState = 1
        while not self.isStop():
            time.sleep(0.5)
        
        print("reset done")

    def isStop(self):
        if self.mState == 1:
            self.cmdSend("getM.status\n")
        if self.nState == 1:
            self.cmdSend("getN.status\n")
        if self.oState == 1:
            self.cmdSend("getO.status\n")
        time.sleep(1)
        return (self.mState == 0 and self.nState == 0 and self.oState == 0)

"""
class new_retail():
    def __init__(self, robot = None, cb=None, width = 22.5, length = 22.5): # width,length mm
        self.cb = cb
        self.width = width
        self.length = length
        self.robot = robot
        self.home = [-35, -35, -20]

    def reset(self):
        self.sendCmd("reset")
    
    def getIt(self, row, col):
        # 1 移动到取棋子位置
        postion = [-row*self.width + self.home[0], -col*self.length + self.home[1], self.home[2]]
        print("move to row:{:} col:{:} x:{:}, y:{:}".format(row, col, postion[0], postion[1]))
        self.robot.goPostion(postion)
        # wait to move
        while not self.robot.isStop():
            time.sleep(0.5)
        
        # 2 吸取棋子 // pick
        self.robot.mvZ(-15) # -10mm
        print("move to chess")
        # wait to move
        while not self.robot.isStop():
            time.sleep(0.5)
        self.robot.pick()
        print("pick up")
        time.sleep(1)
        self.robot.mvZ(+15) # 10mm
        print("ready to move")
        # wait to move
        while not self.robot.isStop():
            time.sleep(0.5)

    def goHome(self):
        self.robot.goPostion(self.home)
        print("go home...")
        while not self.robot.isStop():
            time.sleep(0.5)

    def putIt(self, row, col):
        # 3 移动到目标位置
        postion = [-row*self.width + self.home[0], -col*self.length + self.home[1], self.home[2]]
        self.robot.goPostion(postion)
        print("move to row:{:} col:{:} x:{:}, y:{:}".format(row, col, postion[0], postion[1]))
        while not self.robot.isStop():
            time.sleep(0.5)
        # wait to move

        # 4 放置棋子
        self.robot.mvZ(-15) # -10mm
        while not self.robot.isStop():
            time.sleep(0.5)
        # wait to move
        self.robot.release()
        time.sleep(1)
        self.robot.mvZ(15) # 10mm
        while not self.robot.isStop():
            time.sleep(0.5)
"""
if __name__ == '__main__':
    transport = serial.Serial(port="COM12", baudrate = 115200)
    robot = Robot(transport)
    robotRxTask = Thread(target = robot.rxTask)
    robotRxTask.start()

    robot.reset()

    p1 = [385, 420, 70]
    p2 = [160, 575, 70]
    p3 = [-55, 580, 70]
    p4 = [-240, 520, 70]
    postion_list=[p1,p2,p3,p4]
    p5 = [-480, 75, 300]

    robot.pick()
    time.sleep(1)

    while True:
        # for p in postion_list:
        #     print(p)

        #     tmp = [p[0]*0.8, p[1]*0.8, p[2]*1.2]

        #     robot.goPostion(tmp)
        #     while not robot.isStop():
        #         time.sleep(0.5)

        #     robot.goPostion(p)
        #     while not robot.isStop():
        #         time.sleep(0.5)
            
        #     time.sleep(2)

        #     robot.goPostion(tmp)
        #     while not robot.isStop():
        #         time.sleep(0.5)
        p = p4
        robot.goPostion([p[0]*0.8, p[1]*0.8, 300])
        while not robot.isStop():
            time.sleep(0.5)

        robot.mvZ(300)
        while not robot.isStop():
            time.sleep(0.5)
        robot.goPostion(p5)
        while not robot.isStop():
            time.sleep(0.5)


    # demo for test
"""
    chess = Chess(robot = robot)
    chess.goHome()
    while True:
        chess.getChess(0,0)
        chess.putChess(10,0)
        chess.getChess(10,0)
        chess.putChess(10,10)
        chess.getChess(10,10)
        chess.putChess(0,10)
        chess.getChess(0,10)
        chess.putChess(0,0)
"""
