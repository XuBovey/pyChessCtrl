import sys
from threading import Thread
import time
import serial
from robot_arm_ik import robot_arm_ik
from robot_key_ctrl import ROBOT_KeyCtrl

COM_PORT = 'COM13'

_STEP = 50
_CMD = 0
demo_show = 0

p_home = [0,460,300]
p0 = [-428, 142, 255]
p1 = [385, 420, 68]
p2 = [160, 575, 68]
p3 = [-55, 580, 68]
p4 = [-240, 520, 68]
postion_list = [p0,p1,p2,p3,p4]

class Robot():
    def __init__(self, transport=None):
        self.ik = robot_arm_ik() # support postion to angle

        self.transport = transport
        self.anglePerStep = 360/(64*200*10) # 64细分，1.8度步进角，10:1减速器
        self.LanglePerStep = 360/(64*200) # 64细分，1.8度步进角

        self.mStop = True # stop
        self.nStop = True # stop
        self.oStop = True # stop
        self.lStop = True # stop
        self.lLimitStopEvent = False

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

        self._running = True
        self.stateUpdateTime = time.time()

    def getMotoStopVol(self, moto):
        if moto == 'M':
            return self.mStop
        if moto == 'N':
            return self.nStop
        if moto == 'O':
            return self.oStop
        if moto == 'L':
            return self.lStop
    
    def setMotoStopVol(self, moto, vol):
        if moto == 'M':
            self.mStop = vol
        if moto == 'N':
            self.nStop = vol
        if moto == 'O':
            self.oStop = vol
        if moto == 'L':
            self.lStop = vol
    
    def rxTask(self):
        while self._running == True:
            try:
                if self.transport.in_waiting:
                    data = self.transport.readline()
                    str_data = str(data)
                    if "STOPPED" in str_data: # moveto target stop
                        if "M.STATUS" in str_data:
                            self.setMotoStopVol('M',True)
                        elif "N.STATUS" in str_data:
                            self.setMotoStopVol('N',True)
                        elif "O.STATUS" in str_data:
                            self.setMotoStopVol('O',True)
                        elif "L.STATUS" in str_data:
                            self.setMotoStopVol('L',True)
                    elif 'stop' in str_data: # limited stop
                        if "M.stop" in str_data:
                            self.setMotoStopVol('M',True)
                        elif "N.stop" in str_data:
                            self.setMotoStopVol('N',True)
                        elif "O.stop" in str_data:
                            self.setMotoStopVol('O',True)
                        elif "L.stop" in str_data:
                            self.setMotoStopVol('L',True)
                            self.lLimitStopEvent = True
                            print("L LIMITED STOP")
                    else:
                        # print(str_data)
                        pass
            except:
                return

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

        m1, m2, m3 = self.ik.armAngle_to_motorAngle(armAngle)
        # print(m1,m2,m3)

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

    def goPostion(self, xyz, style = 0):
        # print("go postion:",xyz)

        if self.targetX == xyz[0] and self.targetY == xyz[1] and self.targetZ == xyz[2]:
            return

        mSteps, nSteps, oSteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        if mSteps == 0 and nSteps == 0 and oSteps == 0:
            self.curX = self.targetX
            self.curY = self.targetY
            self.curZ = self.targetZ
            return

        self.targetX = xyz[0]
        self.targetY = xyz[1]
        self.targetZ = xyz[2]
        if style == 0: # 先移动底盘，后移动大、小臂
            self.setMotoStopVol('M', False)
            self.cmdSend("setM:{:}\n".format(mSteps))
            while not self.isStop():
                time.sleep(3)
            self.setMotoStopVol('N', False)
            self.setMotoStopVol('O', False)
            self.cmdSend("setN:{:}setO:{:}\n".format(nSteps, oSteps))
            while not self.isStop():
                time.sleep(3)
        if style == 1:
            self.setMotoStopVol('N', False)
            self.setMotoStopVol('O', False)
            self.cmdSend("setN:{:}setO:{:}\n".format(nSteps, oSteps))
            while not self.isStop():
                time.sleep(3)
            self.setMotoStopVol('M', False)
            self.cmdSend("setM:{:}\n".format(mSteps))
            while not self.isStop():
                time.sleep(3)

    def mvPostion(self, xyz):
        self.goPostion([xyz[0] + self.curX, xyz[1] + self.curY, xyz[2] + self.curZ])

    def mvZ(self, z):
        self.goPostion([self.curX, self.curY, z + self.curZ])
    
    def mvY(self, y):
        self.goPostion([self.curX, y + self.curY, self.curZ])

    def mvX(self, x):
        self.goPostion([x + self.curX, self.curY, self.curZ])
    
    def lenToAngle(self, len):
        return len/(165/360)

    def mvL(self, len):
        self.mvAngle('L', self.lenToAngle(len))
        while not self.lStop:
            time.sleep(2)

    # moto = 'M', 'N', 'O'
    # angle = -180~180
    def mvAngle(self, moto, angle):
        self.setMotoStopVol(moto, False)
        anglePerStep = self.anglePerStep
        if moto == 'L':
            anglePerStep = self.LanglePerStep
        self.cmdSend("add{:}:{:}\n".format(moto, angle / anglePerStep))
        while not self.isStop():
            time.sleep(2)
   
    def setX(self, x):
        self.goPostion([x, self.curY, self.curZ])

    def pick(self):
        self.cmdSend("setP.pump:0\n")
    
    def release(self):
        self.cmdSend("setP.pump:1\n")

    def reset(self):
        self.release()
        # 远离限位器移动一点距离，防止已经运动到限位器位置
        self.mvAngle('M', -3)
        self.mvAngle('N', -5)
        self.mvAngle('O', -3)
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('O', False)
        while not self.isStop():
            time.sleep(2)

        self.mvAngle('M', 360)
        self.mvAngle('N', 360)
        self.mvAngle('O', 360)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('O', False)
        while not self.isStop():
            time.sleep(2)

        self.cmdSend("setM.currentPosition:{:}\n".format(74 / self.anglePerStep))
        self.cmdSend("setN.currentPosition:{:}\n".format(90 / self.anglePerStep))
        self.cmdSend("setO.currentPosition:{:}\n".format(40 / self.anglePerStep))
        time.sleep(0.5)
        self.mvAngle('M', -74)
        self.mvAngle('N', -5)
        self.mvAngle('O', -40)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('O', False)
        while not self.isStop():
            time.sleep(2)

    def isStop(self):
        if self._running == False:
            return True

        if self.mStop  and self.nStop and self.oStop:
            return True
        elif self.stateUpdateTime +2 <= int(time.time()):
                if self.getMotoStopVol('M') == False:
                    self.cmdSend("getM.status\n")
                if self.getMotoStopVol('N') == False:
                    self.cmdSend("getN.status\n")
                if self.getMotoStopVol('O') == False:
                    self.cmdSend("getO.status\n")
                if self.getMotoStopVol('L') == False:
                    self.cmdSend("getL.status\n")
                self.stateUpdateTime = int(time.time())
        return False

class new_retail():
    def __init__(self, robot = None):
        self.robot = robot

    def go_home(self):
        self.robot.goPostion(p_home)
        time.sleep(1)

    def pick_from_postion(self, p):
        pre_postion = [p[0]* 0.8, p[1]* 0.8, p[2]* 1.2]
        print("goto pre posion")
        self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("goto target postion")
        self.robot.goPostion(p, style=1) # 先动大小臂
        print("pick")
        self.robot.pick()
        time.sleep(1)
        print("move O 1 degree")
        self.robot.mvAngle('O', 5)
        time.sleep(1)
        print("goto pre posion")
        self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("go home")
        self.robot.goPostion(p_home, style=1) # 先动大小臂

    def put_to_postion(self, p):
        pre_postion = [p[0]* 0.8, p[1]* 0.8, p[2]* 1.2]
        print("goto pre posion")
        self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("goto target postion")
        self.robot.goPostion(p, style=1) # 先动大小臂
        print("release")
        self.robot.release()
        time.sleep(1)
        print("move O 5 degree")
        self.robot.mvAngle('O', 5)
        time.sleep(1)
        print("goto pre posion")
        self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("go home")
        self.robot.goPostion(p_home, style=1) # 先动大小臂

    def demo1(self):
        i = 0
        pic_p = postion_list[0]
        for p in postion_list:
            if i == 0: 
                i += 1
                continue
            self.robot.setMotoStopVol('L', False)
            self.robot.lLimitStopEvent = False
            while self.robot.lLimitStopEvent == False and self.robot._running:
                print("move line0")
                self.robot.mvL(650)
            print("move line1")
            self.robot.mvL(300)
            print("go home")
            self.robot.goPostion(p_home)
            print("pick from p[{:}]".format(i))
            self.pick_from_postion(pic_p)
            time.sleep(1)
            print("pick from p[{:}]".format(i))
            self.put_to_postion(p)
            i += 1

    
def key_cmd_list():
    print("Z - exit     R - reset")
    print("P - pick     O - put")
    print("A - x+       D - x-")
    print("W - y+       S - y-")
    print("Q - z+       E - z-")
    print("N - L+       N - z-")
    print("T - demo    ")
    print("B - go home  H - help")

def key_cb(key):
    global _STEP
    global _CMD
    global demo_show
    # print(key.upper())
    if key == 'Z': robot._running = False
    if key == 'R': 
        robot.reset() 
        print("reset done")
    if key == 'H': key_cmd_list()
    if key == 'P': robot.pick()
    if key == 'O': robot.release()
    if key == 'A': robot.mvX(_STEP)
    if key == 'D': robot.mvX(-_STEP)
    if key == 'W': robot.mvY(_STEP)
    if key == 'S': robot.mvY(-_STEP)
    if key == 'Q': robot.mvZ(_STEP)
    if key == 'E': robot.mvZ(-_STEP)
    if key == 'M': robot.mvL(100)
    if key == 'N': robot.mvL(-100)
    if key == 'B': robot.goPostion(p_home)
    if key == '+' or key == '=': 
        _STEP = _STEP + 10 if _STEP < 100  else 100
        print("_STEP:",_STEP)
    if key == '-' or key == '_': 
        _STEP = _STEP - 10 if _STEP > 20  else 10
        print("_STEP:",_STEP)
    if key == '1': _CMD = 1
    if key == '2': _CMD = 2
    if key == '3': _CMD = 3
    if key == '4': _CMD = 4
    if key == '5': _CMD = 5
    if key == 'T': demo_show = 1

transport = serial.Serial(port=COM_PORT, baudrate = 115200)
robot = Robot(transport)
robotRxTask = Thread(target = robot.rxTask)

if __name__ == '__main__':

    print("start")
    print("\r\n\r\n警告: 启动前确保，限位器没有动作，且远离其他结构件！\r\n\r\n")
    robotRxTask.start()

    key_ctr= ROBOT_KeyCtrl(key_cb)
    key_cmd_list()

    demo = new_retail(robot)

    while demo.robot._running == True:
        if _CMD != 0:
            wait_go_postion(postion_list[_CMD-1])
            _CMD = 0
        # while not robot.isStop():
        #     time.sleep(0.5)
        if demo_show == 1:
            demo_show = 0
            demo.demo1()
        time.sleep(1)

    print("exit")
    if transport.isOpen():
        transport.flush()
        transport.close()
