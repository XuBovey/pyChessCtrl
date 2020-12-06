import sys
from threading import Thread
import time
import serial

Z_MV_LEN = 20
X_HOME = -52
Y_HOME = -126
Z_HOME = -17

CHESS_WIDTH = 281.5/12
CHESS_LENGHT = 272.5/12

MAX_LEN     = 400
MAX_WIDTH   = 400
MAX_HIGH    = 200

chess_list = [
    [[7,'G'],[6,'F']], # Black, White
    [[7,'F'],[8,'G']],
    [[7,'H'],[7,'I']],
    [[7,'E'],[7,'D']],
    [[6,'G'],[5,'H']],
    [[5,'F'],[4,'E']],
    [[8,'I'],[9,'J']],
    [[8,'F'],[9,'E']],
    [[9,'G'],[10,'H']],
    [[6,'D'],[5,'C']],
    [[8,'E'],[8,'J']],
    [[7,'J'],[9,'K']],
    [[10,'L'],[9,'I']],
    [[11,'G'],[9,'L']],
    [[9,'H'],[9,'M']],
]

class Postion():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Robot():
    def __init__(self, transport=None):
        self.transport = transport
        self.xStepLen = 40/(32*200) # mm 步进电机每转一圈，皮带前进40mm, 32细分，1.8度步进角
        self.yStepLen = 40/(32*200) # mm 步进电机每转一圈，皮带前进40mm, 32细分，1.8度步进角
        self.zStepLen = 8/(32*200)  # mm T8丝杆导程8，即每圈8mm

        self.curPostion = Postion(0, 0, 0)
        self.targetPostion = Postion(0, 0, 0)

        self.xState = 0 # stop
        self.yState = 0 # stop
        self.zState = 0 # stop

        self._running = True
    
    def rxTask(self):
        while self._running == True:
            try:
                if self.transport.in_waiting:
                    data = self.transport.readline()
                    str_data = str(data)
                    if "X.STATUS" in str_data:
                        if "STOPPED" in str_data:
                            self.xState = 0
                    elif "Y.STATUS" in str_data:
                        if "STOPPED" in str_data:
                            self.yState = 0
                    elif "Z.STATUS" in str_data:
                        if "STOPPED" in str_data:
                            self.zState = 0
                    else:
                        print(str(data))
            except:
                return
            time.sleep(0.1)

    def cmdSend(self, data):
        # print(">>:", data)
        self.transport.write(data.encode('UTF-8'))

    def xyzToStep(self,x,y,z): # unit: mm
        xsteps = int(x / self.xStepLen)
        ysteps = int(y / self.yStepLen)
        zsteps = int(z / self.zStepLen)
        return xsteps, ysteps, zsteps

    def stepToXyz(self, xsteps, ysteps, zsteps):
        x = xsteps * self.xStepLen
        y = ysteps * self.yStepLen
        z = zsteps * self.zStepLen
        return x, y, z # unit: mm

    def goPostion(self, xyz):
        xsteps, ysteps, zsteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        self.xState = 1
        self.yState = 1
        self.zState = 1
        self.cmdSend("setX:{:}setY:{:}setZ:{:}\n".format(xsteps, ysteps, zsteps))

    def mvPostion(self, xyz):
        xsteps, ysteps, zsteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        self.xState = 1
        self.yState = 1
        self.zState = 1
        self.cmdSend("addX:{:}addY:{:}addZ:{:}\n".format(xsteps, ysteps, zsteps))

    def mvZ(self, z):
        self.zState = 1
        self.cmdSend("addZ:{:}\n".format(int(z / self.zStepLen)))
    
    def mvY(self, y):
        self.yState = 1
        self.cmdSend("addY:{:}\n".format(int(y / self.yStepLen)))

    def mvX(self, x):
        self.xState = 1
        self.cmdSend("addX:{:}\n".format(int(x / self.xStepLen)))
    
    def setX(self, x):
        self.xState = 1
        self.cmdSend("setX:{:}\n".format(int(x / self.xStepLen)))

    def pick(self):
        self.cmdSend("setP.pump:0\n")
    
    def release(self):
        self.cmdSend("setP.pump:1\n")
    
    def lock(self):
        self.cmdSend("setX.stepperen:0\n")
        self.cmdSend("setY.stepperen:0\n")
        self.cmdSend("setZ.stepperen:0\n")

    def unlock(self):
        self.cmdSend("setX.stepperen:1\n")
        self.cmdSend("setY.stepperen:1\n")
        self.cmdSend("setZ.stepperen:1\n")

    def reset(self):
        self.release()
        self.goPostion([MAX_WIDTH, MAX_LEN, MAX_HIGH]) # 移动到不可达位置，使得必然触发限位器动作

        while not self.isStop():
            time.sleep(0.5)
        
        self.cmdSend("setX.currentPosition:0\n")
        self.cmdSend("setY.currentPosition:0\n")
        self.cmdSend("setZ.currentPosition:0\n")
        time.sleep(1)
    
    def isStop(self):
        if self._running == False:
            return True
        if self.xState == 1:
            self.cmdSend("getX.status\n")
        if self.yState == 1:
            self.cmdSend("getY.status\n")
        if self.zState == 1:
            self.cmdSend("getZ.status\n")
        time.sleep(1)
        return (self.xState == 0 and self.yState == 0 and self.zState == 0)

class Chess():
    def __init__(self, robot = None, cb=None, width = 22.5, length = 22.5): # width,length mm
        self.cb = cb
        self.width = width
        self.length = length
        self.robot = robot
        self.home = [X_HOME, Y_HOME, Z_HOME]

    def reset(self):
        self.sendCmd("reset")
    
    def getChess(self, row, col):
        # 1 移动到取棋子位置
        postion = [-row*self.width + self.home[0], -col*self.length + self.home[1], self.home[2]]
        print("move to row:{:} col:{:} x:{:}, y:{:}".format(row, col, postion[0], postion[1]))
        self.robot.goPostion(postion)
        # wait to move
        while not self.robot.isStop():
            time.sleep(0.5)
        
        # 2 吸取棋子 // pick
        self.robot.mvZ(-Z_MV_LEN) # -10mm
        print("move to chess")
        # wait to move
        while not self.robot.isStop():
            time.sleep(0.5)
        self.robot.pick()
        print("pick up")
        time.sleep(1)
        self.robot.mvZ(+Z_MV_LEN) # 10mm
        print("ready to move")
        # wait to move
        while not self.robot.isStop():
            time.sleep(0.5)

    def goHome(self):
        self.robot.goPostion(self.home)
        print("go home...")
        while not self.robot.isStop():
            time.sleep(0.5)

    def putChess(self, row, col):
        # 3 移动到目标位置
        postion = [-row*self.width + self.home[0], -col*self.length + self.home[1], self.home[2]]
        self.robot.goPostion(postion)
        print("move to row:{:} col:{:} x:{:}, y:{:}".format(row, col, postion[0], postion[1]))
        while not self.robot.isStop():
            time.sleep(0.5)
        # wait to move

        # 4 放置棋子
        self.robot.mvZ(-Z_MV_LEN) # -10mm
        while not self.robot.isStop():
            time.sleep(0.5)
        # wait to move
        self.robot.release()
        time.sleep(0.5)
        self.robot.mvZ(Z_MV_LEN) # 10mm
        while not self.robot.isStop():
            time.sleep(0.5)

if __name__ == '__main__':
    transport = serial.Serial(port="COM13", baudrate = 115200)
    robot = Robot(transport)
    robotRxTask = Thread(target = robot.rxTask)
    robotRxTask.start()

    print('unlock')
    robot.unlock()
    print("reset")
    robot.reset()
    print("reset done")

    chess = Chess(robot = robot, width = CHESS_WIDTH, length = CHESS_LENGHT)
    print("go home")
    chess.goHome()
    time.sleep(2)

    step_count = 1
    for step in chess_list:
        print (step_count)
        chess.getChess(0,0)
        chess.putChess(step[0][0]-1,ord(step[0][1])-ord('A')) # black
        chess.getChess(12,12)
        chess.putChess(step[1][0]-1,ord(step[1][1])-ord('A')) # white
        step_count = step_count + 1
    
    chess.goHome()

    robotRxTask._running = False
    if transport.isOpen():
        transport.flush()
        transport.close()