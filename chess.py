import sys
from threading import Thread
import time
import serial
from robot_key_ctrl import ROBOT_KeyCtrl

COM_PORT = "COM13"

Z_MV_LEN = 20
X_HOME = -52
Y_HOME = -126
Z_HOME = -17

CHESS_WIDTH = 281.5/12
CHESS_LENGHT = 272.5/12

MAX_LEN     = 500
MAX_WIDTH   = 500
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
        self.stateUpdateTime = int(time.time())
    
    def rxTask(self):
        while True:
            while self.transport.in_waiting == 0:
                if self._running == False:
                    return
                pass
            data = str(transport.readall())
            data_list = data.split('\\n')
            for str_data in data_list:
                print (str_data)

                if "STOPPED" in str_data: # moveto target stop
                    if "X.STATUS" in str_data:
                        self.xState = 0
                    elif "Y.STATUS" in str_data:
                        self.yState = 0
                    elif "Z.STATUS" in str_data:
                        self.zState = 0
                    print("STOPPED:", self.xState, self.yState, self.zState)
                elif 'stop' in str_data: # limited stop
                    if "X.stop" in str_data:
                        self.xState = 0
                    elif "Y.stop" in str_data:
                        self.yState = 0
                    elif "Z.stop" in str_data:
                        self.zState = 0
                    print("stop:", self.xState, self.yState, self.zState)

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
        self.stateUpdateTime = int(time.time())

    def mvPostion(self, xyz):
        xsteps, ysteps, zsteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        self.xState = 1
        self.yState = 1
        self.zState = 1
        self.cmdSend("addX:{:}addY:{:}addZ:{:}\n".format(xsteps, ysteps, zsteps))
        self.stateUpdateTime = int(time.time())

    def mvZ(self, z):
        self.zState = 1
        self.cmdSend("addZ:{:}\n".format(int(z / self.zStepLen)))
        self.stateUpdateTime = int(time.time())
    
    def mvY(self, y):
        self.yState = 1
        self.cmdSend("addY:{:}\n".format(int(y / self.yStepLen)))
        self.stateUpdateTime = int(time.time())

    def mvX(self, x):
        self.xState = 1
        self.cmdSend("addX:{:}\n".format(int(x / self.xStepLen)))
        self.stateUpdateTime = int(time.time())
    
    def setX(self, x):
        self.xState = 1
        self.cmdSend("setX:{:}\n".format(int(x / self.xStepLen)))
        self.stateUpdateTime = int(time.time())

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
        
        if self.xState == 0 and self.yState == 0 and self.zState == 0:
            return True

        if self.stateUpdateTime +5 <= int(time.time()):
            if self.xState == 1:
                self.cmdSend("getX.status\n")
            if self.yState == 1:
                self.cmdSend("getY.status\n")
            if self.zState == 1:
                self.cmdSend("getZ.status\n")
            self.stateUpdateTime = int(time.time())
            print("check stop cmd: ", self.stateUpdateTime, ":", self.xState, self.yState, self.zState)
        return False

class Chess():
    def __init__(self, robot = None, cb=None, width = 22.5, length = 22.5): # width,length mm
        self.cb = cb
        self.width = width
        self.length = length
        self.robot = robot
        self.home = [X_HOME, Y_HOME, Z_HOME]

    def reset(self):
        self.sendCmd("reset")
    
    def getChess(self, row, col, row_offset = 0, col_offset = 0):
        # 1 移动到取棋子位置
        postion = [-row*self.width + self.home[0] - row_offset, -col*self.length + self.home[1] - col_offset, self.home[2]]
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
        time.sleep(1)
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

def key_cb(key):
    print(key)
    if key == 'Z': 
        print("exit cmd.")
        robot._running = False
    if key == 'T':
        demoTask.start() 

def exit():
    if transport.isOpen():
        transport.flush()
        transport.close()

def demo1():
    chess.goHome()
    time.sleep(2)

    print("start demo")
    step_count = 1
    for step in chess_list:
        print (step_count)
        chess.getChess(row = 0, col = 0 + step_count - 1, col_offset = 0, row_offset= -10)
        chess.putChess(step[0][0]-1, ord(step[0][1])-ord('A')) # black
        chess.getChess(12, 12 - (step_count - 1), col_offset = 0, row_offset= 6)
        chess.putChess(step[1][0]-1, ord(step[1][1])-ord('A')) # white
        step_count = step_count + 1
        if robot._running == False:
            exit()
    
    print("pick all chess back")
    for step in chess_list:
        print (step_count)
        chess.getChess(step[0][0]-1, ord(step[0][1])-ord('A')) # black
        chess.putChess(row = 0, col = 0 + step_count - 1, col_offset = 0, row_offset= -10)
        chess.getChess(step[1][0]-1, ord(step[1][1])-ord('A')) # white
        chess.putChess(12, 12 - (step_count - 1), col_offset = 0, row_offset= 6)
        
        step_count = step_count + 1
        if robot._running == False:
            exit()
    
    print("stop demo")
    chess.goHome()

transport = serial.Serial(port=COM_PORT, baudrate = 115200, timeout = 0.1)
robot = Robot(transport)
robotRxTask = Thread(target = robot.rxTask)
chess = Chess(robot = robot, width = CHESS_WIDTH, length = CHESS_LENGHT)
demoTask = Thread(target = demo1)

if __name__ == '__main__':
    robotRxTask.start()

    key_ctr= ROBOT_KeyCtrl(key_cb)

    print('unlock')
    robot.unlock()
    print("reset")
    robot.reset()
    print("reset done")

    while robot._running == True:
        pass
    exit()
