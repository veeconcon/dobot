import threading
import DobotDllType as dType
import sys
import numpy as np
import pandas as pd

carryZ = 170
gripZ = 70

beltCarryPos = [64.5918, -186.9643, carryZ, -70.9412]
beltGripPos = [64.5918, -186.9643, gripZ, -70.9412]

destCarryPos = [168.8290, 107.4708, carryZ, 32.4794]
destGripPos = [168.8290, 107.4708, gripZ, 32.4794]

### 関数定義 ###

#許可待ち関数
def WaitPrm(str):
    while True:
        val = input(str+" y/n : ")
        if val=="y":
            return 0
        elif val=="n":
            sys.exit()

#ホーミング関数
def Homing(api, str):
    while True:
        val = input("Execute homing "+str+" ? y/n : ")
        if val=="y":
            dType.SetHOMECmd(api, 0, 1)
            return 0
        elif val=="n":
            return 0

def MoveTo(api, Pos):
    return dType.SetPTPCmd(api, 1, Pos[0], Pos[1], Pos[2], Pos[3], isQueued=1)[0]

def Wait(api, time):
    return dType.SetWAITCmd(api, waitTime=time, isQueued=1)

def Grip(api, startPos, gripZ):
    currentPos = startPos

    # アームを開く
    lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=True, on=False, isQueued=True)[0]
    lastIndex = Wait(api, 1000)

    # 下がる
    currentPos[2] = gripZ
    lastIndex = MoveTo(api, currentPos)
    lastIndex = Wait(api, 1000)

    # アームを閉じる
    lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=True, on=True, isQueued=True)[0]
    lastIndex = Wait(api, 1000)
    lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=False, on=True, isQueued=True)[0]

    # 上がる
    currentPos[2] = carryZ
    lastIndex = MoveTo(api, currentPos)

    return lastIndex

def Release(api, startPos, gripZ):
    currentPos = startPos

    # 下がる
    currentPos[2] = gripZ
    lastIndex = MoveTo(api, currentPos)
    lastIndex = Wait(api, 1000)

    # アームを開く
    lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=True, on=False, isQueued=True)[0]
    lastIndex = Wait(api, 1000)

    # 上がる
    currentPos[2] = carryZ
    lastIndex = MoveTo(api, currentPos)

    # アームを閉じる
    lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=True, on=True, isQueued=True)[0]
    lastIndex = Wait(api, 1000)
    lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=False, on=True, isQueued=True)[0]

    return lastIndex

def  ConveyorBeltMove(api, waitms):
    STEP_PER_CRICLE = 360.0 / 1.8 * 10.0 * 16.0
    MM_PER_CRICLE = 3.1415926535898 * 36.0
    vel = float(50) * STEP_PER_CRICLE / MM_PER_CRICLE
    lastIndex = dType.SetEMotor(api, 0, 1, int(vel), 1)
    lastIndex = Wait(api, waitms)
    lastIndex = dType.SetEMotor(api, 0, 1, 0, 1)[0]
    return lastIndex

#Dobotが指定座標に指定時間留まるまで待つ関数
# def WaitIf(api, crd, msec_check):
#     while True:
#         if all(np.array(dType.GetPose(api)[:3])-np.array(crd)<crd_err):
#             time.sleep(msec_check/1000)
#             if all(np.array(dType.GetPose(api)[:3])-np.array(crd)<crd_err):
#                 return time.time()


def main():
    CON_STR = {
        dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
        dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
        dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

    api = dType.load()

    # Dobot接続
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", CON_STR[state])

    if (state == dType.DobotConnect.DobotConnect_NoError):

        # コマンドキューを初期化
        dType.SetQueuedCmdClear(api)

        # パラメータ設定
        # dType.SetHOMEParams(api, homePos[0], homePos[1], homePos[2], homePos[3], isQueued=1)
        dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

        # ホーミング
        Homing(api, "Dobot1")

        lastIndex = ConveyorBeltMove(api, 2000)

        lastIndex = MoveTo(api, beltCarryPos)

        lastIndex = Grip(api, beltCarryPos, gripZ=120)

        lastIndex = MoveTo(api, destCarryPos)

        lastIndex = Release(api, destCarryPos, gripZ=gripZ)

        # コマンドキューを実行
        dType.SetQueuedCmdStartExec(api)

        # 全てのコマンドが実行されるまで待機
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]: # 現在のコマンド番号が最後のコマンド番号に達していない間
            dType.dSleep(100)

        # コマンドの実行停止
        dType.SetQueuedCmdStopExec(api)

    # Disconnect Dobot
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()