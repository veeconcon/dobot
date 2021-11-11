import threading
import DobotDllType as dType

carryZ = 150
gripZ = 70

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
        dType.SetHOMECmd(api, temp=0, isQueued=1)

        # 動作
        # for i in range(0, 5):
        #     if i % 2 == 0:
        #         offset = 20
        #     else:
        #         offset = -20
        #     lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200 + offset, offset, offset, offset, isQueued=1)[0]

        currentPos = [200, 100, carryZ, 0]
        lastIndex = dType.SetPTPCmd(api, 2, currentPos[0], currentPos[1], currentPos[2], currentPos[3], isQueued=1)[0]

        lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=True, on=False, isQueued=True)[0]

        # 全てのコマンドが実行されるまで待機
        # while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:  # 現在のコマンド番号が最後のコマンド番号に達していない間
        #     dType.dSleep(100)

        currentPos = [200, 100, gripZ, 0]
        lastIndex = dType.SetPTPCmd(api, 2, currentPos[0], currentPos[1], currentPos[2], currentPos[3], isQueued=1)[0]

        lastIndex = dType.SetEndEffectorGripper(api, enableCtrl=True, on=True, isQueued=True)[0]

        currentPos[3]

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