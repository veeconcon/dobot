import cv2
import datetime
import io
import sys
import time
import numpy as np
import pandas as pd
import DobotDllType as dType
import ex_imp1_crack_detect

### 各種初期設定 ###

# DobotCOM番号
COM_dobot1 = "COM3"
COM_dobot2 = "COM8"

# Dobotピン番号
PIN1_pipette = 10
PIN1_heater = 11

# カメラ設定
NUM_hozan = 0  # カメラ番号
CAP_frame_width = 1024  # 撮影解像度, 横
CAP_frame_height = 768  # 撮影解像度, 縦
CAP_brightness = 128  # 明るさ
CAP_whitebalance = 4650  # ホワイトバランス, 使えない
THRESHOLD = 55  # 二値化しきい値, ひび割れ分離用
THRESHOLD_con = 150  # 二値化しきい値, 輪郭検出用
sec_intv_cap = 60  # 撮影間隔

# 実験設定
n_trial = 9  # 31 #試行回数, センサスロット数は31
n_skip = [13, 24]  # 使わないセンサ番号, 1スタート
steps = 5  # 温度区分数
settemp_min = 30  # 設定温度下限
settemp_max = 100  # 設定温度上限
sec_pre = 30  # 予熱時間
sec_intv = 120  # 温度制御の時間間隔, 本番は120
sec_total = sec_pre + sec_intv * steps  # 滴下から加熱終了までの総時間
sec_cool = 600  # サンプル移行時の冷却待ち, 本番は600
temp_max = 200 * 3.3 / 5  # 132, 温調器設定上の温度上限
temp_pre = 30  # 予熱温度

crd1_home = [170, 100, 120, 0]
crd1_wash = [196, 131, -68]
crd1_suck = [260, 138, -68]
z1_move = 100  # 移動時高さ
z1_pool = 45  # 容器蓋高さ
z1_wash = 20  # 洗浄時の吐出高さ

crd2_home = [100, 180, 135, 0]
z2_move = 0  # 移動時高さ

crd_err = 1  # 静止判断の閾値


### 関数定義 ###

# 許可待ち関数
def WaitPrm(str):
    while True:
        val = input(str + " y/n : ")
        if val == "y":
            return 0
        elif val == "n":
            sys.exit()


# ホーミング関数
def Homing(api, str):
    while True:
        val = input("Execute homing " + str + " ? y/n : ")
        if val == "y":
            dType.SetHOMECmd(api, 0, 1)
            return 0
        elif val == "n":
            return 0


# Dobotが指定座標に指定時間留まるまで待つ関数
def WaitIf(api, crd, msec_check):
    while True:
        if all(np.array(dType.GetPose(api)[:3]) - np.array(crd) < crd_err):
            time.sleep(msec_check / 1000)
            if all(np.array(dType.GetPose(api)[:3]) - np.array(crd) < crd_err):
                return time.time()

# Dobot接続関数
CON_STR = {dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
           dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
           dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}


def DobotConnect(api, com):
    state = dType.ConnectDobot(api, com, 115200)[0]
    if state != 0:
        print("Connect status:", CON_STR[state])
        print("Search result:", dType.SearchDobot(api))
        dType.DisconnectDobot(api)
        sys.exit()


### 実験実行 ###

if __name__ == '__main__':

    ## 温度条件読込, 表示 ##
    temp_params = pd.read_csv(filename_params, index_col=0).values[:n_trial, :]
    np.set_printoptions(precision=3, linewidth=100, suppress=True)
    print("({} x {}) Params:".format(n_trial, steps))
    print(temp_params)

    ## 結果格納用配列の用意 ##
    result = np.append(temp_params, np.full((n_trial, 3), -1), axis=1).astype(np.float64)
    columns = ["temp1", "temp2", "temp3", "temp4", "temp5", "pixel_ink", "pixel_crack", "crack_rate"]
    filename = "result/ex02_result_ini_" + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + ".csv"

    ## カメラ選択チェック ##
    capture_check = cv2.VideoCapture(NUM_hozan, cv2.CAP_DSHOW)
    capture_check.set(cv2.CAP_PROP_BRIGHTNESS, CAP_brightness)  # 明るさ
    print("Press 'q' to close the window")
    while True:
        ret_check, frame_check = capture_check.read()
        frame_check = cv2.resize(frame_check, (800, 600))
        cv2.imshow("capture_check", frame_check)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            capture_check.release()
            break
    WaitPrm("IS this camera OK?")

    ## Dobot初期設定 ##
    # 通信クラスのインスタンス化, 標準出力に流さない
    sys.stdout = io.StringIO()
    api1 = dType.load()
    api2 = dType.load()
    sys.stdout = sys.__stdout__

    # Dobot1
    DobotConnect(api1, COM_dobot1)
    dType.SetHOMEParams(api1, crd1_home[0], crd1_home[1], crd1_home[2], crd1_home[3], 1)
    dType.SetPTPJointParams(api1, 200, 200, 200, 200, 200, 200, 200, 200, 1)
    dType.SetPTPCoordinateParams(api1, 200, 200, 200, 200, 1)
    dType.SetPTPCommonParams(api1, 100, 100, 1)
    dType.SetIOMultiplexing(api1, PIN1_pipette, 1, 1)
    dType.SetIOMultiplexing(api1, PIN1_heater, 2, 1)
    Homing(api1, "dobot1")
    SetTemp(api1, temp_pre)
    dType.DisconnectDobot(api1)

    # Dobot2
    DobotConnect(api2, COM_dobot2)
    dType.SetHOMEParams(api2, crd2_home[0], crd2_home[1], crd2_home[2], crd2_home[3], 1)
    dType.SetPTPJointParams(api2, 200, 200, 200, 200, 200, 200, 200, 200, 1)
    dType.SetPTPCoordinateParams(api2, 200, 200, 200, 200, 1)
    dType.SetPTPCommonParams(api2, 100, 100, 1)
    Homing(api2, "dobot2")
    dType.DisconnectDobot(api2)

    ## 座標リスト読み込み ##
    crd1_spit = pd.read_csv("coordinate_pipette.csv", index_col=0, skiprows=n_skip).values
    crd2_cont = pd.read_csv("coordinate_contact.csv", index_col=0, skiprows=n_skip).values
    crd2_cam = pd.read_csv("coordinate_camera.csv", index_col=0, skiprows=n_skip).values

    ## 実行確認, 実行 ##
    WaitPrm("Go to the next step?")
    for i in range(n_trial):

        # Dobot1滴下
        DobotConnect(api1, COM_dobot1)
        Pipetting(api1, crd1_spit[i, :])
        sec_spit = WaitIf(api1, crd1_spit[i, :], 500)
        dType.DisconnectDobot(api1)

        # Dobot1洗浄
        DobotConnect(api1, COM_dobot1)
        Washing(api1)
        WaitIf(api1, crd1_wash, 10)
        dType.DisconnectDobot(api1)

        # Dobot2移動
        DobotConnect(api2, COM_dobot2)
        CapturingPrep(api2, crd2_cam[i, :])
        dType.DisconnectDobot(api2)

        # 予熱時間終了2秒前まで待機
        if time.time() - sec_spit > sec_pre - 2:
            print("Mesurement starting time is over!")
            sys.exit()
        while time.time() - sec_spit < sec_pre - 2:
            continue

        # Dobot1温度制御コマンド送信
        DobotConnect(api1, COM_dobot1)
        dType.SetWAITCmd(api1, 2000, 1)
        for temp in result[i, :steps]:
            SetTemp(api1, temp)
            dType.SetWAITCmd(api1, sec_intv * 1000, 1)
        SetTemp(api1, temp_pre)
        print("TempParam: " + str(result[i, :steps]))
        dType.DisconnectDobot(api1)

        # Dobot2撮影, 加熱開始から1分ごとに
        i_cap = 0
        while time.time() - sec_spit <= sec_total:
            if i_cap > 0:
                filename_cap = "photo/ex02_{:02d}_{:03d}_".format(i + 1, int(time.time() - sec_spit)) \
                               + datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + ".png"
                Capturing(filename_cap)
            while int(time.time() - (sec_spit + sec_pre)) <= i_cap * sec_intv_cap:
                continue
            i_cap += 1
        # 乾燥後に1枚撮影
        filename_cap = "photo/ex02_{:02d}_fin_".format(i + 1) \
                       + datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + ".png"
        Capturing(filename_cap)

        # Dobot2ホームに復帰
        DobotConnect(api2, COM_dobot2)
        CapturingFin(api2, crd2_cam[i, :])
        dType.DisconnectDobot(api2)

        ## 次サンプルに移るまでの処理 ##
        # ひび割れ率計算
        crack_rate = ex_imp1_crack_detect.main(filename_cap, threshold=THRESHOLD, threshold_con=THRESHOLD_con)
        result[i, steps:] = crack_rate
        print(result)

        # 結果保存
        df = pd.DataFrame(data=result[:i + 1], columns=columns, dtype='float')
        df.index = np.arange(1, len(df) + 1)  # indexは1から
        df.to_csv(filename)
        print("Trial No.{} has finished.".format(i + 1))

        # 冷却待ち
        time.sleep(sec_cool)
