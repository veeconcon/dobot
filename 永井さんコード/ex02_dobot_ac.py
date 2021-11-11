import cv2
import datetime
import io
import itertools
import sys
import time
import numpy as np
import pandas as pd
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import DobotDllType as dType
import ex_imp1_crack_detect


### 各種初期設定 ###

#初期データファイル
filename_ini = "result/ex02_result_2021-02-11_19-31-58.csv"

#DobotCOM番号
COM_dobot1 = "COM3"
COM_dobot2 = "COM8"

#Dobotピン番号
PIN1_pipette = 10
PIN1_heater = 11

#カメラ設定
NUM_hozan = 0 #カメラ番号
CAP_frame_width = 1024 #撮影解像度, 横
CAP_frame_height = 768 #撮影解像度, 縦
CAP_brightness = 128 #明るさ
THRESHOLD = 55 #二値化しきい値, ひび割れ分離用
THRESHOLD_con = 150 #二値化しきい値, 輪郭検出用
sec_intv_cap = 60 #撮影間隔

#実験設定
n_trial = 20 #31 #試行回数, センサスロット数は31
n_initial = 25 #30 #初期データの数
n_skip = [1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 24] #使わないセンサ番号, 1スタート
steps = 5 #温度区分数
settemp_min = 30 #設定温度下限
settemp_max = 100 #設定温度上限
sec_pre = 30 #予熱時間
sec_intv = 120 #温度制御の時間間隔, 本番は120
sec_total = sec_pre + sec_intv * steps #滴下から加熱終了までの総時間
sec_cool = 600 #サンプル移行時の冷却待ち, 本番は600
temp_max = 200 * 3.3/5 #132, 温調器設定上の温度上限
temp_pre = 30 #予熱温度

crd1_home = [170, 100, 120, 0]
crd1_wash = [196, 131, -68]
crd1_suck = [260, 138, -68]
z1_move = 100 #移動時高さ
z1_pool = 45 #容器蓋高さ
z1_wash = 20 #洗浄時の吐出高さ

crd2_home = [100, 180, 135, 0]
z2_move = 0 #移動時高さ

crd_err = 1 #静止判断の閾値

#探索する温度条件の全リスト
temp_list = np.array(list(itertools.product(range(int(settemp_min/10), int(settemp_max/10)+1),
                                            repeat=steps))) * 10
C_ucb = 0.5 #UCBのパラメータc
kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2)) #ガウス過程回帰のカーネル


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

#温度指示関数
def SetTemp(api, temp):
    if temp>temp_max or temp<0:
        print("Input [0:"+str(temp_max)+"] temp")
        duty = 0
    else:
        duty = min(temp/temp_max*100+0.1, 100) #温調器との調整の結果の+0.1
    dType.SetIOPWM(api, PIN1_heater,  1e5, duty, 1)
    return 0

#Dobotが指定座標に指定時間留まるまで待つ関数
def WaitIf(api, crd, msec_check):
    while True:
        if all(np.array(dType.GetPose(api)[:3])-np.array(crd)<crd_err):
            time.sleep(msec_check/1000)
            if all(np.array(dType.GetPose(api)[:3])-np.array(crd)<crd_err):
                return time.time()

#ピペットの電源on関数, 既にonの場合混合処理
def PipetteOn(api):
    dType.SetIODO(api, PIN1_pipette, 1, 1)
    dType.SetWAITCmd(api, 3000, 1)
    dType.SetIODO(api, PIN1_pipette, 0, 1)
    return 0

#ピペット吸込or吐出関数
def PipetteSwitch(api):
    dType.SetIODO(api, PIN1_pipette, 1, 1)
    dType.SetWAITCmd(api, 10, 1)
    dType.SetIODO(api, PIN1_pipette, 0, 1)
    return 0

#吸込->滴下->洗浄前半の関数
def Pipetting(api, crd):
    PipetteOn(api)
    dType.SetPTPCmd(api, 2, crd1_home[0], crd1_home[1], crd1_home[2], crd1_home[3], 1)
    dType.SetWAITCmd(api, 100, 1)
    #吸込
    dType.SetPTPCmd(api, 2, crd1_suck[0], crd1_suck[1], z1_move, 0, 1)
    dType.SetWAITCmd(api, 100, 1)
    dType.SetPTPCmd(api, 2, crd1_suck[0], crd1_suck[1], crd1_suck[2]+z1_pool, 0, 1)
    dType.SetPTPCommonParams(api1, 20, 20, 1)
    dType.SetPTPCmd(api, 2, crd1_suck[0], crd1_suck[1], crd1_suck[2], 0, 1)
    dType.SetWAITCmd(api, 500, 1)
    PipetteSwitch(api)
    dType.SetWAITCmd(api, 1000, 1)
    dType.SetPTPCmd(api, 2, crd1_suck[0], crd1_suck[1], crd1_suck[2]+z1_pool, 0, 1)
    dType.SetPTPCommonParams(api1, 100, 100, 1)
    dType.SetPTPCmd(api, 2, crd1_suck[0], crd1_suck[1], z1_move, 0, 1)
    dType.SetWAITCmd(api, 100, 1)
    #滴下
    dType.SetPTPCmd(api, 2, crd[0], crd[1], z1_move, 0, 1)
    dType.SetWAITCmd(api, 100, 1)
    dType.SetPTPCmd(api, 2, crd[0], crd[1], crd[2]+20, 0, 1)
    dType.SetPTPCommonParams(api, 20, 20, 1)
    dType.SetPTPCmd(api, 2, crd[0], crd[1], crd[2], 0, 1)
    dType.SetWAITCmd(api, 500, 1)
    PipetteSwitch(api)
    dType.SetWAITCmd(api, 2000, 1)
    dType.SetPTPCmd(api, 2, crd[0], crd[1], crd[2]+20, 0, 1)
    dType.SetPTPCommonParams(api, 100, 100, 1)
    dType.SetPTPCmd(api, 2, crd[0], crd[1], z1_move, 0, 1)
    dType.SetWAITCmd(api, 100, 1)
    #洗浄位置に移動
    dType.SetPTPCmd(api, 2, crd1_wash[0], crd1_wash[1], z1_move, 0, 1)
    return 0

#洗浄後半の関数
#チューニング余地あり, 加熱開始までにホーム復帰できるよう注意
def Washing(api):
    dType.SetPTPCmd(api, 2, crd1_wash[0], crd1_wash[1], crd1_wash[2]+z1_pool, 0, 1)
    dType.SetPTPCommonParams(api, 20, 20, 1)
    for i in range(2):
        dType.SetPTPCmd(api, 2, crd1_wash[0], crd1_wash[1], crd1_wash[2], 0, 1)
        #dType.SetWAITCmd(api, 500, 1)
        dType.SetPTPCommonParams(api, 100, 100, 1)
        PipetteSwitch(api)
        dType.SetWAITCmd(api, 1000, 1)
        dType.SetPTPCmd(api, 2, crd1_wash[0], crd1_wash[1], crd1_wash[2]+z1_wash, 0, 1)
        dType.SetWAITCmd(api, 500, 1)
        PipetteSwitch(api)
        dType.SetWAITCmd(api, 3500, 1)
    dType.SetPTPCommonParams(api, 20, 20, 1)
    dType.SetPTPCmd(api, 2, crd1_wash[0], crd1_wash[1], crd1_wash[2]+z1_pool, 0, 1)
    dType.SetPTPCommonParams(api, 100, 100, 1)
    dType.SetPTPCmd(api, 2, crd1_wash[0], crd1_wash[1], z1_move, 0, 1)
    dType.SetWAITCmd(api, 100, 1)
    #ホームポジション復帰
    dType.SetPTPCmd(api, 2, crd1_home[0], crd1_home[1], crd1_home[2], crd1_home[3], 1)
    return 0

#画像撮影保存の関数
def Capturing(filename):
    capture = cv2.VideoCapture(NUM_hozan, cv2.CAP_DSHOW) #第2引数はwarning防止
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_frame_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_frame_height)
    capture.set(cv2.CAP_PROP_BRIGHTNESS, CAP_brightness) #明るさ
    cap_start = time.time()
    #while文中でcaptureしないと上手く行かない？
    while True:
        ret, frame = capture.read()
        if time.time()-cap_start>=1:
            cv2.imwrite(filename, frame)
            capture.release()
            break

#Dobot2撮影準備の関数
def CapturingPrep(api, crd_cam):
    dType.SetPTPCmd(api, 1, crd2_home[0], crd2_home[1], crd2_home[2], crd2_home[3], 1)
    dType.SetWAITCmd(api, 100, 1)
    dType.SetPTPCmd(api, 1, crd_cam[0], crd_cam[1], crd_cam[2], crd2_home[3], 1)
    return 0

#Dobot2撮影終了関数
def CapturingFin(api, crd_cam):
    dType.SetPTPCmd(api, 1, crd2_home[0], crd2_home[1], crd2_home[2], crd2_home[3], 1)
    return 0

#Dobot接続関数
CON_STR={dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
         dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
         dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
def DobotConnect(api, com):
    state = dType.ConnectDobot(api, com, 115200)[0]
    if state!=0:
        print("Connect status:", CON_STR[state])
        print("Search result:", dType.SearchDobot(api))
        dType.DisconnectDobot(api)
        sys.exit()

#学習の獲得関数, UCB
def acq_ucb(mean, sigma, c):
    return np.argmax(- mean + sigma * c)


### 実験実行 ###

if __name__=='__main__':

    ## 初期データ読込, 表示, 保存の準備 ##
    result = pd.read_csv(filename_ini, index_col=0).values[:n_initial, :]
    columns = ["temp1", "temp2", "temp3", "temp4", "temp5", "pixel_ink", "pixel_crack", "crack_rate"]
    filename = "result/ex02_result_" + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + ".csv"
    np.set_printoptions(precision=3, linewidth=100, suppress=True)
    print("Initial dataset:")
    print(result)

    """
    ## 温度条件読込, 表示 ##
    temp_params = pd.read_cvs("ex02_tempparams.csv")
    np.set_printoptions(precision=3, linewidth=100, suppress=True)
    print("({} x {}) Params:".format(n_trial, steps))
    print(temp_params)
    """

    """
    ## 結果格納用配列の用意 ##
    result = np.append(temp_params, np.full((n_trial, 3), -1), axis=1).astype(np.float64)
    columns = ["temp1", "temp2", "temp3", "temp4", "temp5", "pixel_ink", "pixel_crack", "pixel_rate"]
    filename = "result/ex02_result_" + datetime.datetime.now().strftime('%y-%m-%d_%h-%m-%s') + ".csv"
    """

    ## カメラ選択チェック ##
    capture_check = cv2.VideoCapture(NUM_hozan, cv2.CAP_DSHOW)
    capture_check.set(cv2.CAP_PROP_BRIGHTNESS, CAP_brightness) #明るさ
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
    #通信クラスのインスタンス化, 標準出力に流さない
    sys.stdout = io.StringIO()
    api1 = dType.load()
    api2 = dType.load()
    sys.stdout = sys.__stdout__

    #Dobot1
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

    #Dobot2
    DobotConnect(api2, COM_dobot2)
    dType.SetHOMEParams(api2, crd2_home[0], crd2_home[1], crd2_home[2], crd2_home[3], 1)
    dType.SetPTPJointParams(api2, 200, 200, 200, 200, 200, 200, 200, 200, 1)
    dType.SetPTPCoordinateParams(api2, 200, 200, 200, 200, 1)
    dType. SetPTPCommonParams(api2, 100, 100, 1)
    Homing(api2, "dobot2")
    dType.DisconnectDobot(api2)

    ## 座標リスト読み込み ##
    crd1_spit = pd.read_csv("coordinate_pipette.csv", index_col=0, skiprows=n_skip).values
    crd2_cont = pd.read_csv("coordinate_contact.csv", index_col=0, skiprows=n_skip).values
    crd2_cam = pd.read_csv("coordinate_camera.csv", index_col=0, skiprows=n_skip).values

    ## 実行確認, 実行 ##
    WaitPrm("Go to the next step?")
    for i in range(n_trial):
        #実験インデックスの定義, 初期データ分をパス
        i_result = i + n_initial


        ## 学習 ##
        #ガウス過程回帰によるモデル化, x:温度*5, y:ひび割れ率
        gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
        gp.fit(result[:, :steps], result[:, steps+2])
        mean, sigma = gp.predict(temp_list, return_std=True)

        #次の実験条件の決定
        temp_next = temp_list[acq_ucb(mean, sigma, C_ucb)]
        result = np.append(result, np.full((1, steps+3), -1), axis=0)
        result[i_result, :steps] = temp_next


        ## 実験実行 ##
        #Dobot1滴下
        DobotConnect(api1, COM_dobot1)
        Pipetting(api1, crd1_spit[i, :])
        sec_spit = WaitIf(api1, crd1_spit[i, :], 500)
        dType.DisconnectDobot(api1)

        #Dobot1洗浄
        DobotConnect(api1, COM_dobot1)
        Washing(api1)
        WaitIf(api1, crd1_wash, 10)
        dType.DisconnectDobot(api1)

        #Dobot2移動
        DobotConnect(api2, COM_dobot2)
        CapturingPrep(api2, crd2_cam[i, :])
        dType.DisconnectDobot(api2)

        #予熱時間終了2秒前まで待機
        if time.time()-sec_spit>sec_pre-2:
            print("Mesurement starting time is over!")
            sys.exit()
        while time.time()-sec_spit<sec_pre-2:
            continue

        #Dobot1温度制御コマンド送信
        DobotConnect(api1, COM_dobot1)
        dType.SetWAITCmd(api1, 2000, 1)
        for temp in result[i_result, :steps]:
            SetTemp(api1, temp)
            dType.SetWAITCmd(api1, sec_intv*1000, 1)
        SetTemp(api1, temp_pre)
        print("TempParam: " + str(result[i_result, :steps]))
        dType.DisconnectDobot(api1)

        #Dobot2撮影, 加熱開始から1分ごとに
        i_cap = 0
        while time.time() - sec_spit <= sec_total:
            if i_cap>0:
                filename_cap = "photo/ex02_{:02d}_{:03d}_".format(i_result+1, int(time.time()-sec_spit)) \
                                + datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + ".png"
                Capturing(filename_cap)
            while int(time.time()-(sec_spit+sec_pre)) <= i_cap*sec_intv_cap:
                continue
            i_cap += 1
        #乾燥後に1枚撮影
        filename_cap = "photo/ex02_{:02d}_fin_".format(i_result+1) \
                        + datetime.datetime.now().strftime('%m-%d_%H-%M-%S') + ".png"
        Capturing(filename_cap)

        #Dobot2ホームに復帰
        DobotConnect(api2, COM_dobot2)
        CapturingFin(api2, crd2_cam[i, :])
        dType.DisconnectDobot(api2)


        ## 次サンプルに移るまでの処理 ##
        #ひび割れ率計算
        crack_rate = ex_imp1_crack_detect.main(filename_cap, threshold=THRESHOLD, threshold_con=THRESHOLD_con)
        result[i_result, steps:] = crack_rate
        print(result)

        #結果保存
        df = pd.DataFrame(data=result[:i_result+1], columns=columns, dtype='float')
        df.index = np.arange(1, len(df)+1) #indexは1から
        df.to_csv(filename)
        print("Trial No.{} has finished.".format(i_result+1))

        #検知面積が極端に小さい時は滴下失敗とみなして中断
        if crack_rate[0] + crack_rate[1] < 50000: #普通8~10万くらいになる
            print("Dropping was failed!!")
            sys.exit()

        #冷却待ち
        time.sleep(sec_cool)
