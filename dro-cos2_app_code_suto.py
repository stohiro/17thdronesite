from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
from math import sin, cos, sqrt, atan2, radians

# ドローンに接続する
# vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=60)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# ドローンをアームしてテイクオフする関数
def arm_and_takeoff(target_altitude):
    while not vehicle.is_armable:
        print("初期化中です")
        time.sleep(1)

    print("アームします")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("アームを待ってます")
        time.sleep(1)

    print("離陸！")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print("高度:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("目標高度に到達しました")
            break
        time.sleep(1)

#   2つのLocationGlobalRelativeオブジェクト間の距離をメートルで計算する関数
def get_distance_metres(aLocation1, aLocation2):
    R = 6378137.0  # 地球の半径 (メートル)
    lat1 = radians(aLocation1.lat)
    lon1 = radians(aLocation1.lon)
    lat2 = radians(aLocation2.lat)
    lon2 = radians(aLocation2.lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance

# ドローンをアームし、5mまで上昇
arm_and_takeoff(5)

# ウェイポイントの作成
waypoint_1 = LocationGlobalRelative(35.8787885, 140.3391342, 5)
waypoint_2 = LocationGlobalRelative(35.8789124, 140.3389813, 5)
waypoint_3 = LocationGlobalRelative(35.8786603, 140.3393242, 5)

# 1番目のミッション：ウェイポイント1に移動
print("ウェイポイント1に移動")
vehicle.simple_goto(waypoint_1)

# 移動中のロケーションメッセージ
while True:
    print("移動中:", vehicle.location.global_relative_frame)
    current_location = vehicle.location.global_relative_frame
    target_distance = get_distance_metres(current_location, waypoint_1)
    if target_distance < 1.0:  # 目標地点に到達したら次のウェイポイントに移動
        print("ウェイポイント1に到達しました")
        break
    time.sleep(0.4)

# ウェイポイント1到達まで待機
time.sleep(10)

# 2番目のミッション：ウェイポイント2に移動
print("ウェイポイント2に移動")
vehicle.simple_goto(waypoint_2)

# 移動中のロケーションメッセージ
while True:
    print("移動中:", vehicle.location.global_relative_frame)
    current_location = vehicle.location.global_relative_frame
    target_distance = get_distance_metres(current_location, waypoint_2)
    if target_distance < 1.0:  # 目標地点に到達したら次のウェイポイントに移動
        print("ウェイポイント2に到達しました")
        break
    time.sleep(0.4)

# ウェイポイント2到達まで待機
time.sleep(10)

# 3番目のミッション：ウェイポイント3に移動
print("ウェイポイント3に移動")
vehicle.simple_goto(waypoint_3)

# 移動中のロケーションメッセージ
while True:
    print("移動中:", vehicle.location.global_relative_frame)
    current_location = vehicle.location.global_relative_frame
    target_distance = get_distance_metres(current_location, waypoint_3)
    if target_distance < 1.0:  # 目標地点に到達したら次のウェイポイントに移動
        print("ウェイポイント3に到達しました")
        break
    time.sleep(0.4)

# ウェイポイント3到達まで待機
time.sleep(10)

# 4番目のミッション：ウェイポイント1に移動
print("ウェイポイント1に移動")
vehicle.simple_goto(waypoint_1)

# 移動中のロケーションメッセージ
while True:
    print("移動中:", vehicle.location.global_relative_frame)
    current_location = vehicle.location.global_relative_frame
    target_distance = get_distance_metres(current_location, waypoint_1)
    if target_distance < 1.0:  # 目標地点に到達したら次のウェイポイントに移動
        print("ウェイポイント1に到達しました")
        break
    time.sleep(0.4)

# ウェイポイント1到達まで待機
time.sleep(10)


# 5番目のミッション：左回転90度、10秒待機、右回転180度で元に戻る
def condition_yaw(degrees, cw, relative):
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        degrees,
        0,
        cw,
        is_relative,
        0, 0, 0)
    vehicle.send_mavlink(msg)

print("左回転90度")
condition_yaw(75, -1, True)
time.sleep(10)

print("右回転180度")
condition_yaw(180, 1, True)
time.sleep(10)

# 6番目のミッション：RTLモードに変更し、HOMEに戻り着陸
print("SMART_RTLモードに変更")
vehicle.mode = VehicleMode("SMART_RTL")

# RTLと着陸が完了するまで待機
while vehicle.armed:
    print("帰還中...")
    time.sleep(1)

print("着陸完了、ディスアームします")

# 接続を閉じる
vehicle.close()
