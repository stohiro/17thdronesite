from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import sqrt

# 距離を計算する関数
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# 目的地に到達したかどうかをチェックする関数
def is_at_location(targetLocation, currentLocation, threshold=1.0):
    distance = get_distance_metres(targetLocation, currentLocation)
    return distance <= threshold

# コプターの接続
copter = connect('tcp:127.0.0.1:5762', wait_ready=True)

def arm_and_takeoff_copter(target_altitude):
    while not copter.is_armable:
        print("初期化中です")
        time.sleep(1)
    
    copter.mode = VehicleMode("GUIDED")
    copter.armed = True

    while not copter.armed:
        print("アームを待ってます")
        time.sleep(1)
    
    copter.simple_takeoff(target_altitude)
    while True:
        if copter.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("目標高度に到達しました")
            break
        time.sleep(1)

# 離陸
arm_and_takeoff_copter(10)  # 10メートルに離陸

# メインポートから隣接ポートへ移動
target_location = LocationGlobalRelative(35.867003, 140.305987, 10)
copter.simple_goto(target_location)

# 目的地に到達するまで待機
while not is_at_location(target_location, copter.location.global_relative_frame):
    print(f"移動中: {copter.location.global_relative_frame}")
    time.sleep(1)

# 荷物の載せ替え中
time.sleep(10)

# 隣接ポートからメインポートへ戻る
target_location = LocationGlobalRelative(35.878275, 140.338069, 10)
copter.simple_goto(target_location)

# 目的地に到達するまで待機
while not is_at_location(target_location, copter.location.global_relative_frame):
    print(f"移動中: {copter.location.global_relative_frame}")
    time.sleep(1)

# 着陸
copter.mode = VehicleMode("LAND")
copter.close()
