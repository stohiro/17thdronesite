from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# 機体と接続します。
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)


# 機体をアームし、目標高度（aTargetAltitude）まで上昇します。
def arm_and_takeoff(aTargetAltitude):
    print("アーム確認")
    while not vehicle.is_armable:
        print(" 機体の初期化を待っています...")
        time.sleep(1)
        
    print("モーターをアーム中")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" アーミングを待っています...")
        time.sleep(1)

    print("離陸します！")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" 高度: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("目標高度に到達しました")
            break
        time.sleep(1)


# 指定された速度ベクトルに基づいて、機体を移動させます。
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    print(f"機体テスト中: vx={velocity_x}, vy={velocity_y}, vz={velocity_z}")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (使用しない)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # フレーム
        0b0000111111000111, # タイプマスク (速度のみ有効)
        0, 0, 0,            # x, y, z 位置 (使用しない)
        velocity_x, velocity_y, velocity_z, # x, y, z 速度( m/s )
        0, 0, 0,            # x, y, z 加速度 (使用しない)
        0, 0)               # yaw, yaw_rate (使用しない)
    
    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# 指定されたヨー角度に機体を移動させます。
def condition_yaw(heading, cw, relative=False):
    print(f"ヨー制御開始: {'相対角' if relative else '絶対角'} {heading}度")
    if relative:
        is_relative = 1 #進行方向に対する相対的なヨー
    else:
        is_relative = 0 #ヨーは絶対角度

    # command_long_encode()を使用してCONDITION_YAWコマンドを作成
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # ターゲットシステム、ターゲットコンポーネント
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #コマンド
        0, #確認
        heading,    # パラメータ1、ヨー角（度）
        0,          # パラメータ2、ヨー速度（度/秒）
        cw,          # パラメータ3、方向 -1 反時計回り、1 時計回り
        is_relative, # パラメータ4、相対オフセット 1、絶対角度 0
        0, 0, 0)    # パラメータ5〜7 未使用
    # 機体にコマンドを送信
    vehicle.send_mavlink(msg)


# 指定された方向に移動させテストします。
# 'forward'（前進）, 'backward'（後退）, 'left'（左）, 'right'（右）,
# 'up'（上昇）, 'down'（下降）, 'yaw_left'（左旋回）, 'yaw_right'（右旋回）
def move_vehicle(direction, duration=5):
    print(f"{direction}方向に移動開始")
    if direction == 'forward':
        send_ned_velocity(1, 0, 0, duration)
    elif direction == 'backward':
        send_ned_velocity(-1, 0, 0, duration)
    elif direction == 'left':
        send_ned_velocity(0, -1, 0, duration)
    elif direction == 'right':
        send_ned_velocity(0, 1, 0, duration)
    elif direction == 'up':
        send_ned_velocity(0, 0, -1, duration)
    elif direction == 'down':
        send_ned_velocity(0, 0, 1, duration)
    elif direction == 'yaw_left':
        print("左旋回開始")
        condition_yaw(90, -1, relative=True)
        time.sleep(duration)
    elif direction == 'yaw_right':
        print("右旋回開始")
        condition_yaw(180, 1, relative=True)
        time.sleep(duration)
    print(f"{direction}方向への移動完了")


def pre_flight_test():
    arm_and_takeoff(2)
    move_vehicle('forward', 3)
    move_vehicle('backward', 5)
    move_vehicle('forward', 2)
    move_vehicle('left', 3)
    move_vehicle('right', 6)
    move_vehicle('left', 2)
    time.sleep(3)
    move_vehicle('up', 5)
    move_vehicle('down', 4)
    move_vehicle('yaw_left', 5)
    move_vehicle('yaw_right', 5)

    
    print("飛行前テストが完了しました")
    
    print("LANDモードで着陸します")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        print("着陸を待機中...")
        time.sleep(1)

    print("着陸しました。ディスアームします。")
    vehicle.close()


if __name__ == "__main__":
    pre_flight_test()
