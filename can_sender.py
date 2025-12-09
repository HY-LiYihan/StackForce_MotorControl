import serial
import time
import math

# --- 1. 参数定义 (保持不变) ---
POS_MIN, POS_MAX = -25.12, 25.12
VEL_MIN, VEL_MAX = -65.0, 65.0
KP_MIN, KP_MAX   = 0.0, 500.0
KD_MIN, KD_MAX   = 0.0, 5.0
TOR_MIN, TOR_MAX = -18.0, 18.0

# --- 2. 转换逻辑 (保持不变) ---
def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if x > x_max: x = x_max
    if x < x_min: x = x_min
    value = (x - offset) * ((1 << bits) - 1) / span
    return int(value)

def create_mit_command(pos, vel, kp, kd, tor):
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16)
    vel_int = float_to_uint(vel, VEL_MIN, VEL_MAX, 12)
    kp_int  = float_to_uint(kp,  KP_MIN, KP_MAX,  12)
    kd_int  = float_to_uint(kd,  KD_MIN, KD_MAX,  12)
    tor_int = float_to_uint(tor, TOR_MIN, TOR_MAX, 12)

    payload = bytearray(8)
    payload[0] = (pos_int >> 8) & 0xFF
    payload[1] = pos_int & 0xFF
    payload[2] = (vel_int >> 4) & 0xFF
    payload[3] = ((vel_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    payload[4] = kp_int & 0xFF
    payload[5] = (kd_int >> 4) & 0xFF
    payload[6] = ((kd_int & 0xF) << 4) | ((tor_int >> 8) & 0xF)
    payload[7] = tor_int & 0xFF
    return bytes(payload)

def send_can_frame(ser, motor_id, data_bytes):
    can_id = 0x200 + motor_id
    head = b'\xAA\xC8'
    id_bytes = can_id.to_bytes(2, byteorder='little')
    tail = b'\x55'
    packet = head + id_bytes + data_bytes + tail
    ser.write(packet)

# --- 3. 主程序：双电机控制 ---
def main():
    COM_PORT = '/dev/ttyCH341USB0'
    
    # === 配置电机 ID ===
    MOTOR_ID_HORIZ = 0x03  # 水平电机 (左右)
    MOTOR_ID_VERT  = 0x04  # 垂直电机 (上下)

    # === 配置运动幅度 (弧度) ===
    # 180度 = 3.14 rad
    AMP_HORIZ = math.pi       
    # 60度 = 1.047 rad
    AMP_VERT  = math.pi / 3.0 
    
    try:
        ser = serial.Serial(COM_PORT, 2000000, timeout=0.05)
        print(f"端口 {COM_PORT} 已打开")

        # ==========================================
        # 1. 初始化阶段 (使能 & 设零)
        # ==========================================
        # --- 设零电机 3 ---
        print(f"2. 设零电机 ID: {MOTOR_ID_HORIZ} (当前位置设为0)...")
        zero_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        for _ in range(3):
            send_can_frame(ser, MOTOR_ID_HORIZ, zero_cmd)
            time.sleep(0.01)

        # --- 设零电机 4 ---
        print(f"   设零电机 ID: {MOTOR_ID_VERT} (当前位置设为0)...")
        for _ in range(3):
            send_can_frame(ser, MOTOR_ID_VERT, zero_cmd)
            time.sleep(0.01)
        
        # --- 使能电机 3 ---
        print(f"1. 使能电机 ID: {MOTOR_ID_HORIZ}...")
        enable_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        for _ in range(5):
            send_can_frame(ser, MOTOR_ID_HORIZ, enable_cmd)
            time.sleep(0.01)
            
        # --- 使能电机 4 ---
        print(f"   使能电机 ID: {MOTOR_ID_VERT}...")
        for _ in range(5):
            send_can_frame(ser, MOTOR_ID_VERT, enable_cmd)
            time.sleep(0.01)
        
        time.sleep(1) # 等待就绪



        # ==========================================
        # 2. 运动控制循环
        # ==========================================
        print("3. 开始双电机运动 (按 Ctrl+C 停止)...")
        print(f"   - 水平幅度: +/- 180度 (ID {MOTOR_ID_HORIZ})")
        print(f"   - 垂直幅度: +/- 60度  (ID {MOTOR_ID_VERT})")
        
        start_time = time.time()
        
        while True:
            t = time.time() - start_time
            
            # --- 轨迹计算 ---
            # 为了看起来协调，我们让两个轴同频率运动 (或者可以用 sin 和 cos 做圆周运动)
            # 这里使用 sin(t) 简单的同步摆动
            
            # 目标位置 3 (水平): +/- 180度
            target_pos_h = AMP_HORIZ * math.sin(t) * 4
            
            # 目标位置 4 (垂直): +/- 60度
            # 可以在这里加 math.cos(t) 让它走一个椭圆，或者保持 sin(t) 走斜线
            target_pos_v = AMP_VERT * math.cos(t) * 4

            # --- 发送 ID 3 (水平) ---
            mit_payload_h = create_mit_command(
                pos = target_pos_h, 
                vel = 0.0, 
                kp  = 2.0,  # 刚度
                kd  = 0.5,   # 阻尼
                tor = 0.0
            )
            send_can_frame(ser, MOTOR_ID_HORIZ, mit_payload_h)
            time.sleep(0.005)
            # --- 发送 ID 4 (垂直) ---
            # 注意：垂直电机需要对抗重力，如果负载重，可以稍微把 Kp 调大一点 (例如 15.0)
            mit_payload_v = create_mit_command(
                pos = target_pos_v, 
                vel = 0.0, 
                kp  = 1.0,  # 垂直轴稍微硬一点
                kd  = 0.1, 
                tor = 0.0
            )
            send_can_frame(ser, MOTOR_ID_VERT, mit_payload_v)
            print(f"时间: {t:.2f} s | 目标位置 - 水平: {target_pos_h:.2f} rad, 垂直: {target_pos_v:.2f} rad")
            
            # --- 读取反馈 & 延时 ---
            if ser.in_waiting:
                ser.read(ser.in_waiting)
                
            time.sleep(0.005) # 稍微增加一点延时给两帧数据发送

    except KeyboardInterrupt:
        print("\n停止控制...")
    finally:
        # 退出前发送失能 (两个电机都要停)
        print("4. 发送失能指令...")
        disable_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        
        for _ in range(5):
            send_can_frame(ser, MOTOR_ID_HORIZ, disable_cmd)
            send_can_frame(ser, MOTOR_ID_VERT,  disable_cmd)
            time.sleep(0.01)
            
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()