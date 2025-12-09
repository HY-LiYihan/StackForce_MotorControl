import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
import time
import math
import threading

# 假设你的消息包名，如果不同请修改
try:
    from auto_aim_interfaces.msg import Armors
except ImportError:
    print("⚠️ 无法导入 auto_aim_interfaces，请确认环境")
    class Armors: pass # 占位符防止报错

# ==========================================
# 1. 协议与辅助函数 (与你提供的保持一致)
# ==========================================
POS_MIN, POS_MAX = -25.12, 25.12
VEL_MIN, VEL_MAX = -65.0, 65.0
KP_MIN, KP_MAX   = 0.0, 500.0
KD_MIN, KD_MAX   = 0.0, 5.0
TOR_MIN, TOR_MAX = -18.0, 18.0

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

# ==========================================
# 2. PID 控制器类
# ==========================================
class PIDController:
    def __init__(self, kp, ki, kd, max_out=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        if dt <= 0: return 0.0
        
        # P
        p_out = self.kp * error
        
        # I
        self.integral += error * dt
        # 积分限幅 (防止过冲)
        if self.integral > 1.0: self.integral = 1.0
        if self.integral < -1.0: self.integral = -1.0
        i_out = self.ki * self.integral

        # D
        derivative = (error - self.prev_error) / dt
        d_out = self.kd * derivative
        
        self.prev_error = error
        output = p_out + i_out + d_out
        
        if self.max_out:
            output = max(min(output, self.max_out), -self.max_out)
            
        return output

# ==========================================
# 3. ROS 2 节点与主逻辑
# ==========================================
class GimbalAimingNode(Node):
    def __init__(self):
        super().__init__('gimbal_aiming_node')
        
        # --- 配置串口 ---
        self.serial_port = '/dev/ttyCH341USB0'
        try:
            self.ser = serial.Serial(self.serial_port, 2000000, timeout=0.005)
            self.get_logger().info(f"串口 {self.serial_port} 打开成功")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            return

        # --- 电机配置 ---
        self.ID_YAW = 0x03    # 水平
        self.ID_PITCH = 0x04  # 垂直
        self.GEAR_RATIO = 4.0 # 关键系数：关节1度 = 电机4度

        # --- 状态变量 ---
        # 记录当前的累积目标角度（关节角度，单位：rad）
        # 初始假设在 0 位置
        self.target_yaw_angle = 0.0 
        self.target_pitch_angle = 0.0
        
        # 视觉误差缓存
        self.latest_yaw_err = 0.0
        self.latest_pitch_err = 0.0
        self.last_msg_time = 0.0
        self.target_visible = False

        # --- PID 设置 (需要根据实际调优) ---
        # 视觉PID: 输入是角度误差，输出是角度增量
        self.pid_yaw = PIDController(kp=0.3, ki=0.0, kd=0.01, max_out=0.1) 
        self.pid_pitch = PIDController(kp=0.2, ki=0.0, kd=0.01, max_out=0.1)

        # --- 订阅视觉话题 ---
        self.sub = self.create_subscription(
            Armors,
            '/detector/armors',
            self.listener_callback,
            qos_profile_sensor_data
        )

        # --- 初始化电机 ---
        self.init_motors()

        # --- 创建控制定时器 (100Hz) ---
        # 这里的频率决定了给电机发送指令的频率，建议 100Hz - 500Hz
        self.timer = self.create_timer(0.01, self.control_loop)
        self.dt = 0.01

    def init_motors(self):
        """使能电机"""
        self.get_logger().info("正在使能电机...")
        enable_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        for _ in range(5):
            send_can_frame(self.ser, self.ID_YAW, enable_cmd)
            send_can_frame(self.ser, self.ID_PITCH, enable_cmd)
            time.sleep(0.01)
        self.get_logger().info("电机已使能")

    def stop_motors(self):
        """失能电机"""
        self.get_logger().info("正在停止电机...")
        disable_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        for _ in range(5):
            if self.ser.is_open:
                send_can_frame(self.ser, self.ID_YAW, disable_cmd)
                send_can_frame(self.ser, self.ID_PITCH, disable_cmd)
            time.sleep(0.01)

    def listener_callback(self, msg):
        """处理视觉数据，计算角度误差"""
        if not msg.armors:
            self.target_visible = False
            return

        self.target_visible = True
        self.last_msg_time = time.time()
        
        # 简单的策略：只取第一个装甲板 (可以改为取最近的)
        armor = msg.armors[0]
        x = armor.pose.position.x
        y = armor.pose.position.y
        z = armor.pose.position.z

        # 计算误差角度 (rad)
        # x 正向为右 -> 需要向右转 -> Yaw 增加/减少取决于安装方向
        # 假设：云台向左转为正(CCW)，目标在右(x>0) -> 误差为负
        # 这里使用 atan2(x, z) 得到目标相对于相机的角度
        yaw_err = math.atan2(x, z)
        pitch_err = math.atan2(y, z)

        # 更新全局误差变量，供控制循环使用
        self.latest_yaw_err = yaw_err
        self.latest_pitch_err = pitch_err

    def control_loop(self):
        """主控制循环 (定时器触发)"""
        
        # 1. 检查目标是否丢失 (超过 0.2s 没收到数据)
        if time.time() - self.last_msg_time > 0.2:
            self.target_visible = False
        
        # 2. 如果有目标，运行 PID 计算增量
        if self.target_visible:
            # 计算 PID 输出 (需要转动的角度增量)
            # 注意符号方向：如果发现越瞄越偏，请将 '+=' 改为 '-='
            d_yaw = self.pid_yaw.update(self.latest_yaw_err, self.dt)
            d_pitch = self.pid_pitch.update(self.latest_pitch_err, self.dt)
            
            # 更新目标关节角度
            # 我们希望 "当前角度 + 误差 = 目标角度"
            # 这里采用了 增量式 逼近： 每一帧向目标方向挪动一点点
            self.target_yaw_angle -= d_yaw   # 符号需根据实际电机方向调试
            self.target_pitch_angle += d_pitch 
        else:
            # 目标丢失时：保持当前位置，或者可以写逻辑归中
            pass

        # 3. 软限位保护 (防止绕线或撞击)
        # 假设 Yaw 限位 +/- 90度, Pitch +/- 30度
        self.target_yaw_angle = max(min(self.target_yaw_angle, 1.5), -1.5)
        self.target_pitch_angle = max(min(self.target_pitch_angle, 0.6), -0.6)

        # 4. 生成电机指令 (应用 4:1 系数)
        # 关节角度 * 4.0 = 电机轴角度
        motor_pos_yaw = self.target_yaw_angle * self.GEAR_RATIO
        motor_pos_pitch = self.target_pitch_angle * self.GEAR_RATIO
        
        # 5. 发送指令
        # 这里的 kp, kd 是电机内部刚度，用于维持目标位置
        # 瞄准时为了响应快，刚度 Kp 可以适当给大一点 (例如 8.0 - 15.0)
        # 阻尼 Kd 用于消除震荡
        
        # --- 发送 Yaw ---
        cmd_yaw = create_mit_command(
            pos = motor_pos_yaw,
            vel = 0.0,
            kp  = 2.0, 
            kd  = 0.5,
            tor = 0.0
        )
        send_can_frame(self.ser, self.ID_YAW, cmd_yaw)
        
        # --- 发送 Pitch (需要稍微大的力对抗重力) ---
        cmd_pitch = create_mit_command(
            pos = motor_pos_pitch,
            vel = 0.0,
            kp  = 1.0,
            kd  = 0.1,
            tor = 0.0 # 如果有重力补偿，可以在这里加前馈力矩
        )
        send_can_frame(self.ser, self.ID_PITCH, cmd_pitch)

        # 读取反馈 (清空缓冲区防止积压)
        if self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)

def main(args=None):
    rclpy.init(args=args)
    node = GimbalAimingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()