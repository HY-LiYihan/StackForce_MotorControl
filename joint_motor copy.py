import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
import time
import math
import threading

# 【新增】导入 TF 广播器和消息类型
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# 假设你的消息包名，如果不同请修改
try:
    from auto_aim_interfaces.msg import Armors
except ImportError:
    # print("⚠️ 无法导入 auto_aim_interfaces，请确认环境")
    class Armors: pass 

# ==========================================
# 0. 【新增】辅助函数：欧拉角转四元数
# ==========================================
def euler_to_quaternion(yaw, pitch, roll):
    """
    将欧拉角 (弧度) 转换为 ROS 四元数 (x, y, z, w)
    顺序: Z (Yaw) -> Y (Pitch) -> X (Roll)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

# ==========================================
# 1. 协议与辅助函数
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
        # 积分限幅
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
            # 如果没有串口，这行 return 会导致后面代码不执行，这里为了测试TF可以注释掉 return
            # return 

        # --- 电机配置 ---
        self.ID_YAW = 0x03    
        self.ID_PITCH = 0x04  
        self.GEAR_RATIO = 4.0 

        # --- 状态变量 ---
        self.target_yaw_angle = 0.0 
        self.target_pitch_angle = 0.0
        
        # 视觉误差缓存
        self.latest_yaw_err = 0.0
        self.latest_pitch_err = 0.0
        self.last_msg_time = 0.0
        self.target_visible = False

        # --- PID 设置 ---
        self.pid_yaw = PIDController(kp=0.4, ki=0.0, kd=0.005, max_out=0.1) 
        self.pid_pitch = PIDController(kp=0.4, ki=0.0, kd=0.005, max_out=0.1)

        # --- 订阅视觉话题 ---
        self.sub = self.create_subscription(
            Armors,
            '/detector/armors',
            self.listener_callback,
            qos_profile_sensor_data
        )
        
        # --- 【新增】初始化 TF 广播器 ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 初始化电机 ---
        if hasattr(self, 'ser') and self.ser.is_open:
            self.init_motors()

        # --- 创建控制定时器 (100Hz) ---
        self.timer = self.create_timer(0.01, self.control_loop)
        self.dt = 0.01

    def init_motors(self):
        """使能电机"""
        self.get_logger().info("正在使能电机...")
        enable_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        for _ in range(5):
            send_can_frame(self.ser, self.ID_YAW, enable_cmd)
            time.sleep(0.002)
            send_can_frame(self.ser, self.ID_PITCH, enable_cmd)
            time.sleep(0.01)
        self.get_logger().info("电机已使能")

    def stop_motors(self):
        """失能电机"""
        self.get_logger().info("正在停止电机...")
        disable_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        for _ in range(5):
            if hasattr(self, 'ser') and self.ser.is_open:
                send_can_frame(self.ser, self.ID_YAW, disable_cmd)
                time.sleep(0.002)
                send_can_frame(self.ser, self.ID_PITCH, disable_cmd)
            time.sleep(0.01)

    def listener_callback(self, msg):
        """处理视觉数据"""
        if not msg.armors:
            self.target_visible = False
            return

        self.target_visible = True
        self.last_msg_time = time.time()
        
        armor = msg.armors[0]
        x = armor.pose.position.x
        y = armor.pose.position.y
        z = armor.pose.position.z

        yaw_err = math.atan2(x, z)
        pitch_err = math.atan2(y, z)

        self.latest_yaw_err = yaw_err
        self.latest_pitch_err = pitch_err

    def control_loop(self):
        """主控制循环"""
        
        # 1. 检查目标是否丢失
        if time.time() - self.last_msg_time > 0.2:
            self.target_visible = False
        
        # 2. 计算 PID
        if self.target_visible:
            d_yaw = self.pid_yaw.update(self.latest_yaw_err, self.dt)
            d_pitch = self.pid_pitch.update(self.latest_pitch_err, self.dt)
            
            self.target_yaw_angle -= d_yaw 
            self.target_pitch_angle += d_pitch 
        else:
            pass

        # 3. 软限位
        self.target_yaw_angle = max(min(self.target_yaw_angle, 1.5), -1.5)
        self.target_pitch_angle = max(min(self.target_pitch_angle, 0.6), -0.6)

        # ==========================================
        # 4. 【新增】发布 TF (camera_link)
        # ==========================================
        t = TransformStamped()
        
        # 填充头部信息
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'       # 父坐标系
        t.child_frame_id = 'camera_link' # 子坐标系
        
        # 填充位置 (比 map 高 1m)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0
        
        # 填充旋转 (根据当前云台的 target 角度)
        # 注意：这里假设 Yaw 是绕 Z 轴，Pitch 是绕 Y 轴
        # 符号方向如果不一致，可以在参数前加负号调整，例如 -self.target_yaw_angle
        qx, qy, qz, qw = euler_to_quaternion(
            self.target_yaw_angle,   # Yaw (Z)
            self.target_pitch_angle, # Pitch (Y)
            0.0                      # Roll (X)
        )
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # 广播 TF
        self.tf_broadcaster.sendTransform(t)
        # ==========================================

        # 5. 生成电机指令
        motor_pos_yaw = self.target_yaw_angle * self.GEAR_RATIO
        motor_pos_pitch = self.target_pitch_angle * self.GEAR_RATIO
        
        # 6. 发送指令 (仅当串口开启时)
        if hasattr(self, 'ser') and self.ser.is_open:
            # --- 发送 Yaw ---
            cmd_yaw = create_mit_command(
                pos = motor_pos_yaw,
                vel = 0.0,
                kp  = 5.0, 
                kd  = 0.2,
                tor = 0.0
            )
            send_can_frame(self.ser, self.ID_YAW, cmd_yaw)
            time.sleep(0.001) 
            
            # --- 发送 Pitch ---
            cmd_pitch = create_mit_command(
                pos = motor_pos_pitch,
                vel = 0.0,
                kp  = 5.0,
                kd  = 0.1,
                tor = 0.0 
            )
            send_can_frame(self.ser, self.ID_PITCH, cmd_pitch)

            # 读取反馈
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
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()