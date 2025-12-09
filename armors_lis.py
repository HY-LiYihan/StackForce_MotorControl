import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rclpy.qos import qos_profile_sensor_data
# ==============================================================================
# ⚠️ 请修改下方导入，替换为你实际定义的接口包名
# 假设你的包名为 'auto_aim_interfaces'，消息文件名为 'Armors' (包含 Armor[] armors)
# 如果找不到包，请检查你的 msg 文件定义所在的功能包名称
try:
    from auto_aim_interfaces.msg import Armors  # 这里需要改为你实际的消息类型
except ImportError:
    print("❌ 错误: 无法导入消息类型。请修改代码中的 'from ... import ...' 部分。")
    # 为了防止编辑器报错，这里定义一个假类 (运行时请确保上面的导入正确)
    class Armors: pass
# ==============================================================================

class ArmorListener(Node):
    def __init__(self):
        super().__init__('armor_listener')
        
        self.subscription = self.create_subscription(
            Armors,              # 消息类型
            '/detector/armors',  # 话题名
            self.listener_callback,
            qos_profile_sensor_data  # <--- 修改这里：从 10 改为 qos_profile_sensor_data
        )
        self.get_logger().info('装甲板监听节点已启动 (QoS: Best Effort)...')

    def listener_callback(self, msg):
        # 如果没有检测到装甲板，直接返回
        if not msg.armors:
            return

        # 遍历检测到的所有装甲板
        for i, armor in enumerate(msg.armors):
            # 提取 Pose 中的位置信息
            # x: 右侧 (Right)
            # y: 下方 (Down)
            # z: 距离 (Forward/Depth)
            pos = armor.pose.position
            
            # 提取数字 ID 和 类型 (如果需要)
            armor_id = armor.number
            
            # 打印日志用于调试瞄准
            # 使用 {:.4f} 保留4位小数，方便查看精度
            log_msg = (
                f"\n--- 检测到装甲板 [{i}] ---\n"
                f"ID: {armor_id} | Type: {armor.type}\n"
                f"坐标 (相机系):\n"
                f"  X (右): {pos.x:.4f} m\n"
                f"  Y (下): {pos.y:.4f} m\n"
                f"  Z (深): {pos.z:.4f} m\n"
                f"距图像中心: {armor.distance_to_image_center:.2f}"
            )
            self.get_logger().info(log_msg)

            # --- 瞄准逻辑建议 ---
            # 如果你需要在这里做简单的逻辑判断（例如选择最近的装甲板）：
            # if pos.z < min_distance: ...

def main(args=None):
    rclpy.init(args=args)
    node = ArmorListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()