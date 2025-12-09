import serial
import time

class USB2CAN:
    def __init__(self, port, baudrate=2000000, timeout=1):
        """
        初始化USB转CAN转换器
        :param port: 串口端口（如"COM5"或"/dev/ttyUSB0"）
        :param baudrate: 串口波特率，默认2000000
        :param timeout: 串口超时时间，默认1秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None  # 串口对象，初始化为None

    def connect(self):
        """建立串口连接"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.serial.is_open:
                print(f"成功连接到 {self.serial.portstr}")
                return True
            return False
        except serial.SerialException as e:
            print(f"连接失败: {e}")
            return False

    def calculate_checksum(self, data):
        """计算校验和"""
        checksum = sum(data[2:])  # 从索引2开始计算
        return checksum & 0xff    # 取低8位

    def set_config(self, can_baudrate=0x01, frame_type=0x01, can_mode=0x00):
        """
        配置CAN转换器参数
        :param can_baudrate: CAN波特率（默认0x01=1Mbps）
        :param frame_type: 帧类型（0x01=标准帧，0x02=扩展帧，默认标准帧）
        :param can_mode: CAN模式（0x00=正常模式，默认正常模式）
        :return: 配置是否成功
        """
        if not self.serial or not self.serial.is_open:
            print("请先建立串口连接")
            return False

        # 配置命令帧结构
        config_data = [
            0xaa,     # 0: 帧头
            0x55,     # 1: 帧头
            0x12,     # 2: 类型 - 使用可变长度协议
            can_baudrate,  # 3: CAN波特率
            frame_type,    # 4: 帧类型
            0x00,     # 5: 过滤ID1
            0x00,     # 6: 过滤ID2
            0x00,     # 7: 过滤ID3
            0x00,     # 8: 过滤ID4
            0x00,     # 9: 掩码ID1
            0x00,     # 10: 掩码ID2
            0x00,     # 11: 掩码ID3
            0x00,     # 12: 掩码ID4
            can_mode,  # 13: CAN模式
            0x00,     # 14: 自动重发
            0x00,     # 15: 保留位
            0x00,     # 16: 保留位
            0x00,     # 17: 保留位
            0x00,     # 18: 保留位
        ]

        # 计算并添加校验和
        checksum = self.calculate_checksum(config_data)
        config_data.append(checksum)
        
        # 发送配置命令
        config_bytes = bytes(config_data)
        bytes_written = self.serial.write(config_bytes)
        print(f"配置命令已发送，写入 {bytes_written} 字节")
        print(f"配置详情: 波特率={can_baudrate}, 帧类型={frame_type}, 模式={can_mode}")
        
        # 等待配置生效
        time.sleep(0.1)
        return True

    def send(self, frame_id, data, is_extended=False):
        """
        发送CAN消息
        :param frame_id: CAN帧ID
        :param data: 要发送的数据列表或字节数组（最大8字节）
        :param is_extended: 是否为扩展帧（默认False为标准帧）
        :return: 发送的字节数，失败返回0
        """
        if not self.serial or not self.serial.is_open:
            print("请先建立串口连接")
            return 0

        # 检查数据长度（CAN数据最大8字节）
        if len(data) > 8:
            print("数据长度超过8字节，自动截断")
            data = data[:8]

        # 构建发送帧
        packet = [0xAA]  # 0: 帧头

        # 类型字节（bit5: 帧类型, bit0~3: 数据长度）
        type_byte = 0xC0  # 基础类型值
        if is_extended:
            type_byte |= 0x20  # bit5=1 表示扩展帧
        type_byte |= len(data)  # 低4位表示数据长度
        packet.append(type_byte)

        # 添加帧ID（根据帧类型选择2字节或4字节）
        if is_extended:
            # 扩展帧：4字节ID（高位在前）
            packet.extend([
                (frame_id >> 24) & 0xFF,
                (frame_id >> 16) & 0xFF,
                (frame_id >> 8) & 0xFF,
                frame_id & 0xFF
            ])
        else:
            # 标准帧：2字节ID（高位在前）
            packet.extend([
                (frame_id >> 8) & 0xFF,
                frame_id & 0xFF
            ])

        # 添加数据
        packet.extend(data)

        # 添加结束字节
        packet.append(0x55)

        # 转换为字节并发送
        packet_bytes = bytes(packet)
        bytes_written = self.serial.write(packet_bytes)
        print(f"发送CAN消息: ID=0x{frame_id:X}, 数据={[hex(b) for b in data]}, 长度={bytes_written}")
        return bytes_written

    def close(self):
        """关闭串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"已关闭 {self.port} 连接")


# 使用示例
if __name__ == "__main__":
    # 初始化USB2CAN对象
    usb2can = USB2CAN(port="COM5")
    
    # 建立连接
    if usb2can.connect():
        # 配置CAN转换器（1Mbps，标准帧）
        usb2can.set_config(
            can_baudrate=0x01,  # 1Mbps
            frame_type=0x01,    # 标准帧
            can_mode=0x00       # 正常模式
        )
        
        # 发送CAN消息（标准帧，ID=0x0203，数据为失能指令）
        usb2can.send(
            frame_id=0x0203,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            is_extended=False
        )
        
        # 关闭连接
        usb2can.close()
