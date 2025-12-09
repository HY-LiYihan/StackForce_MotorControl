import serial
import time
from typing import List, Optional, Union

class USB2CAN:
    """
    USB转CAN转换器通信类，用于配置CAN参数和发送CAN消息
    支持标准帧和扩展帧，采用可变长度协议
    """
    
    # 类常量定义 - CAN参数选项
    BAUDRATE_1M = 0x01
    BAUDRATE_800K = 0x02
    BAUDRATE_500K = 0x03
    BAUDRATE_400K = 0x04
    BAUDRATE_250K = 0x05
    
    FRAME_STANDARD = 0x01
    FRAME_EXTENDED = 0x02
    
    MODE_NORMAL = 0x00
    MODE_SILENT = 0x01
    MODE_LOOPBACK = 0x02
    MODE_LOOPBACK_SILENT = 0x03

    def __init__(self, port: str, baudrate: int = 2000000, timeout: float = 1.0):
        """
        初始化并建立与USB转CAN模块的连接
        
        :param port: 串口端口名称（如"COM5"或"/dev/ttyUSB0"）
        :param baudrate: 串口波特率，默认2000000
        :param timeout: 串口超时时间（秒），默认1.0
        """
        # 公共属性（通过@property暴露）
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        
        # 私有成员
        self._serial: Optional[serial.Serial] = None
        self._is_connected: bool = False
        self._can_baudrate: Optional[int] = None
        self._frame_type: Optional[int] = None
        
        # 初始化时建立连接
        self._connect()

    # ---------- 公共属性（Properties） ----------
    @property
    def port(self) -> str:
        """获取串口端口名称"""
        return self._port

    @property
    def baudrate(self) -> int:
        """获取串口波特率"""
        return self._baudrate

    @property
    def is_connected(self) -> bool:
        """获取当前连接状态"""
        return self._is_connected

    @property
    def can_baudrate(self) -> Optional[int]:
        """获取当前CAN波特率配置"""
        return self._can_baudrate

    @property
    def frame_type(self) -> Optional[int]:
        """获取当前帧类型配置（标准帧/扩展帧）"""
        return self._frame_type

    # ---------- 私有方法（Private Methods） ----------
    def _connect(self) -> None:
        """建立串口连接（私有方法）"""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
                stopbits=serial.STOPBITS_ONE,
            )
            
            if self._serial.is_open:
                self._is_connected = True
                print(f"✅ 成功连接到USB转CAN模块: {self._port} (波特率: {self._baudrate})")
            else:
                self._is_connected = False
                print(f"❌ 无法打开串口: {self._port}")
                
        except serial.SerialException as e:
            self._is_connected = False
            print(f"❌ 串口连接失败: {str(e)}")
        except Exception as e:
            self._is_connected = False
            print(f"❌ 初始化失败: {str(e)}")

    def _calculate_checksum(self, data: List[int]) -> int:
        """计算校验和（私有方法）"""
        if len(data) < 3:
            return 0
        checksum = sum(data[2:])  # 从第三个字节开始计算
        return checksum & 0xFF    # 取低8位

    def _validate_data_length(self, data: Union[List[int], bytes]) -> Union[List[int], bytes]:
        """验证并截断数据长度（最大8字节，私有方法）"""
        if len(data) > 8:
            print(f"⚠️ 数据长度超过8字节，自动截断为前8字节")
            return data[:8]
        return data

    # ---------- 公共方法（Public Methods） ----------
    def set_config(self, 
                  can_baudrate: int = BAUDRATE_1M, 
                  frame_type: int = FRAME_STANDARD, 
                  can_mode: int = MODE_NORMAL) -> bool:
        """
        配置CAN总线参数
        
        :param can_baudrate: CAN波特率（使用类常量BAUDRATE_*）
        :param frame_type: 帧类型（FRAME_STANDARD或FRAME_EXTENDED）
        :param can_mode: 工作模式（使用类常量MODE_*）
        :return: 配置是否成功
        """
        if not self._is_connected or not self._serial:
            print("❌ 未建立连接，请检查串口状态")
            return False

        # 构建配置帧
        config_frame = [
            0xAA,           # 帧头1
            0x55,           # 帧头2
            0x12,           # 协议类型：可变长度协议
            can_baudrate,   # CAN波特率
            frame_type,     # 帧类型
            0x00, 0x00, 0x00, 0x00,  # 过滤ID
            0x00, 0x00, 0x00, 0x00,  # 掩码ID
            can_mode,       # 工作模式
            0x00,           # 自动重发使能
            0x00, 0x00, 0x00, 0x00   # 保留位
        ]

        # 计算并添加校验和
        checksum = self._calculate_checksum(config_frame)
        config_frame.append(checksum)

        # 发送配置命令
        try:
            bytes_written = self._serial.write(bytes(config_frame))
            if bytes_written == len(config_frame):
                self._can_baudrate = can_baudrate
                self._frame_type = frame_type
                print(f"✅ CAN配置成功: 波特率={can_baudrate}, 帧类型={frame_type}, 模式={can_mode}")
                time.sleep(0.5)  # 等待配置生效
                return True
            else:
                print(f"❌ 配置发送不完整 (发送{bytes_written}/{len(config_frame)}字节)")
                return False
        except Exception as e:
            print(f"❌ 配置发送失败: {str(e)}")
            return False

    def send(self, frame_id: int, data: Union[List[int], bytes], is_extended: Optional[bool] = False) -> int:
        """
        发送CAN消息
        
        :param frame_id: CAN帧ID（整数）
        :param data: 消息数据（列表或字节数组，最大8字节）
        :param is_extended: 是否为扩展帧
        :return: 实际发送的字节数，失败返回0
        """
        if not self._is_connected or not self._serial:
            print("❌ 未建立连接，请检查串口状态")
            return 0

        # 确定帧类型（优先使用参数，其次使用配置的默认值）
        frame_type = is_extended if is_extended is not None else (self._frame_type == self.FRAME_EXTENDED)

        # 验证数据长度
        data = self._validate_data_length(data)

        # 构建发送帧
        try:
            # 帧头和类型字节
            packet = [0xAA]  # 帧头
            
            # 类型字节（bit5: 帧类型, bit0~3: 数据长度）
            type_byte = 0xC8  # 基础类型
            type_byte |= len(data)  # 数据长度
            packet.append(type_byte)
            packet.extend([
                    frame_id & 0xFF,         # 低8位先发送
                    (frame_id >> 8) & 0xFF   # 高8位后发送
                ])

            # 添加数据
            packet.extend(data if isinstance(data, list) else list(data))

            # 添加结束标识
            packet.append(0x55)

            # 发送数据
            packet_bytes = bytes(packet)
            bytes_written = self._serial.write(packet_bytes)
            print(f"✅ 发送CAN消息: ID=0x{frame_id:X}, 数据={[hex(b) for b in data]}, 长度={bytes_written}")
            return bytes_written

        except Exception as e:
            print(f"❌ 消息发送失败: {str(e)}")
            return 0

    def close(self) -> None:
        """关闭串口连接"""
        if self._is_connected and self._serial:
            self._serial.close()
            self._is_connected = False
            print(f"✅ 已关闭与{self._port}的连接")
        else:
            print("⚠️ 未建立连接，无需关闭")

    # 上下文管理器支持（with语句）
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# 使用示例
if __name__ == "__main__":
    # 使用上下文管理器（自动关闭连接）
    with USB2CAN(port="COM5", baudrate=2000000) as usb2can:
        if usb2can.is_connected:
            # 配置CAN参数（1Mbps，标准帧，正常模式）
            usb2can.set_config(
                can_baudrate=USB2CAN.BAUDRATE_1M,
                frame_type=USB2CAN.FRAME_STANDARD,
                can_mode=USB2CAN.MODE_NORMAL
            )
            
            # 发送标准帧消息（失能指令）
            usb2can.send(
                frame_id=0x0204,
                data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
                is_extended=False
            )
            
            # 发送扩展帧消息（测试数据）
            # usb2can.send(
            #     frame_id=0x12345678,
            #     data=[0x11, 0x22, 0x33, 0x44],
            #     is_extended=True
            # )
    