import serial
import time

print("The converter is equipped with two built - in conversion protocols, \none is a fixed 20 byte protocol and the other is a variable length protocol. \nPlease ensure that the variable length protocol is selected in the supporting software \nand click the Set and Start button to issue a configuration command to the converter")

t = serial.Serial("COM5", 2000000)

print(t.portstr)


def calculate_checksum(data):
    checksum = sum(data[2:])
    return checksum & 0xff


set_can_baudrate = [
    0xaa,     #  0  Packet header
    0x55,     #  1  Packet header
    0x12,     #  3 Type: use variable protocol to send and receive data##  0x02- Setting (using fixed 20 byte protocol to send and receive data),   0x12- Setting (using variable protocol to send and receive data)##
    0x01,     #  3 CAN Baud Rate:  500kbps  ##  0x01(1Mbps),  0x02(800kbps),  0x03(500kbps),  0x04(400kbps),  0x05(250kbps),  0x06(200kbps),  0x07(125kbps),  0x08(100kbps),  0x09(50kbps),  0x0a(20kbps),  0x0b(10kbps),   0x0c(5kbps)##
    0x01,     #  4  Frame Type: Extended Frame  ##   0x01 standard frame,   0x02 extended frame ##
    0x00,     #  5  Filter ID1
    0x00,     #  6  Filter ID2
    0x00,     #  7  Filter ID3
    0x00,     #  8  Filter ID4
    0x00,     #  9  Mask ID1
    0x00,     #  10 Mask ID2
    0x00,     #  11 Mask ID3
    0x00,     #  12 Mask ID4
    0x00,     #  13 CAN mode:  normal mode  ##   0x00 normal mode,   0x01 silent mode,   0x02 loopback mode,   0x03 loopback silent mode ##
    0x00,     #  14 automatic resend:  automatic retransmission
    0x00,     #  15 Spare
    0x00,     #  16 Spare
    0x00,     #  17 Spare
    0x00,     #  18 Spare
]



# Calculate checksum
checksum = calculate_checksum(set_can_baudrate)
set_can_baudrate.append(checksum)
print(set_can_baudrate)
set_can_baudrate = bytes(set_can_baudrate)

# Send command to set CAN baud rate

num_set_baud = t.write(set_can_baudrate)
print(f"Set CAN baud rate command sent, bytes written: {num_set_baud}")

time.sleep(1)
print("send CAN ID:0x01234567,extended frame, data:0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88")
send_can_id_data = bytes([
    0xAA,     # 0  Packet header
    0xC8,     # 1  0xc0 Tyep
    # bit5(frame type 0- standard frame (frame ID 2 bytes), 1-extended frame (frame ID 4 bytes))
    # bit4(frame format 0- data frame, 1 remote frame)
    # Bit0~3 Frame data length (0~8)
    0x03,     # 2  Frame ID data 1    1~8 bit, high bytes at the front, low bytes at the back
    0x02,     # 3  Frame ID data 2    1~8 bit, high bytes at the front, low bytes at the back
    # 0x00,     # 4  Frame ID data 3    1~8 bit, high bytes at the front, low bytes at the back
    # 0x00,     # 5  Frame ID data 4    9~16 bit, high bytes at the front, low bytes at the back
    0xFF,     # 6  Frame data 1       CAN sends  data 1
    0xFF,     # 7  Frame data 2       CAN sends  data 2
    0xFF,     # 8  Frame data 3       CAN sends  data 3
    0xFF,     # 9  Frame data 4       CAN sends  data 4
    0xFF,     # 10 Frame data 5       CAN sends  data 5
    0xFF,     # 11 Frame data 6       CAN sends  data 6
    0xFF,     # 12 Frame data 7       CAN sends  data 7
    0xFD,     # 13 Frame data 8       CAN sends  data 8
    0x55,     # 14 Frame data 4       CAN sends  data 4
])
print(send_can_id_data)
num = t.write(send_can_id_data)
print(num)
t.close()
