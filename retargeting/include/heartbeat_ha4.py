      
import struct
from collections import namedtuple
import logging
from enum import Enum
import time
import socket


class JointStatusHA3(Enum):
    NORMAL = 0  
    IDLE_LOW_POWER = 1  
    LOCK_LOW_POWER = 2  

class TemperatureLevelHA3(Enum):
    BELOW_70 = 0  
    BETWEEN_70_80 = 1  
    BETWEEN_80_90 = 2  
    ABOVE_90 = 3  

PacketHA4 = namedtuple('Packet', [
    'initial_identifier_0', 'initial_identifier_1', 'protocol_version', 'reserved_0',
    'device_type', 'packet_type', 'payload_version', 'number_of_joints',
    'device_sn', 'type_flag', 'sequence', 'timestamp', 'reserved_1',
    'device_pn', 'manufacturer', 'firmware_version', 'type_flag_minor',
    'mac', 'device_ip', 'des_ip_joint', 'des_ip_tactile',
    'des_ip_debug', 'des_ip_heart', 'des_port_joint',
    'des_port_tactile', 'des_port_debug', 'des_port_heart',
    'lis_port_joint', 'lis_port_tcp', 'paired_sn', 'status',
    'reserved',
    'checksum'
])

class HeartbeatHA4:
    _debug_fmt = (
        '<4B'  
        '4B16sHHIQ4x'  
        '16s'    
        '8s'     
        '4B'     
        'H'      
        '6B'     
        '4B'     
        '4B'     
        '4B'     
        '4B'     
        '4B'     
        'H'      
        'H'      
        'H'      
        'H'      
        'H'      
        'H'      
        '16s'    
        'f'      
        'B'      
        'H'      
        'I'      
        'Q'      
        'Q'      
        'Q'      
        'I'      
    )
    
    STRUCT_FMT = _debug_fmt
    
    PACKET_LEN = struct.calcsize(STRUCT_FMT)  
    

    @staticmethod
    def unpack(data):
        if len(data) != HeartbeatHA4.PACKET_LEN:
            raise ValueError(f"Invalid packet length. Expected {HeartbeatHA4.PACKET_LEN} bytes, got {len(data)} bytes.")
        
        if data[0] != 0xbb or data[1] != 0xee:
            raise ValueError(f"Invalid packet header. Expected 0xbb 0xee, got {hex(data[0])} {hex(data[1])}")
        
        unpacked = struct.unpack(HeartbeatHA4.STRUCT_FMT, data)
        
        raw_sn = unpacked[8]  
        stamp = unpacked[11]
        raw_paired_sn = unpacked[52]
        try:
            device_sn = raw_sn.decode('ascii').rstrip('\x00')
            paired_sn = raw_paired_sn.decode('ascii').rstrip('\x00')
        except Exception as e:
            device_sn = ''.join(chr(b) for b in raw_sn if 32 <= b <= 126)
            paired_sn = ''.join(chr(b) for b in raw_paired_sn if 32 <= b <= 126)

        packet = PacketHA4(
            unpacked[0], unpacked[1], unpacked[2], unpacked[3],
            unpacked[4], unpacked[5], unpacked[6], unpacked[7],
            device_sn,  
            unpacked[9], unpacked[10], stamp, unpacked[12],
            unpacked[13].decode('ascii').rstrip('\x00'),  
            unpacked[14].decode('ascii').rstrip('\x00'),  
            unpacked[15:19],  
            unpacked[19],     
            unpacked[20:26],  
            unpacked[26:30],  
            unpacked[30:34],  
            unpacked[34:38],  
            unpacked[38:42],  
            unpacked[42:46],  
            unpacked[46],     
            unpacked[47],     
            unpacked[48],     
            unpacked[49],     
            unpacked[50],     
            unpacked[51],     
            paired_sn,        
            {
                'temperature': unpacked[53],     
                'battery': unpacked[54],         
                'error_code': unpacked[55],      
                'error_joint': unpacked[56],     
                'joint_lock_status': unpacked[57], 
                'temperature_levels': unpacked[58] 
            },
            unpacked[59],     
            unpacked[60]      
        )
        return packet

    @staticmethod
    def pack(packet):
        """
        Pack a PacketHA4 object into bytes.
        This matches the C++ HeartPacketV0::pack() method exactly.
        
        Args:
            packet (PacketHA4): The packet object to pack
            
        Returns:
            bytes: The packed data
        """
        device_sn_bytes = packet.device_sn.encode('ascii').ljust(16, b'\x00')
        paired_sn_bytes = packet.paired_sn.encode('ascii').ljust(16, b'\x00')
        device_pn_bytes = packet.device_pn.encode('ascii').ljust(16, b'\x00')
        manufacturer_bytes = packet.manufacturer.encode('ascii').ljust(8, b'\x00')

        packed_data = struct.pack(HeartbeatHA4.STRUCT_FMT,
            packet.initial_identifier_0,
            packet.initial_identifier_1,
            packet.protocol_version,
            packet.reserved_0,
            
            packet.device_type,
            packet.packet_type,
            packet.payload_version,
            packet.number_of_joints,
            device_sn_bytes,
            packet.type_flag,
            packet.sequence,
            packet.timestamp,
            packet.reserved_1,
            
            device_pn_bytes,
            manufacturer_bytes,
            *packet.firmware_version,
            packet.type_flag_minor,
            *packet.mac,
            *packet.device_ip,
            *packet.des_ip_joint,
            *packet.des_ip_tactile,
            *packet.des_ip_debug,
            *packet.des_ip_heart,
            packet.des_port_joint,
            packet.des_port_tactile,
            packet.des_port_debug,
            packet.des_port_heart,
            packet.lis_port_joint,
            packet.lis_port_tcp,
            paired_sn_bytes,
            packet.status.get('temperature', 0.0) if isinstance(packet.status, dict) else 0.0,
            packet.status.get('battery', 100) if isinstance(packet.status, dict) else 100,
            packet.status.get('error_code', 0) if isinstance(packet.status, dict) else 0,
            packet.status.get('error_joint', 0) if isinstance(packet.status, dict) else 0,
            packet.status.get('joint_lock_status', 0) if isinstance(packet.status, dict) else 0,
            packet.status.get('temperature_levels', 0) if isinstance(packet.status, dict) else 0,
            packet.reserved,
            
            0
        )
        
        checksum = calculate_checksum(packed_data)
        
        final_packet = packed_data[:-4] + struct.pack('<I', checksum)
        
        
        if len(final_packet) != HeartbeatHA4.PACKET_LEN:
            raise ValueError(f"Packet length mismatch: got {len(final_packet)}, expected {HeartbeatHA4.PACKET_LEN}")
        
        return final_packet
    
    @staticmethod
    def parse_status(status):
        """解析状态字段
        状态字段现在是一个包含DeviceStatusInfo的字典
        返回值: 解析后的状态信息字典
        """
        if isinstance(status, dict):
            
            temp_value = status.get('temperature', 0)
            if isinstance(temp_value, bytes):
                try:
                    temp_float = struct.unpack('<f', temp_value[:4])[0]
                except:
                    temp_float = 0.0
                    print(f"DEBUG: Failed to convert temperature bytes, using 0.0")
            else:
                temp_float = float(temp_value)
            
            return {
                'temperature': f"{temp_float:.1f}°C",
                'battery': f"{status.get('battery', 0)}%",
                'error_code': status.get('error_code', 0),
                'error_joint': status.get('error_joint', 0),
                'joint_lock_status': f"0x{status.get('joint_lock_status', 0):016X}",
                'temperature_levels': f"0x{status.get('temperature_levels', 0):016X}"
            }
        else:
            try:
                if isinstance(status, bytes):
                    status_int = int.from_bytes(status, byteorder='little')
                    return {
                        'raw_status': status_int,
                        'hex_status': f"0x{status_int:016X}",
                        'bytes_status': status
                    }
                else:
                    return {
                        'raw_status': status,
                        'hex_status': f"0x{status:016X}"
                    }
            except Exception as e:
                return {
                    'raw_status': status,
                    'hex_status': f"Error: {e}",
                    'type': type(status).__name__
                }

    @staticmethod
    def parse_joint_status(joint_status, N=22):
        """解析关节状态
        每个关节占用2位，从低位到高位
        返回值: 字典，key为JointStatus枚举值，value为关节索引
        """
        result = {JointStatusHA3.NORMAL: [], JointStatusHA3.IDLE_LOW_POWER: [], JointStatusHA3.LOCK_LOW_POWER: []}
        for i in range(N):  
            status_bits = (joint_status >> (i * 2)) & 0x3
            if status_bits in [0, 1, 2]:
                result[JointStatusHA3(status_bits)].append(i)
        return result

    @staticmethod
    def parse_temperature_level(temp_level, N=22):
        """解析温度等级
        每个关节占用2位，从低位到高位
        返回值: 字典，key为TemperatureLevel枚举值，value为关节索引
        """
        result = {TemperatureLevelHA3.BELOW_70: [], TemperatureLevelHA3.BETWEEN_70_80: [], TemperatureLevelHA3.BETWEEN_80_90: [], TemperatureLevelHA3.ABOVE_90: []}
        for i in range(N):  
            temp_bits = (temp_level >> (i * 2)) & 0x3
            if temp_bits in [0, 1, 2, 3]:
                result[TemperatureLevelHA3(temp_bits)].append(i)
        return result

def format_ip(ip_bytes):
    """Format IP address bytes to string, handling different input formats"""
    if ip_bytes is None:
        return "None"
    elif isinstance(ip_bytes, (list, tuple)):
        try:
            return '.'.join(str(b) for b in ip_bytes)
        except Exception as e:
            return f"Error: {e} (raw: {ip_bytes})"
    elif isinstance(ip_bytes, bytes):
        try:
            return '.'.join(str(b) for b in ip_bytes)
        except Exception as e:
            return f"Error: {e} (raw: {ip_bytes})"
    else:
        return str(ip_bytes)

def format_mac(mac_bytes):
    """Format MAC address bytes to string, handling different input formats"""
    if mac_bytes is None:
        return "None"
    elif isinstance(mac_bytes, (list, tuple)):
        try:
            return ':'.join(f'{b:02x}' for b in mac_bytes)
        except Exception as e:
            return f"Error: {e} (raw: {mac_bytes})"
    elif isinstance(mac_bytes, bytes):
        try:
            return ':'.join(f'{b:02x}' for b in mac_bytes)
        except Exception as e:
            return f"Error: {e} (raw: {mac_bytes})"
    else:
        return str(mac_bytes)

def calculate_checksum(data):
    """Calculate CRC32 checksum for packet data (excluding checksum field)
    This matches the C++ implementation: crc32(0, data.data(), data.size() - sizeof(uint32_t))
    """
    import zlib
    data_for_checksum = data[:-4]
    return zlib.crc32(data_for_checksum, 0) & 0xffffffff  

def print_packet(packet):
    try:
        print("=== Packet PreHeader ===")
        print(f"Initial Identifier 0: {packet.initial_identifier_0}")
        print(f"Initial Identifier 1: {packet.initial_identifier_1}")
        print(f"Protocol Version: {packet.protocol_version}")
        print(f"Reserved: {packet.reserved_0}")
        
        print("\n=== Packet DataHeader ===")
        print(f"Device Type: {packet.device_type}")
        print(f"Packet Type: {packet.packet_type}")
        print(f"Payload Version: {packet.payload_version}")
        print(f"Number of Joints: {packet.number_of_joints}")
        print(f"Device SN: {packet.device_sn}")
        print(f"Type Flag: {packet.type_flag}")
        print(f"Sequence: {packet.sequence}")
        print(f"Timestamp: {packet.timestamp}")
        print(f"Reserved: {packet.reserved_1}")
        
        print("\n=== HeartPayloadV0 ===")
        print(f"Device PN: {packet.device_pn}")
        print(f"Manufacturer: {packet.manufacturer}")
        print(f"Firmware Version: {packet.firmware_version}")
        print(f"Type Flag Minor: {packet.type_flag_minor}")
        
        print(f"MAC Address (raw): {packet.mac}")
        print(f"MAC Address: {format_mac(packet.mac)}")
        
        print(f"Device IP (raw): {packet.device_ip}")
        print(f"Device IP: {format_ip(packet.device_ip)}")
        print(f"Destination IP - Joint (raw): {packet.des_ip_joint}")
        print(f"Destination IP - Joint: {format_ip(packet.des_ip_joint)}")
        print(f"Destination IP - Tactile (raw): {packet.des_ip_tactile}")
        print(f"Destination IP - Tactile: {format_ip(packet.des_ip_tactile)}")
        print(f"Destination IP - Debug (raw): {packet.des_ip_debug}")
        print(f"Destination IP - Debug: {format_ip(packet.des_ip_debug)}")
        print(f"Destination IP - Heart (raw): {packet.des_ip_heart}")
        print(f"Destination IP - Heart: {format_ip(packet.des_ip_heart)}")
        
        print(f"Destination Port - Joint: {packet.des_port_joint}")
        print(f"Destination Port - Tactile: {packet.des_port_tactile}")
        print(f"Destination Port - Debug: {packet.des_port_debug}")
        print(f"Destination Port - Heart: {packet.des_port_heart}")
        print(f"Listen Port - Joint: {packet.lis_port_joint}")
        print(f"Listen Port - TCP: {packet.lis_port_tcp}")
        print(f"Paired SN: {packet.paired_sn}")
        
        print(f"Status (raw): {packet.status} (type: {type(packet.status).__name__})")
        status_info = HeartbeatHA4.parse_status(packet.status)
        if isinstance(packet.status, dict):
            print(f"Temperature: {status_info['temperature']}")
            print(f"Battery: {status_info['battery']}")
            print(f"Error Code: {status_info['error_code']}")
            print(f"Error Joint: {status_info['error_joint']}")
            print(f"Joint Lock Status: {status_info['joint_lock_status']}")
            print(f"Temperature Levels: {status_info['temperature_levels']}")
        else:
            print(f"Status: {status_info.get('hex_status', status_info)}")
        print(f"Reserved: 0x{packet.reserved:016X}")
        
        print("\n=== Tail ===")
        print(f"Checksum: 0x{packet.checksum:08X}")
        
        if hasattr(packet, 'joint_status') and hasattr(packet, 'temperature_level'):
            print("\n=== Additional Info ===")
            joint_status_info = HeartbeatHA4.parse_joint_status(packet.joint_status)
            temp_level_info = HeartbeatHA4.parse_temperature_level(packet.temperature_level)
            print(f"Joint Status: {joint_status_info}")
            print(f"Temperature Level: {temp_level_info}")
            
    except Exception as e:
        print(f"Error in print_packet: {e}")
        import traceback
        traceback.print_exc()

def recv_heartbeat():
    example_data = bytes(HeartbeatHA4.PACKET_LEN)    
    import socket
    UDP_IP='0.0.0.0'
    port = 54321
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
    sock.bind((UDP_IP, port))
    sock.settimeout(2.0)
    data, addr = sock.recvfrom(1500)
    try:
        packet = HeartbeatHA4.unpack(data)
        print_packet(packet)
    except ValueError as e:
        print(f"Error parsing packet: {e}")

def calculate_broadcast_ip(ip_address):
    """
    Calculate broadcast address for a given IP address (assuming /24 subnet)
    
    Args:
        ip_address: IP address string like "192.168.1.100"
        
    Returns:
        Broadcast address string like "192.168.1.255"
    """
    try:
        parts = ip_address.split('.')
        if len(parts) != 4:
            raise ValueError(f"Invalid IP address format: {ip_address}")
        
        # For /24 subnet, broadcast is x.x.x.255
        broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
        return broadcast
    except Exception as e:
        print(f"Error calculating broadcast address for {ip_address}: {e}")
        # Fallback to common broadcast address
        return "255.255.255.255"

def send_heartbeat(device_sn, ip):
    """
    发送心跳包到灵巧手
    
    心跳包作用：
    1. 告诉灵巧手此设备的存在和标识信息
    2. 配置灵巧手的数据发送目标（IP和端口）
    3. 告诉灵巧手在哪里监听命令
    
    Args:
        device_sn: 设备序列号，如 'Glove-R-0001'
        ip: 设备IP地址，用于标识设备并计算广播地址
        
    心跳包包含的关键配置：
        - device_ip: 设备标识IP
        - des_ip_joint: 灵巧手将关节状态数据发送到此IP
        - des_port_joint: 灵巧手将关节状态数据发送到此端口
        - lis_port_joint: 灵巧手在此端口监听关节命令
    """
    import socket
    import time
    import logging
    
    class HandType(Enum):
        LEFT = 0
        RIGHT = 1
        
    class HardwareVersion(Enum):
        H = 0
        M = 1
    
    # Calculate broadcast address based on device IP
    BROADCAST_IP = calculate_broadcast_ip(ip)
    port = 54321  
    
    direction_bit = 1 if 'R' in device_sn else 0
    hand_side = HandType(direction_bit)

    device_type = 1 if device_sn.startswith('Glove') else 0
    
    packet_type = 0x03  
    
    control_mode = 0x01  
    type_flag = (direction_bit << 15) | control_mode  
    
    if hand_side == HandType.LEFT:
        joint_port = 50020  
    else:
        joint_port = 50030

    
    
    ip_tuple = tuple(map(int, ip.split('.')))
    
    # Calculate broadcast address tuple for destination IPs
    broadcast_tuple = tuple(map(int, BROADCAST_IP.split('.')))
    
    type_flag_minor = 0x02  
    
    packet = PacketHA4(
        initial_identifier_0=0xbb,
        initial_identifier_1=0xee,
        protocol_version=1,
        reserved_0=0,
        
        device_type=device_type,  
        packet_type=packet_type,  
        payload_version=0x00,     
        number_of_joints=22,      
        device_sn=device_sn,
        type_flag=type_flag,      
        sequence=0,
        timestamp=int(time.time()),  
        reserved_1=0,
        
        device_pn='Wave-X1',     
        manufacturer='Sharpa',     
        firmware_version=(0x00, 0x01, 0x00, 0x00),  
        type_flag_minor=type_flag_minor,  
        mac=(0x12, 0x34, 0x56, 0x78, 0x90, 0xAB),  
        device_ip=ip_tuple,                    # 设备标识IP
        des_ip_joint=broadcast_tuple,          # 关节数据发送到广播地址（动态计算）
        des_ip_tactile=ip_tuple,               # 触觉数据发送到设备IP（双向通信）
        des_ip_debug=broadcast_tuple,          # 调试数据发送到广播地址（动态计算）
        des_ip_heart=(255, 255, 255, 255),     # 心跳响应发送到全局广播    
        des_port_joint=joint_port,             # 目标端口：灵巧手将关节状态数据发送到此端口
        des_port_tactile=50001,                # 目标端口：灵巧手将触觉数据发送到此端口
        des_port_debug=50005,                  # 目标端口：灵巧手将调试数据发送到此端口
        des_port_heart=54321,                  # 目标端口：灵巧手将心跳响应发送到此端口
        lis_port_joint=50020,                  # 监听端口：灵巧手在此端口监听关节命令
        lis_port_tcp=0,                        # TCP监听端口（0表示不使用TCP）
        paired_sn='',                          # 配对的设备序列号（空表示未配对）             
        status={
            'temperature': 25.0,  
            'battery': 100,       
            'error_code': 0,      
            'error_joint': 0,     
            'joint_lock_status': 0,    
            'temperature_levels': 0    
        },
        reserved=0,
        
        checksum=0
    )
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(1.0)
        
        logging.info(f"Starting heartbeat for device {device_sn} from IP {ip}")
        logging.info(f"Device type: {device_type} (0x{device_type:02X}), Packet type: {packet_type} (0x{packet_type:02X})")
        logging.info(f"Type Flag: {type_flag} (0x{type_flag:04X})")
        logging.info(f"  - Control Mode: {'Remote Mode' if (type_flag & 0x07) == 0x01 else 'Master Mode' if (type_flag & 0x07) == 0x00 else 'Floating Mode'}")
        logging.info(f"  - Hand Side: {'Right' if (type_flag & 0x8000) else 'Left'}")
        logging.info(f"Broadcasting to {BROADCAST_IP}:{port}")
        
        sequence = 0
        while True:
            try:
                packet = packet._replace(
                    sequence=sequence,
                    timestamp=int(time.time())
                )
                
                data = HeartbeatHA4.pack(packet)
                
                sock.sendto(data, (BROADCAST_IP, port))
                logging.debug(f"Heartbeat sent for device {device_sn}, sequence: {sequence}")
                
                sequence += 1
                time.sleep(1)  
                
            except socket.error as e:
                logging.error(f"Error sending heartbeat: {e}")
                time.sleep(1)  
            except Exception as e:
                logging.error(f"Unexpected error: {e}")
                time.sleep(1)
                
    except Exception as e:
        logging.error(f"Failed to create socket: {e}")
    finally:
        if 'sock' in locals():
            sock.close()
            logging.info("Socket closed")


if __name__ == "__main__":
    send_heartbeat(device_sn='Glove-R-0000', ip='192.168.10.100')    
