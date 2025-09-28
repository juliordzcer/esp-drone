import socket
import time
import struct
import math

DRONE_IP = "192.168.1.83"
DRONE_PORT = 1998

def create_exact_commander_packet(roll, pitch, yaw, thrust):
    header = 0x30
    return struct.pack('<BfffH', header, roll, pitch, yaw, thrust)

def test_commander_25hz():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        print("=== COMMANDER 25Hz OPTIMIZADO ===")
        
        start_time = time.time()
        packet_count = 0
        
        while time.time() - start_time < 12:
            elapsed = time.time() - start_time
            
            # Perfil de vuelo más conservador
            if elapsed < 3:
                thrust = int(15000 + 25000 * (elapsed / 3))  # 15000-40000
            elif elapsed < 6:
                thrust = 40000  # Mantener
            elif elapsed < 9:
                thrust = 35000  # Reducir un poco
            else:
                thrust = int(35000 - 35000 * ((elapsed - 9) / 3))  # Descender
            
            # Movimientos muy suaves
            roll = 0.02 * math.sin(elapsed * 1.0)
            pitch = 0.02 * math.cos(elapsed * 1.0)
            
            packet = create_exact_commander_packet(roll, pitch, 0.0, thrust)
            sock.sendto(packet, (DRONE_IP, DRONE_PORT))
            
            packet_count += 1
            
            if packet_count % 25 == 0:  # Log cada segundo (25 paquetes)
                print(f"T+{int(elapsed)}s: thrust={thrust}")
            
            time.sleep(0.04)  # 25Hz
        
        print(f"✅ Test completado. Total paquetes: {packet_count}")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        # Stop seguro
        for i in range(10):
            stop_packet = create_exact_commander_packet(0.0, 0.0, 0.0, 0)
            sock.sendto(stop_packet, (DRONE_IP, DRONE_PORT))
            time.sleep(0.01)
        
        sock.close()

if __name__ == "__main__":
    test_commander_25hz()