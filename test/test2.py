import socket
import time
import struct

DRONE_IP = "192.168.1.83"
DRONE_PORT = 1998

def create_correct_commander_packet(roll, pitch, yaw, thrust):
    """
    Formato exacto que espera el commander:
    struct CommanderCrtpValues {
        float roll;
        float pitch; 
        float yaw;
        uint16_t thrust;
    } __attribute__((packed));
    
    Total: 4 + 4 + 4 + 2 = 14 bytes + header
    """
    header = 0x30  # port=3, channel=0
    # Empaquetar: header + 3 floats + 1 uint16
    data = struct.pack('<BfffH', header, roll, pitch, yaw, thrust)
    return data

def test_commander_fixed():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        print("=== TEST COMMANDER CON FORMATO CORRECTO ===")
        
        # Enviar comando de despegue por 5 segundos
        start_time = time.time()
        packet_count = 0
        
        while time.time() - start_time < 10:
            # Thrust gradual
            elapsed = time.time() - start_time
            if elapsed < 3:
                thrust = int(30000 + 10000 * (elapsed / 3))  # 30000-40000
            elif elapsed < 7:
                thrust = 40000  # Mantener
            else:
                thrust = int(40000 - 20000 * ((elapsed - 7) / 3))  # Descender
            
            packet = create_correct_commander_packet(0.0, 0.0, 0.0, thrust)
            sock.sendto(packet, (DRONE_IP, DRONE_PORT))
            
            packet_count += 1
            if packet_count % 10 == 0:
                print(f"Paquete {packet_count}: thrust={thrust}")
            
            time.sleep(0.02)  # 50Hz - igual que la frecuencia del stabilizer
        
        print(f"Test completado. Total paquetes: {packet_count}")
        
    finally:
        # Enviar comando de stop final
        stop_packet = create_correct_commander_packet(0.0, 0.0, 0.0, 0)
        sock.sendto(stop_packet, (DRONE_IP, DRONE_PORT))
        sock.close()

if __name__ == "__main__":
    test_commander_fixed()