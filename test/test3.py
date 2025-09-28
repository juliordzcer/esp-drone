import socket
import struct
import time
import threading
import math
from collections import deque

# --- CONFIGURACIÓN DE CONEXIÓN ---
DRONE_IP = "192.168.1.83" 
PORT = 1998 

# --- CONSTANTES CRTP ---
CRTP_PORT_LOG = 0x05
CRTP_CHANNEL_TOC = 0x00
CRTP_CHANNEL_CONFIG = 0x01
CRTP_CHANNEL_DATA = 0x02 # Canal de datos de log

# Comandos y Tipos
TOC_CMD_GET_ITEM = 0x00
TOC_CMD_GET_INFO = 0x01 
LOG_CMD_CREATE_BLOCK = 0x00
LOG_CMD_STOP_BLOCK = 0x02 
LOG_CMD_START_BLOCK = 0x03
LOG_TYPE_FLOAT = 7 

LOG_BLOCK_ID = 0 

# --- FUNCIÓN DE UTILIDAD ---
def CRTP_HEADER(port, channel):
    """Genera la cabecera CRTP."""
    return ((port & 0x0F) << 4) | (channel & 0x03)

class DroneDataReceiver:
    def __init__(self, ip=DRONE_IP, port=PORT):
        self.drone_ip = ip
        self.port = port
        self.sock = None
        self.running = False
        self.thread = None
        self.log_block_id = None
        self.toc_data = {} 
        self.required_vars = ['stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw'] 
        self.packet_count = 0
        self.start_time = 0
        
    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Escuchar en el puerto local para recibir paquetes del drone
            self.sock.bind(('', self.port)) 
            self.sock.settimeout(1.0)
            self.running = True
            self.thread = threading.Thread(target=self._receive_loop)
            self.thread.daemon = True
            self.thread.start()
            self.start_time = time.time()
            print(f"✅ Receptor iniciado. Escuchando y enviando a {self.drone_ip}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ Error iniciando receptor: {e}")
            return False
    
    def stop(self):
        self.running = False
        self._stop_log_block() 
        if self.sock: self.sock.close()
        if self.thread: self.thread.join(timeout=2.0)
        print("🔴 Receptor detenido")

    def _send_crtp_packet(self, port, channel, data=b''):
        try:
            header = CRTP_HEADER(port, channel)
            packet = bytes([header]) + data
            self.sock.sendto(packet, (self.drone_ip, self.port)) 
            return True
        except:
            return False
            
    def get_log_toc(self):
        print("-> [CRTP] Solicitando Log TOC Info...")
        self._send_crtp_packet(CRTP_PORT_LOG, CRTP_CHANNEL_TOC, bytes([TOC_CMD_GET_INFO]))
        time.sleep(0.1)
        
        MAX_TOC_ITEMS_TO_CHECK = 40 
        print(f"-> [CRTP] Solicitando detalles de hasta {MAX_TOC_ITEMS_TO_CHECK} variables...")
        for i in range(MAX_TOC_ITEMS_TO_CHECK):
            payload = struct.pack('<BB', TOC_CMD_GET_ITEM, i)
            self._send_crtp_packet(CRTP_PORT_LOG, CRTP_CHANNEL_TOC, payload)
            time.sleep(0.01)

        print(f"✅ Esperando 1s para recibir todas las respuestas del TOC...")
        time.sleep(1) 

        found_vars = [v for v in self.toc_data.values() if v['name'] in self.required_vars]
        if len(found_vars) == len(self.required_vars):
            print("✅ TOC descargado y variables encontradas:")
            for var in found_vars:
                 print(f"   - {var['name']} (ID: {var['id']}, Tipo: {var['type']})")
            return True
        else:
            print(f"❌ Error: No se encontraron todas las variables requeridas en el TOC.")
            return False

    def create_and_start_log_block(self, period_10ms=4):
        roll_id = next((v['id'] for v in self.toc_data.values() if v['name'] == 'stabilizer.roll'), None)
        pitch_id = next((v['id'] for v in self.toc_data.values() if v['name'] == 'stabilizer.pitch'), None)
        yaw_id = next((v['id'] for v in self.toc_data.values() if v['name'] == 'stabilizer.yaw'), None)

        if roll_id is None or pitch_id is None or yaw_id is None: return

        # 1. Crear el bloque 
        payload_create = struct.pack('<BB', LOG_CMD_CREATE_BLOCK, LOG_BLOCK_ID)
        payload_create += struct.pack('<BB', LOG_TYPE_FLOAT, roll_id)
        payload_create += struct.pack('<BB', LOG_TYPE_FLOAT, pitch_id)
        payload_create += struct.pack('<BB', LOG_TYPE_FLOAT, yaw_id)
        
        print(f"-> [CRTP] Creando Log Block ID {LOG_BLOCK_ID} con IDs {roll_id}, {pitch_id}, {yaw_id}...")
        self._send_crtp_packet(CRTP_PORT_LOG, CRTP_CHANNEL_CONFIG, payload_create)

        time.sleep(0.1) 

        # 2. Iniciar el bloque (period_10ms=4 significa 40ms, o 25Hz)
        payload_start = struct.pack('<BBB', LOG_CMD_START_BLOCK, LOG_BLOCK_ID, period_10ms)
        self._send_crtp_packet(CRTP_PORT_LOG, CRTP_CHANNEL_CONFIG, payload_start)
        
        self.log_block_id = LOG_BLOCK_ID

    def _stop_log_block(self):
        if self.log_block_id is None: return
        payload_stop = struct.pack('<BB', LOG_CMD_STOP_BLOCK, self.log_block_id)
        print(f"-> [CRTP] Deteniendo Log Block ID {self.log_block_id}...")
        self._send_crtp_packet(CRTP_PORT_LOG, CRTP_CHANNEL_CONFIG, payload_stop) 
        self.log_block_id = None
        
    def _receive_loop(self):
        self.packet_count = 0
        
        while self.running:
            try:
                # Intenta recibir un paquete UDP
                data, addr = self.sock.recvfrom(64) 
                
                # Intentamos parsear el paquete y, si es de log, lo mostramos
                if self._parse_incoming_packet(data):
                    self.packet_count += 1
                        
            except socket.timeout: 
                continue
            except Exception as e:
                if self.running: print(f"⚠️  Error recibiendo datos: {e}")
                break
                
    def _parse_incoming_packet(self, data):
        if len(data) < 1: return False
        header = data[0]
        port = (header >> 4) & 0x0F
        channel = header & 0x03
        
        if port == CRTP_PORT_LOG and channel == CRTP_CHANNEL_DATA:
            return self._parse_log_data_packet(data)
        elif port == CRTP_PORT_LOG and channel == CRTP_CHANNEL_TOC:
            return self._parse_log_toc_response(data)
                
        return False
        
    def _parse_log_toc_response(self, data):
        if len(data) < 2 or data[1] != TOC_CMD_GET_ITEM: return False
        
        try:
            item_id = data[2]
            log_type = data[3]
            
            null_pos = data.find(b'\x00', 4) 
            if null_pos == -1: return False
            group_name = data[4:null_pos].decode('ascii')
            
            var_name_start = null_pos + 1
            var_name_end = data.find(b'\x00', var_name_start)
            if var_name_end == -1: return False
            var_name = data[var_name_start:var_name_end].decode('ascii')
            
            full_name = f"{group_name}.{var_name}" if group_name else var_name

            if full_name in self.required_vars:
                self.toc_data[item_id] = {'id': item_id, 'name': full_name, 'type': log_type}
            return True
        except:
            return False
    
    def _parse_log_data_packet(self, data):
        # Mínimo 17 bytes esperados (5 encabezado + 12 datos)
        if self.log_block_id is None or len(data) < 17:
            return False
            
        try:
            block_id = data[1]
            
            if block_id == self.log_block_id:
                
                # --- DEPURACIÓN: IMPRIMIR BYTES ---
                current_time = time.time() - self.start_time

                # Imprimir el paquete completo en formato HEX
                hex_data = data.hex()
                # Paquete de 17 bytes = 34 caracteres HEX
                
                # Estructura del paquete (Índices HEX):
                # | 0-9 (5B Encabezado) | 10-17 (4B Roll) | 18-25 (4B Pitch) | 26-33 (4B Yaw) |

                print("-" * 70)
                print(f"T+{current_time:5.1f}s | Pkts: {self.packet_count+1}")
                print(f"| Encabezado (5B) | Datos Roll (4B) | Datos Pitch (4B) | Datos Yaw (4B) |")
                
                # Encabezado (5 bytes: 0 a 9 en hex)
                header_info = hex_data[0:10]
                
                # Datos de Roll (4 bytes: 10 a 17 en hex)
                roll_hex = hex_data[10:18]
                
                # Datos de Pitch (4 bytes: 18 a 25 en hex)
                pitch_hex = hex_data[18:26]
                
                # Datos de Yaw (4 bytes: 26 a 33 en hex)
                yaw_hex = hex_data[26:34] if len(hex_data) >= 34 else "..."

                # Imprimir en consola con separación
                print(f"| {header_info:<15} | {roll_hex:<15} | {pitch_hex:<17} | {yaw_hex:<15} |")

                # --- Decodificación CRTP: Empezamos en el índice 5 (log_data = data[5:]) ---
                log_data = data[5:]
                roll_rad, pitch_rad, yaw_raw = struct.unpack('<fff', log_data[0:12])
                
                # Roll y Pitch suelen venir en radianes, por lo que se convierten a grados.
                # roll_deg = math.degrees(roll_rad)
                roll_deg = roll_rad
                # pitch_deg = math.degrees(pitch_rad)
                pitch_deg = pitch_rad
                
                # Yaw ya está en grados (o una escala angular).
                # Usamos el valor float crudo para obtener el valor correcto (eliminamos math.degrees())
                yaw_deg = yaw_raw 
                
                print(f"| Decodificación (Índice 5): R:{roll_deg:+6.1f}° | P:{pitch_deg:+6.1f}° | Y:{yaw_deg:+6.1f}°")
                
                return True

        except struct.error as se:
             print(f"⚠️ Error de struct (alineación/bytes faltantes): {se}")
        except Exception as e:
             print(f"⚠️ Error general en parsing: {e}")
             
        return False
        
def test_receiver_debug():
    receiver = DroneDataReceiver()
    
    try:
        if receiver.start():
            print("🚀 Esperando 5 segundos para que el drone inicialice el Wi-Fi/CRTP...")
            time.sleep(5) 
            
            if not receiver.get_log_toc(): return

            # Frecuencia de log a 25Hz (4 * 10ms)
            receiver.create_and_start_log_block(period_10ms=4) 
            
            print("\n🎯 Recibiendo datos y decodificando. Verifique que los valores de Yaw sean ahora razonables.")
            
            # Bucle de espera
            while True:
                time.sleep(0.5) 
                
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo...")
    finally:
        receiver.stop()

if __name__ == "__main__":
    test_receiver_debug()