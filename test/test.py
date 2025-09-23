#!/usr/bin/env python3
"""
Script de prueba para ESP-Drone en modo Station
Necesitas conocer la IP que el router asignó al ESP32
"""

import socket
import time
import threading
import sys
import struct

# Constantes para la comunicación CRTP
CRTP_PORT_CMD = 0
CRTP_PORT_LOG = 5
CRTP_CHANNEL_TEST = 0

class DroneTester:
    def __init__(self, drone_ip=None, port=1998):
        self.drone_ip = drone_ip
        self.port = port
        self.sock = None
        self.running = False
        self.rx_thread = None
        
    def create_crtp_packet(self, port, channel, data):
        """Crea un paquete CRTP con una cabecera y datos."""
        header = (port << 4) | (channel & 0x0F)
        packet = bytes([header]) + data
        return packet

    def parse_crtp_packet(self, raw_data):
        """Analiza un paquete CRTP y devuelve la cabecera y los datos."""
        if not raw_data:
            return None, None
        
        header = raw_data[0]
        port = (header >> 4) & 0x0F
        channel = header & 0x0F
        payload = raw_data[1:]
        
        return (port, channel), payload

    def send_test_packet(self):
        """Envía un paquete de prueba al drone."""
        message = "Hello from Python!"
        payload = message.encode('utf-8')
        packet = self.create_crtp_packet(CRTP_PORT_CMD, CRTP_CHANNEL_TEST, payload)
        
        try:
            self.sock.sendto(packet, (self.drone_ip, self.port))
            print("🚀 Paquete enviado. Esperando respuesta...")
            return True
        except Exception as e:
            print(f"❌ Error al enviar paquete: {e}")
            return False

    def receive_packet(self):
        """Recibe y procesa un paquete del drone."""
        try:
            data, addr = self.sock.recvfrom(1024)
            (port, channel), payload = self.parse_crtp_packet(data)
            
            print("✅ Paquete recibido del drone:")
            print(f"   - IP: {addr[0]}")
            print(f"   - Puerto: {port}, Canal: {channel}")
            print(f"   - Datos: {payload.decode('utf-8')}")
            return True
        except socket.timeout:
            print("⏳ Tiempo de espera agotado. No se recibió respuesta.")
        except Exception as e:
            print(f"❌ Error al recibir paquete: {e}")
        return False

    def connect(self, ip=None):
        """Conectar al drone"""
        if ip:
            self.drone_ip = ip
            
        if not self.drone_ip:
            print("🔍 No se especificó IP del drone. Saliendo.")
            return False
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(5.0) # Aumenta el timeout para la conexión inicial
            print(f"✅ Conectado al drone en {self.drone_ip}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ Error conectando: {e}")
            return False

    # --- Métodos que faltaban en tu código ---
    def run_comprehensive_test(self):
        """Ejecuta un test completo de enviar/recibir."""
        if not self.connect():
            return

        print("---")
        print("Iniciando prueba de ping...")
        if self.send_test_packet():
            self.receive_packet()
        print("---")
        print("Prueba completa.")

    def interactive_mode(self):
        """Modo interactivo para enviar comandos manualmente."""
        if not self.connect():
            return
            
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop)
        self.rx_thread.daemon = True
        self.rx_thread.start()
        
        print("\n--- MODO INTERACTIVO ---")
        print("Escribe 'exit' para salir.")
        
        try:
            while self.running:
                message = input("> ")
                if message.lower() == "exit":
                    self.running = False
                    break
                
                payload = message.encode('utf-8')
                packet = self.create_crtp_packet(CRTP_PORT_CMD, CRTP_CHANNEL_TEST, payload)
                self.sock.sendto(packet, (self.drone_ip, self.port))
                time.sleep(0.1) # Pequeño retraso para evitar inundar
        except KeyboardInterrupt:
            self.running = False
        finally:
            print("Saliendo del modo interactivo...")
            self.sock.close()
    
    def _receive_loop(self):
        """Bucle de recepción en un hilo separado."""
        self.sock.settimeout(0.5) # Timeout corto para el bucle
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                (port, channel), payload = self.parse_crtp_packet(data)
                print(f"\n[RX] - Port:{port} Channel:{channel} -> {payload.decode('utf-8', errors='ignore')}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Error en hilo de recepción: {e}")
                self.running = False

def main():
    """Función principal."""
    drone_ip = None
    if len(sys.argv) > 1:
        drone_ip = sys.argv[1]
    
    tester = DroneTester(drone_ip)
    
    print("🚀 TESTER PARA ESP-DRONE (MODO STATION)")
    print("========================================")
    
    if not drone_ip:
        print("⚠️  No se especificó IP del drone")
        print("   Usa: python test_drone.py <IP_DEL_DRONE>")
        print("   Ejemplo: python test_drone.py 192.168.1.100")
        print()
        print("   Para encontrar la IP del drone:")
        print("   1. Mira los logs del ESP32 después de conectarse al Wi-Fi")
        print("   2. Revisa la lista de dispositivos en tu router")
        print("   3. Usa herramientas como 'arp -a' o apps de escaneo de red")
        return
    
    if len(sys.argv) > 2 and sys.argv[2] == "interactive":
        tester.interactive_mode()
    else:
        tester.run_comprehensive_test()

if __name__ == "__main__":
    main()