# compass_server.py
import asyncio
import websockets
import json
import smbus2
import math
import threading
from flask import Flask, render_template
import socket
import RPi.GPIO as GPIO
import time

class CompassSystem:
    def __init__(self):
        self.i2c = smbus2.SMBus(1)  # CMPS12 I2C connection
        self.cmps12_address = 0x60  # Default CMPS12 address
        self.heading = 0
        self.calibrated = False
        self.setup_pwm()
        
    def setup_pwm(self):
        """Initialize PWM outputs for sin/cos"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)  # Sin PWM
        GPIO.setup(19, GPIO.OUT)  # Cos PWM
        
        self.pwm_sin = GPIO.PWM(18, 1000)  # 1kHz PWM
        self.pwm_cos = GPIO.PWM(19, 1000)
        
        self.pwm_sin.start(50)  # Start at center (2.5V)
        self.pwm_cos.start(90)  # Start at max (5V) for 0° heading
        
    def read_cmps12_heading(self):
        """Read heading from CMPS12 via I2C"""
        try:
            # Read 16-bit heading register (0x02-0x03)
            self.i2c.write_byte(self.cmps12_address, 0x02)
            time.sleep(0.001)  # Small delay
            
            high_byte = self.i2c.read_byte(self.cmps12_address)
            low_byte = self.i2c.read_byte(self.cmps12_address)
            
            # Combine bytes and convert to degrees
            heading_raw = (high_byte << 8) | low_byte
            self.heading = heading_raw / 10.0  # CMPS12 returns degrees * 10
            
            return self.heading
            
        except Exception as e:
            print(f"Error reading CMPS12: {e}")
            return None
            
    def output_sin_cos(self, heading_degrees):
        """Generate sin/cos PWM outputs for Robertson autopilot"""
        heading_rad = math.radians(heading_degrees)
        
        sin_val = math.sin(heading_rad)
        cos_val = math.cos(heading_rad)
        
        # Convert to PWM duty cycle (10-90% range)
        sin_duty = 50 + 40 * sin_val
        cos_duty = 50 + 40 * cos_val
        
        # Clamp values to safe range
        sin_duty = max(10, min(90, sin_duty))
        cos_duty = max(10, min(90, cos_duty))
        
        self.pwm_sin.ChangeDutyCycle(sin_duty)
        self.pwm_cos.ChangeDutyCycle(cos_duty)
        
    def generate_nmea_hdt(self):
        """Generate NMEA HDT sentence"""
        # Calculate checksum
        sentence = f"IIHDT,{self.heading:.1f},T"
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        
        return f"${sentence}*{checksum:02X}\r\n"
        
    async def compass_loop(self):
        """Main compass reading and output loop"""
        while True:
            heading = self.read_cmps12_heading()
            if heading is not None:
                self.output_sin_cos(heading)
                
            await asyncio.sleep(0.05)  # 20Hz update rate
            
    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections for real-time data"""
        try:
            while True:
                data = {
                    'heading': self.heading,
                    'nmea': self.generate_nmea_hdt().strip(),
                    'calibrated': self.calibrated,
                    'timestamp': time.time()
                }
                
                await websocket.send(json.dumps(data))
                await asyncio.sleep(0.2)  # 5Hz web update
                
        except websockets.exceptions.ConnectionClosed:
            pass
            
    def start_nmea_server(self):
        """TCP server for NMEA data (port 10110)"""
        def nmea_server():
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('', 10110))
            server_socket.listen(5)
            
            print("NMEA server listening on port 10110")
            
            while True:
                try:
                    client_socket, address = server_socket.accept()
                    print(f"NMEA client connected: {address}")
                    
                    while True:
                        nmea_data = self.generate_nmea_hdt()
                        client_socket.send(nmea_data.encode())
                        time.sleep(1)  # 1Hz NMEA rate
                        
                except Exception as e:
                    print(f"NMEA server error: {e}")
                    if 'client_socket' in locals():
                        client_socket.close()
                        
        thread = threading.Thread(target=nmea_server, daemon=True)
        thread.start()

# Main execution
if __name__ == "__main__":
    compass = CompassSystem()
    compass.start_nmea_server()
    asyncio.run(compass.compass_loop())
    asyncio.run(compass.websocket_handler())
    
