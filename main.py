# compass_pico.py
import machine
import math
import time
import network
import socket
import _thread
from machine import Pin, I2C, PWM
import ujson

led = Pin("LED", Pin.OUT)

class CompassPico:
    def __init__(self):
        # I2C setup for CMPS12 (bus 0)
        self.i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
        self.cmps12_addr = 0x60

        # I2C setup for MCP4725 DACs (bus 1)
        self.i2c_dac = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
        self.dac_sin_addr = 0x60
        self.dac_cos_addr = 0x61

        self.heading = 0.0
        self.wifi_connected = False

    def save_calibration(self):
        # Save calibration data to non-volatile storage
        # To store a profile write the following to the command register 0xF0, 0xF5, 0xF6 with a 20ms delay
        # after each of the three bytes.
        self.i2c.writeto_mem(self.cmps12_addr, 0x00, b'\xF0')
        time.sleep(0.02)
        self.i2c.writeto_mem(self.cmps12_addr, 0x00, b'\xF5')
        time.sleep(0.02)
        self.i2c.writeto_mem(self.cmps12_addr, 0x00, b'\xF6')
        time.sleep(0.02)
        print("Calibration data saved ----------============--------------")

    #function to read the status register at address 0x1e
    def read_status_register(self):
        status = self.i2c.readfrom_mem(self.cmps12_addr, 0x1E, 1)[0]
        return status & 0x03

    def connect_wifi(self, ssid, password):
        """Connect to WiFi network"""
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(ssid, password)
        
        # Wait for connection
        timeout = 10
        while timeout > 0:
            if wlan.status() < 0 or wlan.status() >= 3:
                break
            timeout -= 1
            time.sleep(1)
            
        if wlan.status() != 3:
            print("WiFi connection failed")
            return False
            
        print(f"WiFi connected: {wlan.ifconfig()[0]}")
        self.wifi_connected = True
        return True
        
    def read_cmps12(self):
        """Read heading from CMPS12"""
        try:
            # Read 16-bit heading register
            data = self.i2c.readfrom_mem(self.cmps12_addr, 0x02, 2)
            heading_raw = (data[0] << 8) | data[1]
            self.heading = heading_raw / 10.0
            return self.heading
        except:
            return None
            
    def output_sin_cos(self, heading_deg):
        """Output sin/cos as analog voltages via MCP4725 DACs on I2C(1)"""
        heading_rad = math.radians(heading_deg)
        sin_val = math.sin(heading_rad)
        cos_val = math.cos(heading_rad)

        # MCP4725 is a 12-bit DAC (0-4095), Vout = (value/4095) * Vcc
        # Output range: 0V to Vcc (typically 3.3V)
        # Center at 2048, amplitude ±1638 (40% of full scale)
        sin_dac = int(2048 + 1638 * sin_val)
        cos_dac = int(2048 + 1638 * cos_val)
        # Clamp to valid range
        sin_dac = max(0, min(4095, sin_dac))
        cos_dac = max(0, min(4095, cos_dac))

        # Write to MCP4725: [0x40, MSB, LSB]
        sin_bytes = bytes([0x40, (sin_dac >> 4) & 0xFF, (sin_dac & 0x0F) << 4])
        cos_bytes = bytes([0x40, (cos_dac >> 4) & 0xFF, (cos_dac & 0x0F) << 4])
        self.i2c_dac.writeto(self.dac_sin_addr, sin_bytes)
        self.i2c_dac.writeto(self.dac_cos_addr, cos_bytes)
        
    def generate_nmea_hdt(self):
        """Generate NMEA HDT sentence"""
        # hdg is integer of heading
        hdg = int(self.heading) % 360
        sentence = f"IIHDM,{hdg:.1f},T"
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"${sentence}*{checksum:02X}\r\n"
        
    def nmea_server(self):
        """Simple TCP server for NMEA data with extra debug output"""
        try:
            print("[NMEA] Creating socket...")
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print("[NMEA] Binding to 0.0.0.0:10110...")
            server.bind(('0.0.0.0', 10110))
            print("[NMEA] Listening...")
            server.listen(1)
            print("NMEA server listening on port 10110")

            while True:
                try:
                    print("[NMEA] Waiting for client connection...")
                    client, addr = server.accept()
                    print(f"NMEA client connected: {addr}")

                    while True:
                        nmea_data = self.generate_nmea_hdt()
                        try:
                            client.send(nmea_data.encode())
                            #print(f"Sent NMEA: {nmea_data.strip()}")
                        except Exception as send_err:
                            #print(f"[NMEA] Send error: {send_err}")
                            break
                        time.sleep(1)  # 1Hz NMEA rate

                except OSError as accept_err:
                    print(f"[NMEA] OSError in accept or client loop: {accept_err}")
                    if 'client' in locals():
                        try:
                            client.close()
                        except:
                            pass
                        print("NMEA client disconnected")
                    continue
                except Exception as e:
                    print(f"[NMEA] Unexpected error: {e}")
                    continue
        except Exception as e:
            print(f"NMEA server error (outer): {e}")
            
    def compass_loop(self):
        """Background compass reading loop"""
        led_count = 0
        saved_calibration_state = False
        calibration_save_count = 0

        while True:
            #status = self.read_status_register()
            heading = self.read_cmps12()
            if heading is not None:
                self.output_sin_cos(heading)
                #print(f"Heading: {heading:.1f} Deg Mag, status: {status}")
            time.sleep(0.1)  # 10Hz update rate
            # Here we toggle the Led on and off for 0.2 seconds on and 5 seconds off
            if led_count  > 20:
                led_count = 0;
                led.value(1)
            else:
                led_count += 1
                if (led_count == 1):
                    led.value(0)   # Turn LED off

            # Save config once after 10 seconds
            if calibration_save_count > 300:
                #self.save_calibration()
                saved_calibration_state = True
                calibration_save_count = 0
            elif saved_calibration_state is False:
                calibration_save_count += 1

    def run(self):
        """Main application loop"""
        # Start compass in background thread
        _thread.start_new_thread(self.compass_loop, ())
        
        # Run NMEA server in main thread if WiFi connected
        if self.wifi_connected:
            self.nmea_server()
        else:
            # If no WiFi, just keep main thread alive
            while True:
                time.sleep(1)

# Main execution
if __name__ == "__main__":
    compass = CompassPico()
    
    # Connect to WiFi (replace with your credentials)
    if compass.connect_wifi("baba-40", "xxxxxxxx"):
        print("Starting compass system with WiFi")
    else:
        print("Starting compass system without WiFi")
    
    # Run the system
    compass.run()
