# compass_pico.py
import math
import time
import network
import socket
import _thread
from machine import Pin, I2C, PWM
import ujson

led = Pin("LED", Pin.OUT)

class CompassPico:
    def erase_cmps12_calibration(self):
        """Erase the stored calibration profile on the CMPS12 (write 0xE0, 0xE5, 0xE2 to 0x00 with 20ms delay)"""
        try:
            #for b in [0xE0, 0xE5, 0xE2]:
            self.abuff[0] = 0xe0
            self.bbuff[0] = 0xe5
            self.cbuff[0] = 0xe2

            self.i2c.writeto_mem(self.cmps12_addr, 0x00, self.abuff)
            time.sleep(0.2)
            self.i2c.writeto_mem(self.cmps12_addr, 0x00, self.bbuff)
            time.sleep(0.2)
            self.i2c.writeto_mem(self.cmps12_addr, 0x00, self.cbuff)
            time.sleep(0.2)
        except Exception as e:
            print(f"CMPS12 calibration erase failed: {e}")

    def read_calibration_status(self):
        """Read and decode calibration status from CMPS12 register 0x1E, retry if 0xFF or error"""
        try:
            reg = self.i2c.readfrom_mem(self.cmps12_addr, 0x1E, 1)
            print(f"CMPS12 calibration status register: {reg:08b}")
            # Bits: [7:6]=sys, [5:4]=gyro, [3:2]=accel, [1:0]=mag
            mag = reg & 0x03
            sys = (reg >> 6) & 0x03
            gyro = (reg >> 4) & 0x03
            accel = (reg >> 2) & 0x03
            return sys, gyro, accel, mag
        except Exception as e:
            print(f"Error: {e} CMPS12 not responding or calibration status invalid after retries.")
        return None, None, None, None
    
    def __init__(self):
        
        print("---------------- CompassPico initialisation starting\n")

        self.abuff = bytearray(1)
        self.bbuff = bytearray(1)
        self.cbuff = bytearray(1)
        

        # I2C setup for CMPS12
        self.i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
        self.cmps12_addr = 0x60
        self.heading_buffer = []

        # I2C1 setup for MCP4725 DACs (sin/cos)
        self.i2c_dac = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
        self.dac_sin_addr = 0x60
        self.dac_cos_addr = 0x61

        # Initialize to 0° heading (North)
        self.write_mcp4725(self.dac_sin_addr, 2048)  # mid-scale
        self.write_mcp4725(self.dac_cos_addr, 4095)  # max-scale

        self.heading = 0.0
        self.wifi_connected = False

        # Erase calibration profile on startup
        time.sleep(0.5)  # Ensure I2C is stable
        self.erase_cmps12_calibration()
        time.sleep(0.5)  # Wait after erase for device to reset
        self.erase_cmps12_calibration()
        time.sleep(0.5)  # Wait for device to reset
        sys, gyro, accel, mag = self.read_calibration_status()
        print(f"Power Up State 1 = Calibration: (System:{sys} Gyro:{gyro} Accelerometer:{accel} Magnetometer:{mag})")
        sys, gyro, accel, mag = self.read_calibration_status()
        print(f"Power Up State 2 = Calibration: (System:{sys} Gyro:{gyro} Accelerometer:{accel} Magnetometer:{mag})")
        print("---------------- CompassPico initialisation complete\n")



    def write_mcp4725(self, addr, value):
        """Write a 12-bit value to MCP4725 DAC at given address"""
        value = max(0, min(4095, int(value)))
        # MCP4725 expects: [C2 C1 C0 X X X X X] [D11 D10 D9 D8 D7 D6 D5 D4] [D3 D2 D1 D0 X X X X]
        data = bytearray(3)
        data[0] = 0x40  # Write DAC register
        data[1] = (value >> 4) & 0xFF
        data[2] = (value & 0x0F) << 4
        self.i2c_dac.writeto(addr, data)
        
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
        """Read heading from CMPS12 and apply LPF over last 10 samples"""
        try:
            data = self.i2c.readfrom_mem(self.cmps12_addr, 0x02, 2)
            heading_raw = (data[0] << 8) | data[1]
            self.heading = heading_raw / 10.0
            # Add to buffer and keep last 10
            self.heading_buffer.append(self.heading)
            if len(self.heading_buffer) > 10:
                self.heading_buffer.pop(0)
            # Return average
            #return sum(self.heading_buffer) / len(self.heading_buffer)
            # or return raw
            return self.heading
        except Exception:
            return None
  
    def output_sin_cos(self, heading_deg):
        """Output sin/cos as analog voltages via MCP4725 DACs (0-3.3V)"""
        heading_rad = math.radians(heading_deg)
        sin_val = math.sin(heading_rad)
        cos_val = math.cos(heading_rad)

        # Scale: -1..1 -> 0..4095 (12-bit DAC)
        sin_dac = int(2048 + (2047 * sin_val))
        cos_dac = int(2048 + (2047 * cos_val))

        # Clamp
        sin_dac = max(0, min(4095, sin_dac))
        cos_dac = max(0, min(4095, cos_dac))

        self.write_mcp4725(self.dac_sin_addr, sin_dac)
        self.write_mcp4725(self.dac_cos_addr, cos_dac)
        
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
                            print(f"[NMEA] Send error: {send_err}")
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
        while True:
            heading = self.read_cmps12()
            if heading is not None:
                self.output_sin_cos(heading)
                sys, gyro, accel, mag = self.read_calibration_status()
                print(f"Heading: {heading:.1f} Deg Mag | Calib (SYS:{sys} G:{gyro} A:{accel} M:{mag})")
            
            time.sleep(0.1)  # 10Hz update rate
            # Here we toggle the Led on and off for 0.2 seconds on and 5 seconds off
            if led_count  > 20:
                led_count = 0;
                led.value(1)
            else:
                led_count += 1
                if (led_count == 1):
                    led.value(0)   # Turn LED off

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
                time.sleep(5)

# Main execution
if __name__ == "__main__":
    
    time.sleep(1)  # Allow time for system to stabilize
    compass = CompassPico()
    
    # Connect to WiFi (replace with your credentials)
    if compass.connect_wifi("baba-40", "badbadbadface"):
        print("Starting compass system with WiFi")
    else:
        print("Starting compass system without WiFi")
    
    # Run the system
    compass.run()