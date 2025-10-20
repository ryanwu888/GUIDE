from machine import UART, Pin
import utime
import struct

led = machine.Pin(25, machine.Pin.OUT)
buzzer = machine.Pin(15, machine.Pin.OUT)  # Assuming the buzzer is connected to GPIO 15
UNIT = 0 
# Initialize UART on GP0 (TX) and GP1 (RX)
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) #LiDAR
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5)) #Depth Camera

def send_at_command(command): # send configuration settings to the depth camera
    uart1.write(command + '\r')
    utime.sleep_ms(100)  # Wait for response
    response = b''
    if  uart1.any():
        response += uart1.read()
    print(f"Command: {command}, Response: {response}")
    return response

def initialize_camera():
    # send_at_command("AT+ISP=1")  # Turn on ISP
    utime.sleep_ms(100)
    send_at_command("AT+DISP=5")  # Enable UART display
    utime.sleep_ms(100)
    # send_at_command("AT+SAVE=1")
    utime.sleep_ms(100)
    send_at_command("AT+UNIT=9")
    utime.sleep_ms(100)
    send_at_command("AT+FPS=15")
    
def query_unit():
    """Query the UNIT value from the camera."""
    response = send_at_command("AT+UNIT?")
    # Parse the UNIT value from the response
    try:
        response_str = response.decode('utf-8', errors='ignore')
        if '=' in response_str:
            unit_str = response_str.split('=')[1].strip()
            unit_value = float(unit_str)
            print(f"UNIT Value: {unit_value}")
            return unit_value
    except Exception as e:
        print(f"Error parsing UNIT value: {e}")
    return None

last_frame_time = None  # Global variable to store the last frame time (for tracking latency)
last_frame_time_camera = None

# Query UNIT value once at the beginning
'''unit_value = query_unit()
if unit_value is None:
    print("Failed to get UNIT value. Using default UNIT=1")
    unit_value = 1.0'''

def read_lidar():
    global last_frame_time
    current_time = utime.ticks_ms()
    
    try:
        if uart0.any():
            data = uart0.read()
            
            if len(data) >= 7 and data.startswith(b'YY'):
                distance = struct.unpack('<H', data[2:4])[0]
                # print(f"Distance: {distance}",end=" ")
                
                if last_frame_time is not None:
                    time_diff = utime.ticks_diff(current_time, last_frame_time)
                    # print("in",time_diff,"ms")
                
                last_frame_time = current_time
              
              
                return distance
            else:
                # print("Incomplete or invalid frame")
                return None
    except Exception as e:
        print(f"Error: {e}")
        return None
    
    return None

'''def read_camera():
    global last_frame_time_camera
    current_time_camera = utime.ticks_ms()
    try:
        if uart1.any():
            data = uart1.read(256)
            
            print(f"Received image frame of {len(data)} bytes",end=" ")
            
            if last_frame_time is not None:
                    time_diff_camera = utime.ticks_diff(current_time_camera, last_frame_time_camera)
                    print("in",time_diff_camera,"ms")
                
            last_frame_time_camera = current_time_camera
            
            return data
    except Exception as e:
        print(f"Error: {e}")
        return None
'''
def get_distance_from_packet(data_bytes):
    """
    Extract distance from packet using formulas from section 1.8
    """
    try:
        # Make sure we have enough bytes
        if len(data_bytes) < 32:
            return None
            
        # Look for packet header (0x00, 0xFF)
        for i in range(1,len(data_bytes) - 1):
            if data_bytes[i] == 0x00 and data_bytes[i+1] == 0xFF:
                # Found packet start
                packet = data_bytes[i:]
                
                # Make sure we have enough bytes after header
                if len(packet) < 22:  # 2(header) + 2(length) + 16(other) + 1(check) + 1(end)
                    return None
                
                # Image frame starts after header(2) + length(2) + other(16) = 20 bytes
                frame_start = i + 20
                
                # Now process pixel data, starting from frame_start
                pixel_values = []
                for idx in range(frame_start, len(data_bytes), 3):
                    p = data_bytes[idx]
                    distance = int((p / 5.1) ** 2)
                    pixel_values.append(distance)
                
                if pixel_values:
                    avg_distance = sum(pixel_values) / len(pixel_values)
                    print(f"Packet found at {i}, Average Distance: {avg_distance}")
                    return avg_distance
                else:
                    return None
                
        return None

    except Exception as e:
        print(f"Error in get_distance_from_packet: {e}")
        return None


def read_camera():
    """Read distance data from the Maxisense A010 depth camera."""
    try:
        if uart1.any():
            # Read enough bytes for a complete packet
            data = uart1.read()
            if data is None or len(data) < 32:
                return None
                
            data_bytes = bytearray(data)
            # print(f"Raw bytes: {[hex(x) for x in data_bytes]}")  # Debug print
           
            distance = get_distance_from_packet(data_bytes)
            return distance
            
    except Exception as e:
        print(f"Camera Error: {e}")
        return None
    
    return None


print("Initializing camera...")
send_at_command("AT")
initialize_camera()
print("Camera initialized.")

# Main loop
while True:
    try:
        '''distance = read_lidar()
        
        if distance is not None:
            if distance < 100:
                led.value(1)
                buzzer.value(1)  # Turn on the buzzer when LED is on
            else:
                led.value(0)
                buzzer.value(0)  # Turn off the buzzer when LED is off'''
        
        distance2 = read_camera()
        if distance2 is not None:
            if distance2 < 2000:
                led.value(1)
                buzzer.value(1)  # Turn on the buzzer when LED is on
            else:
                led.value(0)
                buzzer.value(0)  # Turn off the buzzer when LED is off
        
        
    except Exception as e:
        print(f"Error: {e}")
    
    # utime.sleep_ms(100)  # Adjust delay as needed
