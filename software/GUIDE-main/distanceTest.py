from machine import UART, Pin
import utime
import struct

led = machine.Pin(21, machine.Pin.OUT)
buzzer_one = machine.Pin(10, machine.Pin.OUT)
buzzer_two = machine.Pin(14, machine.Pin.OUT)
UNIT = 0 

# Initialize UART on GP0 (TX) and GP1 (RX)
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) # LiDAR
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5)) # Depth Camera

def send_at_command(command):  # Send configuration settings to the depth camera
    uart1.write(command + '\r')
    utime.sleep_ms(100)  # Wait for response
    response = b''
    if uart1.any():
        response += uart1.read()
    print(f"Command: {command}, Response: {response}")
    return response

def initialize_camera():
    utime.sleep_ms(1000)
    send_at_command("AT+DISP=4")  # Enable UART display
    utime.sleep_ms(1000)
    send_at_command("AT+UNIT=9")
    utime.sleep_ms(1000)
    send_at_command("AT+FPS=15")
        
last_frame_time = None  # Global variable to store the last frame time (for tracking latency)
last_frame_time_camera = None

def read_lidar():
    global last_frame_time
    current_time = utime.ticks_ms()
    try:
        if uart0.any():
            data = uart0.read()
            if len(data) >= 7 and data.startswith(b'YY'):
                distance = struct.unpack('<H', data[2:4])[0]
                last_frame_time = current_time
                return distance
            else:
                return None
    except Exception as e:
        print(f"Error: {e}")
        return None
    return None

def get_distance_from_packet(data_bytes):
    """
    Extract distance from packet using formulas from section 1.8
    """
    try:
        # Make sure we have enough bytes
        if len(data_bytes) < 32:
            return None
                
        # Look for packet header (0x00, 0xFF)
        for i in range(len(data_bytes) - 1):
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

# Global variables for non-blocking camera read
camera_data_buffer = bytearray()
camera_reading_in_progress = False

def read_camera_non_blocking():
    """Non-blocking read from the depth camera."""
    global camera_data_buffer, camera_reading_in_progress
    try:
        if uart1.any():
            data = uart1.read(uart1.any())
            if data is not None:
                camera_data_buffer.extend(data)
                # Check if we have a complete packet
                if len(camera_data_buffer) >= 32:
                    # Attempt to extract distance from packet
                    distance = get_distance_from_packet(camera_data_buffer)
                    if distance is not None:
                        # Reset buffer and reading flag
                        camera_data_buffer = bytearray()
                        camera_reading_in_progress = False
                        return distance
                    else:
                        # Remove processed bytes to avoid buffer growing indefinitely
                        camera_data_buffer = camera_data_buffer[-64:]  # Keep last 64 bytes
    except Exception as e:
        print(f"Camera Error: {e}")
        camera_data_buffer = bytearray()
        camera_reading_in_progress = False
    return None

print("Initializing camera...")
send_at_command("AT")
initialize_camera()
print("Camera initialized.")


while True:
    try:
        # Read LiDAR data
        distance = read_lidar()
        if distance is not None:
            # Decide whether to turn on the buzzer and LED
            if (distance <= 200 and distance >= 100):
                led.value(1)
                buzzer_one.value(1)
                buzzer_two.value(0)
            elif (distance < 100 and distance >= 50):
                led.value(1)
                buzzer_one.value(0)
                buzzer_two.value(1)
            elif (distance < 50):
                led.value(1)
                buzzer_one.value(1)
                buzzer_two.value(1)
            else:
                led.value(0)
                buzzer_one.value(0)
                buzzer_two.value(0)
    except Exception as e:
        print(f"Error: {e}")

