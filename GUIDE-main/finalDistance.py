from machine import UART, Pin, PWM
import utime
import struct

# Initialize PWM outputs on GP21, GP10, GP17, GP14
pwm_pins = [21, 10, 17, 14]
pwms = []
pwm_freq = 1000  # 1 kHz frequency

for pin_num in pwm_pins:
    pwm = PWM(Pin(pin_num))
    pwm.freq(pwm_freq)
    pwm.duty_u16(0)  # Start with 0% duty cycle (buzzer off)
    pwms.append(pwm)

# Initialize UART on GP0 (TX) and GP1 (RX)
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # LiDAR
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))  # Depth Camera

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
    send_at_command("AT+DISP=5")  # Enable UART display
    utime.sleep_ms(1000)
    send_at_command("AT+UNIT=9")
    utime.sleep_ms(1000)
    send_at_command("AT+FPS=15")

def read_lidar():
    try:
        if uart0.any():
            data = uart0.read()
            if len(data) >= 7 and data.startswith(b'YY'):
                distance = struct.unpack('<H', data[2:4])[0]
                return distance
            else:
                return None
    except Exception as e:
        print(f"Error: {e}")
        return None
    return None

def get_distance_from_packet(data_bytes):
    try:
        # Ensure we have enough bytes
        if len(data_bytes) < 32:
            return None
        # Look for packet header (0x00, 0xFF)
        for i in range(len(data_bytes) - 1):
            if data_bytes[i] == 0x00 and data_bytes[i+1] == 0xFF:
                # Found packet start
                packet = data_bytes[i:]
                # Ensure we have enough bytes after header
                if len(packet) < 22:
                    return None
                # Image frame starts after header(2) + length(2) + other(16) = 20 bytes
                frame_start = i + 20
                # Process pixel data, starting from frame_start
                pixel_values = []
                for idx in range(frame_start, len(data_bytes), 10):
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
                else:
                    # No data available, continue
                    pass
        else:
            # No data available, continue
            pass
    except Exception as e:
        print(f"Camera Error: {e}")
        camera_data_buffer = bytearray()
        camera_reading_in_progress = False
    return None

def calculate_averages():
    global lidar_average_calculate, lidar_average_count, depth_average_calculate, depth_average_count
    global lidar_average, depth_average
    if lidar_average_count > 0:
        lidar_average = lidar_average_calculate / lidar_average_count
    else:
        lidar_average = -1
    if depth_average_count > 0:
        depth_average = depth_average_calculate / depth_average_count
    else:
        depth_average = -1
    # Reset counts and sums
    lidar_average_count = 0
    lidar_average_calculate = 0
    depth_average_count = 0
    depth_average_calculate = 0

class BuzzerController:
    def __init__(self, pwm_list):
        self.pwms = pwm_list  # list of PWM objects
        self.state = 'off'  # current state: 'on' or 'off'
        self.on_duration = 0  # duration in ms
        self.off_duration = 0
        self.last_change_time = utime.ticks_ms()

    def set_pattern(self, on_duration, off_duration):
        self.on_duration = on_duration
        self.off_duration = off_duration
        self.last_change_time = utime.ticks_ms()
        self.state = 'on' if on_duration > 0 else 'off'
        self.update_pwm()

    def update(self):
        current_time = utime.ticks_ms()
        elapsed_time = utime.ticks_diff(current_time, self.last_change_time)
        if self.state == 'on' and self.on_duration > 0:
            if elapsed_time >= self.on_duration:
                self.state = 'off'
                self.last_change_time = current_time
                self.update_pwm()
        elif self.state == 'off' and self.off_duration > 0:
            if elapsed_time >= self.off_duration:
                self.state = 'on'
                self.last_change_time = current_time
                self.update_pwm()

    def update_pwm(self):
        if self.state == 'on':
            for pwm in self.pwms:
                pwm.duty_u16(32768)  # 50% duty cycle (adjust as needed)
        else:
            for pwm in self.pwms:
                pwm.duty_u16(0)  # duty cycle 0% (off)

print("Initializing camera...")
send_at_command("AT")
initialize_camera()
print("Camera initialized.")

# Initialize variables for the main loop
calibration_start_time = utime.ticks_ms()
calibration_duration = 10000  # Calibration period in milliseconds (10 seconds)
calibration = True  # Flag to indicate calibration period

lidar_average_calculate = 0
lidar_average_count = 0
lidar_average = -1

depth_average_calculate = 0
depth_average_count = 0
depth_average = -1

# Initialize the BuzzerController
buzzer_controller = BuzzerController(pwms)

# Main loop
while True:
    try:
        current_time = utime.ticks_ms()
        elapsed_calibration_time = utime.ticks_diff(current_time, calibration_start_time)

        if calibration:
            if elapsed_calibration_time <= calibration_duration:
                # Collect data for calibration
                distance = read_lidar()
                if distance is not None:
                    lidar_average_calculate += distance
                    lidar_average_count += 1
                distance2 = read_camera_non_blocking()
                if distance2 is not None:
                    depth_average_calculate += distance2
                    depth_average_count += 1
            else:
                # Calibration over, calculate averages
                calculate_averages()
                print(f"Calibration completed. LiDAR average: {lidar_average} mm, Depth camera average: {depth_average} mm")
                calibration = False
        else:
            # Process new readings
            distance = read_lidar()
            if distance is not None:
                # Categorize the LiDAR distance and set buzzer pattern accordingly
                if distance <= 500:
                    category = "0.5m or less"
                    buzzer_controller.set_pattern(on_duration=0, off_duration=0)  # Buzzer on continuously
                elif distance <= 1000:
                    category = "0.5m to 1m"
                    buzzer_controller.set_pattern(on_duration=500, off_duration=500)  # On 0.5s, Off 0.5s
                elif distance <= 2000:
                    category = "1m to 2m"
                    buzzer_controller.set_pattern(on_duration=200, off_duration=800)  # On 0.2s, Off 0.8s
                else:
                    category = "beyond 2m"
                    buzzer_controller.set_pattern(on_duration=0, off_duration=0)  # Buzzer off continuously
                print(f"LiDAR distance: {distance} mm, Category: {category}")
            else:
                pass

            # Update the buzzer controller
            buzzer_controller.update()

            # Process depth camera readings if needed
            # distance2 = read_camera_non_blocking()
            # Add similar logic for the depth camera if need be idk :)

    except Exception as e:
        print(f"Error: {e}")
