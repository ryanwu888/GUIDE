from machine import UART, Pin
import utime
import struct

led = machine.Pin(25, machine.Pin.OUT)
buzzer = machine.Pin(15, machine.Pin.OUT)  # Assuming the buzzer is connected to GPIO 15

# Initialize UART on GP0 (TX) and GP1 (RX)
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # LiDAR
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))  # Depth Camera

def send_at_command(command):
    """Send configuration settings to the depth camera."""
    uart1.write(command + '\r')
    utime.sleep_ms(100)  # Wait for response
    response = b''
    while uart1.any():
        response += uart1.read()
    print(f"Command: {command}, Response: {response}")
    return response

def initialize_camera():
    """Initialize the depth camera with necessary settings."""
    send_at_command("AT")
    send_at_command("AT+ISP=1")   # Turn on ISP
    utime.sleep_ms(1000)
    send_at_command("AT+DISP=4")  # Enable UART display
    utime.sleep_ms(1000)

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

def read_camera():
    """Read distance data from the Maxisense A010 depth camera."""
    try:
        # Read data until we find a complete packet
        while True:
            if uart1.any():
                data = uart1.read(uart1.any())
                data_bytes = bytearray(data)
                # Search for the packet header (0x00, 0xFF)
                for i in range(len(data_bytes) - 1):
                    if data_bytes[i] == 0x00 and data_bytes[i+1] == 0xFF:
                        # Extract packet length
                        if i + 3 < len(data_bytes):
                            packet_length = data_bytes[i+2] << 8 | data_bytes[i+3]
                            total_length = packet_length + 2  # Include header bytes
                            if i + total_length <= len(data_bytes):
                                packet = data_bytes[i:i+total_length]
                                # Verify end of packet
                                if packet[-1] == 0xDD:
                                    # Verify checksum
                                    checksum = sum(packet[:-2]) & 0xFF
                                    if checksum == packet[-2]:
                                        print("Valid packet received")
                                        # Extract image frame data
                                        image_data = packet[20:-2]  # Exclude header, other content, checksum, and end byte
                                        # For simplicity, assume image is grayscale with 8-bit pixel values
                                        # Choose the center pixel value as 'p'
                                        if image_data:
                                            center_index = len(image_data) // 2
                                            p = image_data[center_index]
                                            print(f"Pixel value p: {p}")
                                            # Calculate distance using UNIT value
                                            unit_value = query_unit()
                                            if unit_value is not None:
                                                if unit_value != 0:
                                                    distance = p * unit_value
                                                else:
                                                    distance = (p / 5.1) ** 2
                                                print(f"Camera Distance: {distance} mm")
                                                return distance
                                            else:
                                                print("UNIT value not available")
                                                return None
                                        else:
                                            print("No image data available")
                                            return None
                                    else:
                                        print("Checksum mismatch")
                                        continue
                                else:
                                    print("End of packet byte mismatch")
                                    continue
                            else:
                                print("Incomplete packet received")
                                continue
                # If we didn't find a packet, read again
                utime.sleep_ms(10)
            else:
                utime.sleep_ms(10)
    except Exception as e:
        print(f"Camera Error: {e}")
        return None

print("Initializing camera...")
initialize_camera()
print("Camera initialized.")

# Query UNIT value once at the beginning
unit_value = query_unit()
if unit_value is None:
    print("Failed to get UNIT value. Using default UNIT=1")
    unit_value = 1.0

# Main loop
while True:
    try:
        camera_distance = read_camera()
        
        if camera_distance is not None:
            if camera_distance < 25:
                led.value(1)
                buzzer.value(1)
            else:
                led.value(0)
                buzzer.value(0)
        else:
            # If no distance data is available, turn off LED and buzzer
            led.value(0)
            buzzer.value(0)
        
        # Optionally print the distance
        print(f"Camera Distance: {camera_distance}")
        
    except Exception as e:
        print(f"Main Loop Error: {e}")
    
    utime.sleep_ms(100)  # Adjust delay as needed
