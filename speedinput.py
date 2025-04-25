import serial
import time

# Function to open and return a serial connection
def open_serial_connection(port, baud_rate, timeout=1):
    try:
        ser = serial.Serial(port, baud_rate, timeout=timeout)
        time.sleep(2)  # Allow Arduino time to reset
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

# Function to send v and omega to the Arduino
def send_velocity_and_angular_velocity(ser, v, omega):
    if ser is not None:
        message = f"{v},{omega}\n"
        ser.write(message.encode())  # Send the message to Arduino
        print(f"Sent: v={v}, omega={omega}")
    else:
        print("Serial connection not established.")

# Function to read response from Arduino
def read_arduino_response(ser):
    if ser is not None and ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        print(f"Arduino says: {response}")
        return response
    return None

# Function to send the stop command to Arduino
def stop_motors(ser):
    send_velocity_and_angular_velocity(ser, 0, 0)
    print("Motors stopped.")

# Main loop to send data periodically or interactively
def main():
    arduino_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    baud_rate = 9600

    ser = open_serial_connection(arduino_port, baud_rate)

    if ser is None:
        return  # Exit if the serial connection couldn't be established

    try:
        while True:
            v = float(input("Enter linear velocity (m/s): "))   # Linear velocity in m/s
            omega = float(input("Enter angular velocity (rad/s): "))  # Angular velocity in rad/s

            send_velocity_and_angular_velocity(ser, v, omega)
            
            time.sleep(3)
            # Read response from Arduino
            read_arduino_response(ser)

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Stopping motors...")

    # Ensure that the serial connection is closed properly
    finally:
        if ser is not None:
            stop_motors(ser)  # Stop the motors before closing
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
