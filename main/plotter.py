import serial
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import sys

# --- Configuration ---
SERIAL_PORT = 'COM3'  # Default COM port, can be overridden by command line argument
BAUD_RATE = 115200
MAX_DATA_POINTS = 100  # Number of data points to display on the graph

# --- Data Storage ---
# Using deque for efficient fixed-size storage that automatically discards old data
time_points = deque(maxlen=MAX_DATA_POINTS)
roll_data = deque(maxlen=MAX_DATA_POINTS)
pitch_data = deque(maxlen=MAX_DATA_POINTS)

# --- Regex for parsing serial data ---
# Example log: I (44613) Mpu6050: Roll: -1.17, Pitch: 0.73, Temp: 41.89
data_pattern = re.compile(r"Roll: (-?\d+\.\d+), Pitch: (-?\d+\.\d+)")

# --- Plot Setup ---
fig, ax = plt.subplots()
line_roll, = ax.plot([], [], 'r-', label='Roll')
line_pitch, = ax.plot([], [], 'b-', label='Pitch')

def init_plot():
    """Initializes the plot aesthetics."""
    ax.set_title('MPU6050 Roll & Pitch Real-Time Data')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (°)')
    ax.legend()
    ax.grid(True)
    line_roll.set_data([], [])
    line_pitch.set_data([], [])
    return line_roll, line_pitch

def update_plot(frame, ser):
    """
    This function is called by FuncAnimation to update the plot.
    It reads a line from serial, parses it, and updates the plot data.
    """
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            match = data_pattern.search(line)
            if match:
                roll = float(match.group(1))
                pitch = float(match.group(2))

                # Use a simple counter for the x-axis for consistent spacing
                current_time = len(time_points)

                time_points.append(current_time)
                roll_data.append(roll)
                pitch_data.append(pitch)

                line_roll.set_data(time_points, roll_data)
                line_pitch.set_data(time_points, pitch_data)

                # Adjust plot limits
                ax.relim()
                ax.autoscale_view()

    except (serial.SerialException, UnicodeDecodeError) as e:
        print(f"Error reading from serial port: {e}")
        # Attempt to close and reopen might be an option here, or just exit.

    return line_roll, line_pitch

def main():
    """Main function to set up serial and start plotting."""
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"Connected to {port} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}: {e}")
        sys.exit(1)

    # Create the animation
    ani = animation.FuncAnimation(fig, update_plot, fargs=(ser,),
                                  init_func=init_plot,
                                  blit=True, interval=50, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        print("Plotting stopped by user.")
    finally:
        # Ensure the serial port is closed on exit
        if ser.is_open:
            ser.close()
            print(f"Serial port {port} closed.")

if __name__ == '__main__':
    main()
