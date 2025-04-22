import serial
import matplotlib.pyplot as plt
import re  # Regular expressions for parsing
import time
import csv  # For saving data to CSV

# --- Configuration ---
SERIAL_PORT = '/dev/cu.usbmodem1101'  # CHANGE THIS to your Pico's serial port
# Linux: usually /dev/ttyACM0 or /dev/ttyACM1 etc. Check with 'dmesg | grep tty' after plugging in.
# Windows: usually COM3, COM4, etc. Check in Device Manager.
# macOS: usually /dev/cu.usbmodemXXXX or /dev/tty.usbmodemXXXX
BAUD_RATE = 115200
CSV_FILENAME = 'pwm_rpm_wind_data.csv'  # Updated CSV filename
PLOT_FILENAME_RPM = 'pwm_vs_rpm_plot.png'  # Plot for PWM vs RPM
PLOT_FILENAME_WIND = 'pwm_vs_wind_plot.png'  # Plot for PWM vs Wind
PLOT_FILENAME_COMBINED = 'pwm_vs_rpm_wind_combined_plot.png'  # Combined plot

# --- Data Storage ---
# Use simple lists to store all collected data
pwm_values = []
rpm_values = []
wind_mph_values = []
temp_c_values = []

# --- Main Execution ---
ser = None  # Initialize ser to None
print("Starting Serial Data Logger (RPM & Wind)...")
print(f"Will read from {SERIAL_PORT} at {BAUD_RATE} baud.")
print(f"Press Ctrl+C to stop logging and generate CSV/Plots.")

try:
    print(f"Attempting to connect to serial port {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Serial port opened successfully.")
    time.sleep(2)  # Give connection and Pico time to settle

    ser.reset_input_buffer()
    print("Input buffer cleared. Starting data acquisition...")
    print("-" * 30)
    print("Waiting for data (Format: PWM:val,RPM:val,WindMPH:val,TempC:val)...")

    # --- Data Acquisition Loop ---
    while True:  # Loop indefinitely until interrupted
        try:
            if ser.in_waiting > 0:
                line_bytes = ser.readline()
                line_str = line_bytes.decode('utf-8', errors='ignore').strip()

                # Updated regex to capture all four values
                match = re.search(
                    r'PWM:(\d+),RPM:([\d.-]+),WindMPH:([\d.-]+),TempC:([\d.-]+)', line_str)
                # Note: [\d.-]+ allows for optional decimal points and negative signs (e.g., for temp)

                if match:
                    try:
                        pwm_val = int(match.group(1))
                        rpm_val = float(match.group(2))
                        wind_val = float(match.group(3))
                        temp_val = float(match.group(4))

                        # Append new data to lists
                        pwm_values.append(pwm_val)
                        rpm_values.append(rpm_val)
                        wind_mph_values.append(wind_val)
                        temp_c_values.append(temp_val)

                        # Print parsed data to console for feedback
                        print(
                            f"Data -> PWM: {pwm_val} µs, RPM: {rpm_val:.2f}, Wind: {wind_val:.2f} MPH, Temp: {temp_val:.2f} C")

                    except ValueError:
                        print(
                            f"Warn: Could not parse values from matched line: {line_str}")
                    except Exception as e:
                        print(f"Warn: Error processing data point: {e}")
                # else:
                    # Optional: Print non-matching lines for debugging
                    # if line_str and not line_str.startswith("---") and not line_str.startswith("Sweep:") :
                    #    print(f"Info: Skipping non-data line: {line_str}")

            time.sleep(0.01)  # Small delay

        except serial.SerialException as e:
            print(f"\nError: Serial communication error: {e}")
            print("Stopping data acquisition.")
            break
        except Exception as e:
            print(f"\nError: An unexpected error occurred during reading: {e}")
            # break # Optionally stop on other errors

except KeyboardInterrupt:
    print("\n" + "-" * 30)
    print("Ctrl+C detected. Stopping data acquisition...")

except serial.SerialException as e:
    print(f"Fatal Error: Could not open serial port {SERIAL_PORT}.")
    print(e)
except FileNotFoundError:
    print(f"Fatal Error: Serial port {SERIAL_PORT} not found.")
except Exception as e:
    print(f"Fatal Error: An unexpected error occurred on startup: {e}")

# --- Data Processing and Saving ---
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")

    print("-" * 30)
    if not pwm_values:
        print("No data collected. Exiting.")
    else:
        print(f"Collected {len(pwm_values)} data points.")

        # --- Save Data to CSV ---
        try:
            print(f"Saving data to {CSV_FILENAME}...")
            with open(CSV_FILENAME, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write header with new columns
                writer.writerow(['PWM (us)', 'RPM', 'Wind (MPH)', 'Temp (C)'])
                # Write data rows
                for pwm, rpm, wind, temp in zip(pwm_values, rpm_values, wind_mph_values, temp_c_values):
                    writer.writerow([pwm, rpm, wind, temp])
            print(f"Data successfully saved to {CSV_FILENAME}")
        except IOError as e:
            print(f"Error: Could not write to CSV file {CSV_FILENAME}: {e}")
        except Exception as e:
            print(
                f"Error: An unexpected error occurred during CSV writing: {e}")

        # --- Generate and Save Final Plots ---
        try:
            print(f"Generating final plots...")

            # Plot 1: PWM vs RPM
            fig1, ax1 = plt.subplots(figsize=(10, 6))
            ax1.plot(pwm_values, rpm_values, 'bo-', label='RPM', markersize=4)
            ax1.set_xlabel("PWM Pulse Width (µs)")
            ax1.set_ylabel("Motor Speed (RPM)", color='blue')
            ax1.set_title("PWM vs Motor RPM")
            ax1.tick_params(axis='y', labelcolor='blue')
            ax1.grid(True)
            ax1.legend(loc='upper left')
            print(f"Saving RPM plot to {PLOT_FILENAME_RPM}...")
            plt.savefig(PLOT_FILENAME_RPM)
            # plt.show() # Show plot 1

            # Plot 2: PWM vs Wind Speed
            fig2, ax2 = plt.subplots(figsize=(10, 6))
            ax2.plot(pwm_values, wind_mph_values, 'gs-',
                     label='Wind Speed', markersize=4)  # Green squares
            ax2.set_xlabel("PWM Pulse Width (µs)")
            ax2.set_ylabel("Wind Speed (MPH)", color='green')
            ax2.set_title("PWM vs Wind Speed")
            ax2.tick_params(axis='y', labelcolor='green')
            ax2.grid(True)
            ax2.legend(loc='upper left')
            print(f"Saving Wind plot to {PLOT_FILENAME_WIND}...")
            plt.savefig(PLOT_FILENAME_WIND)
            # plt.show() # Show plot 2

            # Plot 3: Combined PWM vs RPM and Wind Speed (using secondary axis)
            fig3, ax3 = plt.subplots(figsize=(10, 6))

            # Plot RPM on primary axis
            color1 = 'tab:blue'
            ax3.set_xlabel("PWM Pulse Width (µs)")
            ax3.set_ylabel("Motor Speed (RPM)", color=color1)
            line1, = ax3.plot(pwm_values, rpm_values, color=color1,
                              marker='o', linestyle='-', markersize=4, label='RPM')
            ax3.tick_params(axis='y', labelcolor=color1)
            ax3.grid(True, axis='y', linestyle='--',
                     alpha=0.7)  # Grid for primary axis

            # Create secondary axis sharing the same x-axis
            ax4 = ax3.twinx()
            color2 = 'tab:green'
            ax4.set_ylabel("Wind Speed (MPH)", color=color2)
            line2, = ax4.plot(pwm_values, wind_mph_values, color=color2,
                              marker='s', linestyle='--', markersize=4, label='Wind Speed')
            ax4.tick_params(axis='y', labelcolor=color2)

            # Add title and legend
            ax3.set_title("PWM vs RPM and Wind Speed")
            # Combine legends from both axes
            lines = [line1, line2]
            labels = [l.get_label() for l in lines]
            ax3.legend(lines, labels, loc='upper center')

            fig3.tight_layout()  # Adjust layout to prevent labels overlapping
            print(f"Saving Combined plot to {PLOT_FILENAME_COMBINED}...")
            plt.savefig(PLOT_FILENAME_COMBINED)
            print("Plots generated and saved.")

            # Show the combined plot last
            plt.show()

        except Exception as e:
            print(f"Error: An unexpected error occurred during plotting: {e}")

    print("Script finished.")
