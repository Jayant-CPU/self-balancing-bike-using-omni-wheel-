import serial
import time
import matplotlib.pyplot as plt
import csv


PORT = 'COM3'        
BAUD_RATE = 9600    
PLOT_WINDOW = 100    
LOG_TO_CSV = True    
CSV_FILENAME = "telemetry_log.csv"
# -----------------------------


try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  
except:
    print(f"Failed to connect to {PORT}")
    exit()


plt.ion()
fig, ax = plt.subplots()
data = []


if LOG_TO_CSV:
    csv_file = open(CSV_FILENAME, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Time (s)', 'Value'])

start_time = time.time()

print("📡 Telemetry Started... Press Ctrl+C to stop.")

try:
    while True:
        line = ser.readline().decode('utf-8').strip()

        if line:
            try:
                value = float(line)
                current_time = time.time() - start_time
                data.append(value)

                
                if len(data) > PLOT_WINDOW:
                    data.pop(0)

                
                ax.clear()
                ax.plot(data, label='Sensor Value')
                ax.set_title("Live Sensor Data")
                ax.set_ylabel("Value")
                ax.set_xlabel("Samples")
                ax.grid(True)
                ax.legend()
                plt.pause(0.01)

               
                if LOG_TO_CSV:
                    csv_writer.writerow([round(current_time, 2), value])

            except ValueError:
                print(f"Ignored non-numeric line: {line}")

except KeyboardInterrupt:
    print("\n Telemetry stopped by user.")

finally:
    ser.close()
    if LOG_TO_CSV:
        csv_file.close()
        print(f" Data saved to {CSV_FILENAME}")