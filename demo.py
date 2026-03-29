# monitor_demo.py
# Minimal interactive monitor for smart_traffic_demo.ino
# pip install pyserial

import serial, time, threading, sys

SERIAL_PORT = 'COM3'   # <-- set your port
BAUD = 115200

L1 = 0.50
L2 = 0.50
stop_flag = False

def open_serial(port, baud):
    try:
        s = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)
        s.reset_input_buffer(); s.reset_output_buffer()
        print(f"--- Serial Connection Established on {port} @ {baud} ---")
        return s
    except Exception as e:
        print("Open serial error:", e); sys.exit(1)

def send_density(ser):
    pkt = f"L1:{L1:.2f},L2:{L2:.2f}\n"
    ser.write(pkt.encode('utf-8'))

def reader(ser):
    while not stop_flag:
        try:
            while ser.in_waiting:
                raw = ser.readline()
                if not raw: break
                try:
                    line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
                except:
                    line = str(raw)
                if line:
                    print("[ARDUINO]", line)
            time.sleep(0.05)
        except Exception as e:
            print("Read error:", e)
            time.sleep(0.2)

def auto_sender(ser):
    while not stop_flag:
        send_density(ser)
        time.sleep(1.0)

def interactive(ser):
    global L1, L2, stop_flag
    print("Commands:")
    print("  dens l1 0.80    -> set L1 density")
    print("  dens l2 0.20    -> set L2 density")
    print("  mode auto|trad|manual -> change mode")
    print("  manual L1|L2    -> immediate lane green for 10s")
    print("  sim             -> simulate emergency (forces L1 10s)")
    print("  clear           -> clear emergency")
    print("  quit            -> exit")
    while True:
        cmd = input("> ").strip().lower()
        if not cmd: continue
        parts = cmd.split()
        if parts[0] == "dens" and len(parts) == 3:
            try:
                v = float(parts[2])
                if parts[1] == 'l1':
                    L1 = max(0.0, min(1.0, v))
                else:
                    L2 = max(0.0, min(1.0, v))
                send_density(ser)
                print(f"Sent densities -> L1:{L1:.2f} L2:{L2:.2f}")
            except:
                print("Invalid number")
        elif parts[0] == "mode" and len(parts) == 2:
            m = parts[1]
            if m in ("auto","trad","manual"):
                ser.write(f"CMD:MODE:{m.upper()}\n".encode('utf-8'))
                print("Sent mode change:", m)
            else:
                print("Unknown mode")
        elif parts[0] == "manual" and len(parts) == 2:
            lane = parts[1].upper()
            if lane in ("L1","L2"):
                ser.write(f"CMD:MANUAL:{lane}\n".encode('utf-8'))
                print("Sent manual:", lane)
            else:
                print("Use L1 or L2")
        elif parts[0] == "sim":
            ser.write(b"CMD:SIM_EMG\n"); print("Sent SIM_EMG")
        elif parts[0] == "clear":
            ser.write(b"CMD:CLEAR_EMG\n"); print("Sent CLEAR_EMG")
        elif parts[0] == "quit":
            stop_flag = True; break
        else:
            print("Unknown command")

def main():
    ser = open_serial(SERIAL_PORT, BAUD)
    rt = threading.Thread(target=reader, args=(ser,), daemon=True)
    rt.start()
    ats = threading.Thread(target=auto_sender, args=(ser,), daemon=True)
    ats.start()
    try:
        interactive(ser)
    except KeyboardInterrupt:
        pass
    print("Closing serial")
    try: ser.close()
    except: pass

if __name__ == "__main__":
    main()
