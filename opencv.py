# monitor_with_opencv.py
# Merge of monitor_demo.py + OpenCV lane density estimation
# pip install opencv-python numpy pyserial

import serial, time, threading, sys, argparse
import cv2, numpy as np

# ---------------- CONFIG ----------------
# Serial / Arduino
SERIAL_PORT = "COM3"    # set your port
BAUD = 115200

# Videos (set VIDEO2=None to use one video + ROIs)
VIDEO1 = "tra2.mp4"
VIDEO2 = "tra7.mp4"     # set to None to use single video with ROIs

# If using single video, define two ROIs as (x,y,w,h) in pixels
USE_SINGLE_VIDEO = False
L1_ROI = (0, 0, 640, 360)
L2_ROI = (640, 0, 640, 360)

# OpenCV processing params
FRAME_STEP = 3            # process every Nth frame (speed)
MOG_HISTORY = 300
MOG_VAR_THRESHOLD = 16
DETECT_SHADOWS = True
MIN_CONTOUR_AREA = 400
ALPHA_EMA = 0.7           # smoothing for density (0..1) higher -> smoother
SEND_INTERVAL = 1.0       # seconds between auto sends

SEND_SERIAL = True        # set False to run without Arduino (useful for testing)
SHOW_WINDOW = True        # set False to disable visual window

# ----------------------------------------

# Globals used by interactive/send threads (shared)
L1 = 0.50
L2 = 0.50
stop_flag = False

# --- Serial helpers ---
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
    try:
        ser.write(pkt.encode('utf-8'))
    except Exception as e:
        print("Serial write error:", e)

def reader(ser):
    global stop_flag
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
    global stop_flag
    while not stop_flag:
        if ser:
            send_density(ser)
        else:
            # print local send for debugging
            print(f"SEND (local): L1:{L1:.2f},L2:{L2:.2f}")
        time.sleep(SEND_INTERVAL)

# --- Interactive CLI (unchanged except uses globals) ---
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
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            cmd = "quit"
        if not cmd: continue
        parts = cmd.split()
        if parts[0] == "dens" and len(parts) == 3:
            try:
                v = float(parts[2])
                if parts[1] == 'l1':
                    L1 = max(0.0, min(1.0, v))
                else:
                    L2 = max(0.0, min(1.0, v))
                if ser:
                    send_density(ser)
                print(f"Sent densities -> L1:{L1:.2f} L2:{L2:.2f}")
            except:
                print("Invalid number")
        elif parts[0] == "mode" and len(parts) == 2:
            m = parts[1]
            if m in ("auto","trad","manual"):
                if ser:
                    ser.write(f"CMD:MODE:{m.upper()}\n".encode('utf-8'))
                print("Sent mode change:", m)
            else:
                print("Unknown mode")
        elif parts[0] == "manual" and len(parts) == 2:
            lane = parts[1].upper()
            if lane in ("L1","L2"):
                if ser:
                    ser.write(f"CMD:MANUAL:{lane}\n".encode('utf-8'))
                print("Sent manual:", lane)
            else:
                print("Use L1 or L2")
        elif parts[0] == "sim":
            if ser:
                ser.write(b"CMD:SIM_EMG\n")
            print("Sent SIM_EMG")
        elif parts[0] == "clear":
            if ser:
                ser.write(b"CMD:CLEAR_EMG\n")
            print("Sent CLEAR_EMG")
        elif parts[0] == "quit":
            stop_flag = True; break
        else:
            print("Unknown command")

# --- OpenCV lane-density thread ---
def lane_worker(video1, video2, use_single_video, roi1, roi2):
    """
    Thread that continuously estimates densities and updates global L1, L2.
    """
    global L1, L2, stop_flag

    # open captures
    cap1 = cv2.VideoCapture(video1)
    if not cap1.isOpened():
        print("ERROR: cannot open", video1); stop_flag = True; return
    if use_single_video:
        cap2 = None
    else:
        cap2 = cv2.VideoCapture(video2)
        if not cap2.isOpened():
            print("ERROR: cannot open", video2); stop_flag = True; cap1.release(); return

    fgbg1 = cv2.createBackgroundSubtractorMOG2(history=MOG_HISTORY,
                                              varThreshold=MOG_VAR_THRESHOLD,
                                              detectShadows=DETECT_SHADOWS)
    fgbg2 = cv2.createBackgroundSubtractorMOG2(history=MOG_HISTORY,
                                              varThreshold=MOG_VAR_THRESHOLD,
                                              detectShadows=DETECT_SHADOWS)

    smooth1 = L1
    smooth2 = L2
    last_send_time = time.time()

    frame_i = 0
    window_name = "lane_masks (press q to quit)"
    if SHOW_WINDOW:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while not stop_flag:
            frame_i += 1
            ret1, frame1 = cap1.read()
            if not ret1:
                cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret1, frame1 = cap1.read()
                if not ret1:
                    print("Video1 ended/unreadable"); break

            if use_single_video:
                frame2 = frame1.copy()
            else:
                ret2, frame2 = cap2.read()
                if not ret2:
                    cap2.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret2, frame2 = cap2.read()
                    if not ret2:
                        print("Video2 ended/unreadable"); break

            if (frame_i % FRAME_STEP) != 0:
                # still allow stopping via window if shown
                if SHOW_WINDOW and cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_flag = True
                continue

            # get ROI frames
            if use_single_video:
                x1,y1,w1,h1 = roi1
                x2,y2,w2,h2 = roi2
                r1 = frame1[y1:y1+h1, x1:x1+w1]
                r2 = frame1[y2:y2+h2, x2:x2+w2]
            else:
                r1 = frame1
                r2 = frame2

            # preprocess + mask
            def get_clean_mask(r, fgbg):
                g = cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
                g = cv2.GaussianBlur(g, (5,5), 0)
                mask = fgbg.apply(g)
                _, mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=2)
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cleaned = np.zeros_like(mask)
                for c in cnts:
                    if cv2.contourArea(c) >= MIN_CONTOUR_AREA:
                        cv2.drawContours(cleaned, [c], -1, 255, -1)
                return cleaned

            mask1 = get_clean_mask(r1, fgbg1)
            mask2 = get_clean_mask(r2, fgbg2)

            # compute density as foreground pixel ratio
            dens1 = cv2.countNonZero(mask1) / (mask1.shape[0]*mask1.shape[1] + 1e-9)
            dens2 = cv2.countNonZero(mask2) / (mask2.shape[0]*mask2.shape[1] + 1e-9)

            # EMA smoothing
            smooth1 = ALPHA_EMA * smooth1 + (1 - ALPHA_EMA) * dens1
            smooth2 = ALPHA_EMA * smooth2 + (1 - ALPHA_EMA) * dens2

            # update globals (thread-safe enough for our use case)
            L1 = float(smooth1)
            L2 = float(smooth2)

            # update visual window
            if SHOW_WINDOW:
                vis1 = cv2.cvtColor(mask1, cv2.COLOR_GRAY2BGR)
                vis2 = cv2.cvtColor(mask2, cv2.COLOR_GRAY2BGR)
                # annotate with density text
                cv2.putText(vis1, f"L1:{L1:.2f}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(vis2, f"L2:{L2:.2f}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                try:
                    vis = np.hstack([cv2.resize(vis1, (400,300)), cv2.resize(vis2, (400,300))])
                except:
                    vis = np.hstack([vis1, vis2])
                cv2.imshow(window_name, vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_flag = True

            # optional: small CPU-friendly sleep
            # time.sleep(0.001)

    finally:
        cap1.release()
        if 'cap2' in locals() and cap2:
            cap2.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        stop_flag = True
        print("Lane worker stopped")

# --- main launcher ---
def main():
    global stop_flag
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-serial", action="store_true", help="don't open serial port (test mode)")
    parser.add_argument("--video1", help="path to video1", default=VIDEO1)
    parser.add_argument("--video2", help="path to video2", default=VIDEO2)
    parser.add_argument("--single", action="store_true", help="use single video + ROIs")
    args = parser.parse_args()

    send_serial_flag = not args.no_serial and SEND_SERIAL
    use_single = args.single or USE_SINGLE_VIDEO
    v1 = args.video1
    v2 = None if use_single else args.video2

    ser = None
    if send_serial_flag:
        ser = open_serial(SERIAL_PORT, BAUD)

    # start serial reader + auto_sender threads (only if serial open, but keep auto_sender printing if not)
    if ser:
        rt = threading.Thread(target=reader, args=(ser,), daemon=True)
        rt.start()
    ats = threading.Thread(target=auto_sender, args=(ser,), daemon=True)
    ats.start()

    # start lane worker thread
    worker = threading.Thread(target=lane_worker, args=(v1, v2, use_single, L1_ROI, L2_ROI), daemon=True)
    worker.start()

    # run interactive in main thread (blocks until quit)
    try:
        interactive(ser)
    except KeyboardInterrupt:
        pass

    # teardown
    stop_flag = True
    print("Closing serial and exiting...")
    time.sleep(0.5)
    try:
        if ser: ser.close()
    except:
        pass
    sys.exit(0)

if __name__ == "__main__":
    main()
