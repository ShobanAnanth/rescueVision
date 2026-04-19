import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import math
import sys
from collections import deque
import time

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usb" in port.device.lower() or "cu.SLAB" in port.device or "cu.wchusb" in port.device:
            return port.device
    if ports:
        return ports[0].device
    return None

def main():
    port_name = find_serial_port()
    if not port_name:
        print("No serial port found. Please connect your ESP32.")
        sys.exit(1)
        
    print(f"Connecting to {port_name} at 115200 baud...")
    try:
        ser = serial.Serial(port_name, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port {port_name}: {e}")
        sys.exit(1)

    # State tracking
    current_angle_deg = 0.0
    
    # Store points over time. Format: (time, x_world, y_world, label_type)
    # We will decay old points after a certain time, e.g., 30 seconds
    history_time_window = 15.0 # Keep points for 15 seconds
    points_history = deque()

    # Regexes
    # e.g., "compass= 350.1°  stepper=   0.00° (Δ  +0.00°)  →  assemblyHdg= 350.1°"
    hdg_re = re.compile(r"assemblyHdg=\s*([0-9.-]+)")
    # e.g., "CW 90° (angle=90.0)"
    angle_re = re.compile(r"angle=\s*([0-9.-]+)")
    
    # e.g., "📍 POS: (+ 0.12, - 1.23, + 0.05)"
    pos_re = re.compile(r"POS:\s*\(\s*([+-]?[0-9.]+)\s*,\s*([+-]?[0-9.]+)\s*,\s*([+-]?[0-9.]+)\s*\)")
    # Class labels
    ghost_re = re.compile(r"GHOST|👻")
    active_re = re.compile(r"ACTIVE|🚶")
    unconscious_re = re.compile(r"UNCONSCIOUS|💤")

    # Set up Matplotlib
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("UWB / Radar 360 POV Visualization")

    def update_plot():
        ax.clear()
        
        # Draw sensor at center (UWB position)
        ax.plot(0, 0, 'go', markersize=10, label="UWB Base / Sensor")
        
        # The sensor forward vector in world coordinates (assuming DWM orientation)
        # We negate the angle because the stepper angle is CCW-positive while our plot mapping expects CW or vice-versa
        h_rad = math.radians(-current_angle_deg)
        cone_length = 5.0
        cone_angle = math.radians(60) # half FOV
        
        c_left = h_rad + cone_angle
        c_right = h_rad - cone_angle
        
        # The sensor forward vector in world coordinates (assuming DWM orientation)
        # In dwm_geom.c local Y maps to world Y (cos) and world X (-sin).
        # Actually dwm_geom.c has:
        # world_x = cos(H) * x + sin(H) * y
        # world_y = -sin(H) * x + cos(H) * y
        lx, ly = 0, cone_length
        fw_x = math.cos(h_rad) * lx + math.sin(h_rad) * ly
        fw_y = -math.sin(h_rad) * lx + math.cos(h_rad) * ly
        ax.plot([0, fw_x], [0, fw_y], 'c--', alpha=0.5, label="Current Sensor Heading")

        now = time.time()
        
        # Filter and plot points
        valid_points = []
        while points_history and now - points_history[0][0] > history_time_window:
            points_history.popleft()
            
        ghosts_x, ghosts_y = [], []
        active_x, active_y = [], []
        unconscious_x, unconscious_y = [], []
        
        for p in points_history:
            t_stamp, x, y, label = p
            if label == "GHOST":
                ghosts_x.append(x)
                ghosts_y.append(y)
            elif label == "ACTIVE":
                active_x.append(x)
                active_y.append(y)
            elif label == "UNCONSCIOUS":
                unconscious_x.append(x)
                unconscious_y.append(y)
                
        # Scatter plots
        if ghosts_x: ax.scatter(ghosts_x, ghosts_y, c='gray', alpha=0.3, label="Noise / Ghost", s=20)
        if active_x: ax.scatter(active_x, active_y, c='green', marker='s', label="Active Track", s=60)
        if unconscious_x: ax.scatter(unconscious_x, unconscious_y, c='red', marker='x', label="Unconscious", s=60)

        # Plot limits and aesthetics
        ax.set_xlim(-6, 6)
        ax.set_ylim(-6, 6)
        ax.set_aspect('equal', 'box')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.set_xlabel("World X (meters)")
        ax.set_ylabel("World Y (meters)")
        ax.set_title(f"360° Track History (Decay {history_time_window}s)\nHdg: {current_angle_deg:.1f}°")
        ax.legend(loc='upper right')
        
        fig.canvas.draw()
        fig.canvas.flush_events()

    print("Listening to serial data... (Press Ctrl+C to exit)")
    
    last_draw_time = time.time()
    
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                # Still want to redraw occasionally for decaying points
                if time.time() - last_draw_time > 0.5:
                    update_plot()
                    last_draw_time = time.time()
                continue
                
            # Process heading
            h_match = hdg_re.search(line)
            if h_match:
                current_angle_deg = float(h_match.group(1))
                
            a_match = angle_re.search(line)
            if a_match:
                # Update angle if stepper moves explicitly reported
                current_angle_deg = float(a_match.group(1))

            # Process coordinates
            p_match = pos_re.search(line)
            if p_match:
                loc_x = float(p_match.group(1))
                loc_y = float(p_match.group(2))
                loc_z = float(p_match.group(3))
                
                label = "GHOST"
                if active_re.search(line):
                    label = "ACTIVE"
                elif unconscious_re.search(line):
                    label = "UNCONSCIOUS"
                    
                # Transform to "world" POV based on DWM orientation rule
                h_rad = math.radians(-current_angle_deg)
                
                # Using the dwm_transform_iwr_xyz math for the 2D plane:
                # world_x = cos(H)*x + sin(H)*y
                # world_y = -sin(H)*x + cos(H)*y
                world_x = math.cos(h_rad) * loc_x + math.sin(h_rad) * loc_y
                world_y = -math.sin(h_rad) * loc_x + math.cos(h_rad) * loc_y
                
                points_history.append((time.time(), world_x, world_y, label))
                
            if time.time() - last_draw_time > 0.1: # Max 10 fps draw rate
                update_plot()
                last_draw_time = time.time()

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
