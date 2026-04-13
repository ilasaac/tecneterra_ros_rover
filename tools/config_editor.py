#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import yaml
import os

# Parameter descriptions for tooltips
DESCRIPTIONS = {
    'lookahead_distance': 'Distance (metres) along the path to find the target lookahead point.',
    'default_acceptance_radius': 'A waypoint is considered reached when the rover is within this distance (m).',
    'max_speed': 'Maximum speed (m/s) in autonomous mode.',
    'min_speed': 'Minimum speed (m/s) during approach and tight turns.',
    'max_steering': 'Maximum steering effort fraction (0.0 to 1.0).',
    'control_rate': 'Control loop frequency (Hz). Should match GPS update rate.',
    'gps_timeout': 'Time (s) after which stale GPS data causes a safety halt.',
    'heading_deadband': 'Angle (degrees) within which heading error is ignored to prevent oscillation.',
    'align_threshold': 'Heading error (degrees) above which the rover stops and spins in place.',
    'stanley_k': 'Stanley lateral controller gain (higher = more aggressive correction).',
    'stanley_softening': 'Softening term for Stanley controller (prevents instability at low speed).',
    'pivot_threshold': 'Turn angle (degrees) that triggers an in-place pivot turn.',
    'pivot_approach_dist': 'Distance (m) from a pivot waypoint where the rover slows down to prepare.',
    'min_pivot_segment_m': 'Minimum segment length (m) required to trust the path bearing for pivot detection.',
    'rover_width_m': 'Physical width of the rover (m). Used for obstacle avoidance.',
    'obstacle_clearance_m': 'Additional safety buffer (m) added to rover half-width for obstacles.',
    'min_throttle_ppm': 'Minimum PPM value for forward motion to overcome motor stiction.',
    'min_steer_ppm_delta': 'Minimum steering PPM delta from center (1500) to overcome stiction.',
    'steer_coast_angle': 'Heading error (degrees) below which the steering floor is not applied.',
    'gps_accuracy_alarm_mm': 'hAcc (mm) threshold above which the rover triggers a safety halt.',
    'peer_rover_ns': 'ROS2 namespace of the other rover for proximity safety (e.g., /rv2).',
    'rover_front_corner_dist_m': 'Diagonal distance (m) from front antenna to front corners.',
    'rover_rear_corner_dist_m': 'Diagonal distance (m) from rear antenna to rear corners.',
    'rover_half_width_m': 'Half the physical width of the rover (m).',
    'proximity_slow_m': 'Inter-rover body clearance (m) to trigger speed reduction.',
    'proximity_halt_m': 'Inter-rover body clearance (m) to trigger a safety halt.',
    'proximity_estop_m': 'Inter-rover body clearance (m) to trigger an emergency stop.',
    'enable_diag_log': 'Enable logging of diagnostic data to a CSV file.',
    'diag_log_path': 'Absolute path to the diagnostic CSV log file.',
    'uart_port': 'Serial port path for the RP2040 (e.g., /dev/ttyACM0).',
    'uart_baud': 'Baud rate for RP2040 serial communication.',
    'heartbeat_interval': 'Interval (s) for the safety heartbeat to the RP2040.',
    'primary_port': 'Serial port for the primary (rear) GPS antenna.',
    'secondary_port': 'Serial port for the secondary (front) GPS antenna.',
    'baud': 'Baud rate for GPS serial communication.',
    'publish_rate': 'Frequency (Hz) at which data is published by the node.',
    'rover_id': 'Unique ID for the rover (e.g., 1 or 2).',
    'gqc_host': 'IP address or broadcast address for GQC communication.',
    'gqc_port': 'UDP port for GQC communication.',
    'bind_port': 'Local port to bind for MAVLink communication.',
    'rtsp_port': 'Port for the RTSP video stream.',
    'mount_point': 'URL mount point for the RTSP stream.',
    'camera_source': 'Video source: csi, usb, or test.',
    'csi_sensor_id': 'ID of the CSI camera sensor.',
    'width': 'Video stream width (pixels).',
    'height': 'Video stream height (pixels).',
    'framerate': 'Video stream framerate (FPS).',
    'bitrate': 'Video stream bitrate (kbps).',
    'flip_method': 'GStreamer flip method for the video stream.'
}

class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tooltip_window = None
        self.widget.bind("<Enter>", self.show_tooltip)
        self.widget.bind("<Leave>", self.hide_tooltip)

    def show_tooltip(self, event=None):
        x, y, cx, cy = self.widget.bbox("insert")
        x += self.widget.winfo_rootx() + 25
        y += self.widget.winfo_rooty() + 20
        self.tooltip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify='left',
                         background="#ffffe0", relief='solid', borderwidth=1,
                         font=("tahoma", "8", "normal"), wraplength=300)
        label.pack(ipadx=1)

    def hide_tooltip(self, event=None):
        if self.tooltip_window:
            self.tooltip_window.destroy()
        self.tooltip_window = None

class ParamEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("AgriRover Parameter Editor")
        self.root.geometry("800x800")
        
        self.file_path = "ros2_ws/src/agri_rover_bringup/config/rover1_params.yaml"
        self.data = {}
        self.entries = {}
        
        self.create_widgets()
        self.load_file()

    def create_widgets(self):
        # File selection frame
        file_frame = tk.Frame(self.root)
        file_frame.pack(fill='x', padx=10, pady=5)
        
        self.path_label = tk.Label(file_frame, text=f"File: {self.file_path}")
        self.path_label.pack(side='left')
        
        btn_open = tk.Button(file_frame, text="Open...", command=self.browse_file)
        btn_open.pack(side='right')
        
        # Scrollable area
        self.canvas = tk.Canvas(self.root)
        self.scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scroll_frame = tk.Frame(self.canvas)

        self.scroll_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        self.canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        
        # Bottom buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill='x', padx=10, pady=10)
        
        btn_save = tk.Button(btn_frame, text="Save Changes", command=self.save_file, bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        btn_save.pack(side='right')
        
        btn_reload = tk.Button(btn_frame, text="Reload", command=self.load_file)
        btn_reload.pack(side='right', padx=10)

    def browse_file(self):
        path = filedialog.askopenfilename(initialdir="ros2_ws/src/agri_rover_bringup/config",
                                          filetypes=(("YAML files", "*.yaml"), ("All files", "*.*")))
        if path:
            self.file_path = path
            self.path_label.config(text=f"File: {self.file_path}")
            self.load_file()

    def load_file(self):
        try:
            with open(self.file_path, 'r') as f:
                self.data = yaml.safe_load(f)
            self.rebuild_editor()
        except Exception as e:
            messagebox.showerror("Error", f"Could not load file: {e}")

    def rebuild_editor(self):
        # Clear existing widgets
        for widget in self.scroll_frame.winfo_children():
            widget.destroy()
        self.entries = {}
        
        if not self.data:
            return

        for node_name, content in self.data.items():
            if 'ros__parameters' not in content:
                continue
            
            node_frame = tk.LabelFrame(self.scroll_frame, text=node_name, font=("Arial", 10, "bold"), padx=10, pady=10)
            node_frame.pack(fill='x', padx=10, pady=5)
            
            params = content['ros__parameters']
            for i, (key, value) in enumerate(params.items()):
                row_frame = tk.Frame(node_frame)
                row_frame.pack(fill='x', pady=2)
                
                lbl = tk.Label(row_frame, text=key, width=30, anchor='w')
                lbl.pack(side='left')
                
                # Add tooltip
                if key in DESCRIPTIONS:
                    ToolTip(lbl, DESCRIPTIONS[key])
                
                entry = tk.Entry(row_frame)
                entry.insert(0, str(value))
                entry.pack(side='right', expand=True, fill='x', padx=5)
                
                self.entries[(node_name, key)] = (entry, type(value))

    def save_file(self):
        # Update data from entries
        for (node_name, key), (entry, original_type) in self.entries.items():
            val_str = entry.get()
            try:
                if original_type == bool:
                    val = val_str.lower() in ('true', '1', 'yes', 'on')
                elif original_type == int:
                    val = int(val_str)
                elif original_type == float:
                    val = float(val_str)
                else:
                    val = val_str
                
                self.data[node_name]['ros__parameters'][key] = val
            except ValueError:
                messagebox.showerror("Invalid Input", f"Value for {key} must be {original_type.__name__}")
                return

        try:
            with open(self.file_path, 'w') as f:
                yaml.dump(self.data, f, default_flow_style=False, sort_keys=False)
            messagebox.showinfo("Success", "Parameters saved successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Could not save file: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ParamEditor(root)
    root.mainloop()
