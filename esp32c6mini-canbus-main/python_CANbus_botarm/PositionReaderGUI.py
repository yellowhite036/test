import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import time
import json
import os
import atexit
import threading
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable

class PositionReaderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Position Reader - Motor Position Transmitter")
        self.root.geometry("800x600")
        
        # Connection variables
        self.reader_serial_port = None
        self.reader_motor_control = None
        self.target_serial_port = None
        self.target_motor_control = None
        
        # Motor storage
        self.reader_motors = {}
        self.target_motors = {}
        
        # Motor mapping (reader_id -> target_id)
        self.motor_mapping = {}
        
        # Pending motor configurations (loaded from config but waiting for connection)
        self.pending_reader_configs = []
        self.pending_target_configs = []
        
        # Position monitoring
        self.is_monitoring = False
        self.monitor_thread = None
        self.position_data = {}
        
        # Safety limits and previous positions for jump detection
        self.max_position_jump = 2.0  # Maximum allowed position jump in radians
        self.position_limit = 10.0    # Absolute position limit in radians
        self.last_target_positions = {}  # Track last sent positions
        
        # Reader testing
        self.is_reader_testing = False
        self.reader_test_thread = None
        self.reader_test_data = {}
        
        # Target testing
        self.is_target_testing = False
        self.target_test_thread = None
        self.target_test_data = {}
        self.target_test_positions = {}  # Store target positions for each motor
        
        # USB port monitoring
        self.port_refresh_thread = None
        self.is_monitoring_ports = False
        self.available_ports = []
        
        # Config file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(script_dir, "position_reader_config.json")
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.create_gui()
        self.load_config()
        self.start_port_monitoring()
        atexit.register(self.on_closing)
    
    def create_gui(self):
        # Main notebook
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
        # Tabs
        self.config_tab = ttk.Frame(self.notebook)
        self.reader_test_tab = ttk.Frame(self.notebook)
        self.target_test_tab = ttk.Frame(self.notebook)
        self.monitor_tab = ttk.Frame(self.notebook)
        
        self.notebook.add(self.config_tab, text="Configuration")
        self.notebook.add(self.reader_test_tab, text="Reader Test")
        self.notebook.add(self.target_test_tab, text="Target Test")
        self.notebook.add(self.monitor_tab, text="Position Monitor")
        
        self.create_config_tab()
        self.create_reader_test_tab()
        self.create_target_test_tab()
        self.create_monitor_tab()
    
    def create_config_tab(self):
        self.config_tab.grid_rowconfigure(4, weight=1)
        self.config_tab.grid_columnconfigure(0, weight=1)
        
        # Position Reader Port Configuration
        reader_frame = ttk.LabelFrame(self.config_tab, text="Position Reader Port Configuration")
        reader_frame.grid(row=0, column=0, padx=15, pady=10, sticky="ew")
        
        ttk.Label(reader_frame, text="Reader Port:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.reader_port_var = tk.StringVar(value="/dev/ttyACM1")
        self.reader_port_combo = ttk.Combobox(reader_frame, textvariable=self.reader_port_var, width=20)
        self.reader_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(reader_frame, text="Baudrate:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.reader_baud_var = tk.StringVar(value="921600")
        self.reader_baud_entry = ttk.Entry(reader_frame, textvariable=self.reader_baud_var)
        self.reader_baud_entry.grid(row=0, column=3, padx=5, pady=5, sticky="w")
        
        self.reader_connect_btn = ttk.Button(reader_frame, text="Connect Reader", command=self.connect_reader)
        self.reader_connect_btn.grid(row=0, column=4, padx=5, pady=5)
        
        # Target Motors Port Configuration
        target_frame = ttk.LabelFrame(self.config_tab, text="Target Motors Port Configuration")
        target_frame.grid(row=1, column=0, padx=15, pady=10, sticky="ew")
        
        ttk.Label(target_frame, text="Target Port:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.target_port_var = tk.StringVar(value="/dev/ttyACM0")
        self.target_port_combo = ttk.Combobox(target_frame, textvariable=self.target_port_var, width=20)
        self.target_port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(target_frame, text="Baudrate:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.target_baud_var = tk.StringVar(value="921600")
        self.target_baud_entry = ttk.Entry(target_frame, textvariable=self.target_baud_var)
        self.target_baud_entry.grid(row=0, column=3, padx=5, pady=5, sticky="w")
        
        self.target_connect_btn = ttk.Button(target_frame, text="Connect Target", command=self.connect_target)
        self.target_connect_btn.grid(row=0, column=4, padx=5, pady=5)
        
        # Available Ports Display
        ports_frame = ttk.LabelFrame(self.config_tab, text="Available USB Ports")
        ports_frame.grid(row=2, column=0, padx=15, pady=10, sticky="ew")
        
        self.ports_tree = ttk.Treeview(ports_frame, columns=("Port", "Description", "Hardware"), show="headings", height=4)
        self.ports_tree.heading("Port", text="Port")
        self.ports_tree.heading("Description", text="Description")
        self.ports_tree.heading("Hardware", text="Hardware ID")
        
        self.ports_tree.column("Port", width=120)
        self.ports_tree.column("Description", width=250)
        self.ports_tree.column("Hardware", width=200)
        
        self.ports_tree.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        # Bind double-click event to fill port fields
        self.ports_tree.bind("<Double-1>", self.on_port_double_click)
        
        ports_scrollbar = ttk.Scrollbar(ports_frame, orient="vertical", command=self.ports_tree.yview)
        self.ports_tree.configure(yscrollcommand=ports_scrollbar.set)
        ports_scrollbar.grid(row=0, column=1, sticky="ns")
        
        # Refresh button
        refresh_btn = ttk.Button(ports_frame, text="Refresh Ports", command=self.refresh_ports_manual)
        refresh_btn.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        # Auto-refresh status
        self.port_status_var = tk.StringVar(value="Auto-refresh: ON (every 2s)")
        status_label = ttk.Label(ports_frame, textvariable=self.port_status_var)
        status_label.grid(row=1, column=0, padx=5, pady=5, sticky="e")
        
        ports_frame.grid_rowconfigure(0, weight=1)
        ports_frame.grid_columnconfigure(0, weight=1)
        
        # Control buttons
        control_frame = ttk.LabelFrame(self.config_tab, text="Control")
        control_frame.grid(row=3, column=0, padx=15, pady=10, sticky="ew")
        
        
        self.save_config_btn = ttk.Button(control_frame, text="Save Config", command=self.save_config)
        self.save_config_btn.grid(row=0, column=2, padx=5, pady=5)
        
        self.load_config_btn = ttk.Button(control_frame, text="Load Config", command=self.load_config)
        self.load_config_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # Status display
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.status_var)
        status_label.grid(row=1, column=0, columnspan=3, padx=5, pady=5)
        
        # Motor configuration frames
        motors_frame = ttk.LabelFrame(self.config_tab, text="Motor Configuration")
        motors_frame.grid(row=4, column=0, padx=15, pady=10, sticky="nsew")
        
        # Reader motors section
        reader_section = ttk.Frame(motors_frame)
        reader_section.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        reader_motors_frame = ttk.LabelFrame(reader_section, text="Reader Motors")
        reader_motors_frame.grid(row=0, column=0, padx=0, pady=0, sticky="nsew")
        
        self.reader_motors_tree = ttk.Treeview(reader_motors_frame, columns=("Type", "SlaveID", "MasterID", "Position"), show="headings", height=4)
        self.reader_motors_tree.heading("Type", text="Type")
        self.reader_motors_tree.heading("SlaveID", text="Slave ID")
        self.reader_motors_tree.heading("MasterID", text="Master ID")
        self.reader_motors_tree.heading("Position", text="Position")
        self.reader_motors_tree.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        # Reader motor controls
        reader_controls = ttk.Frame(reader_section)
        reader_controls.grid(row=1, column=0, padx=0, pady=5, sticky="ew")
        
        ttk.Label(reader_controls, text="Motor Type:").grid(row=0, column=0, padx=2, pady=2)
        self.reader_motor_type_var = tk.StringVar(value="DM4310")
        reader_type_combo = ttk.Combobox(reader_controls, textvariable=self.reader_motor_type_var, width=10, state="readonly")
        reader_type_combo['values'] = ["DM4310", "DM4310_48V", "DM4340", "DM4340_48V", "DM6006", "DM8006", "DM8009", "DM10010", "DM10010L", "DMG6220", "DMH3510", "DMH6215"]
        reader_type_combo.grid(row=0, column=1, padx=2, pady=2)
        
        ttk.Label(reader_controls, text="Slave ID:").grid(row=0, column=2, padx=2, pady=2)
        self.reader_slave_var = tk.StringVar(value="0x01")
        ttk.Entry(reader_controls, textvariable=self.reader_slave_var, width=8).grid(row=0, column=3, padx=2, pady=2)
        
        ttk.Label(reader_controls, text="Master ID:").grid(row=0, column=4, padx=2, pady=2)
        self.reader_master_var = tk.StringVar(value="0x00")
        ttk.Entry(reader_controls, textvariable=self.reader_master_var, width=8).grid(row=0, column=5, padx=2, pady=2)
        
        ttk.Button(reader_controls, text="Add", command=self.add_reader_motor).grid(row=0, column=6, padx=2, pady=2)
        ttk.Button(reader_controls, text="Remove", command=self.remove_reader_motor).grid(row=0, column=7, padx=2, pady=2)
        
        # Target motors section
        target_section = ttk.Frame(motors_frame)
        target_section.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        
        target_motors_frame = ttk.LabelFrame(target_section, text="Target Motors")
        target_motors_frame.grid(row=0, column=0, padx=0, pady=0, sticky="nsew")
        
        self.target_motors_tree = ttk.Treeview(target_motors_frame, columns=("Type", "SlaveID", "MasterID", "Position"), show="headings", height=4)
        self.target_motors_tree.heading("Type", text="Type")
        self.target_motors_tree.heading("SlaveID", text="Slave ID")  
        self.target_motors_tree.heading("MasterID", text="Master ID")
        self.target_motors_tree.heading("Position", text="Target Position")
        self.target_motors_tree.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        # Target motor controls
        target_controls = ttk.Frame(target_section)
        target_controls.grid(row=1, column=0, padx=0, pady=5, sticky="ew")
        
        ttk.Label(target_controls, text="Motor Type:").grid(row=0, column=0, padx=2, pady=2)
        self.target_motor_type_var = tk.StringVar(value="DM4310")
        target_type_combo = ttk.Combobox(target_controls, textvariable=self.target_motor_type_var, width=10, state="readonly")
        target_type_combo['values'] = ["DM4310", "DM4310_48V", "DM4340", "DM4340_48V", "DM6006", "DM8006", "DM8009", "DM10010", "DM10010L", "DMG6220", "DMH3510", "DMH6215"]
        target_type_combo.grid(row=0, column=1, padx=2, pady=2)
        
        ttk.Label(target_controls, text="Slave ID:").grid(row=0, column=2, padx=2, pady=2)
        self.target_slave_var = tk.StringVar(value="0x01")
        ttk.Entry(target_controls, textvariable=self.target_slave_var, width=8).grid(row=0, column=3, padx=2, pady=2)
        
        ttk.Label(target_controls, text="Master ID:").grid(row=0, column=4, padx=2, pady=2)
        self.target_master_var = tk.StringVar(value="0x00")
        ttk.Entry(target_controls, textvariable=self.target_master_var, width=8).grid(row=0, column=5, padx=2, pady=2)
        
        ttk.Button(target_controls, text="Add", command=self.add_target_motor).grid(row=0, column=6, padx=2, pady=2)
        ttk.Button(target_controls, text="Remove", command=self.remove_target_motor).grid(row=0, column=7, padx=2, pady=2)
        
        
        motors_frame.grid_columnconfigure(0, weight=1)
        motors_frame.grid_columnconfigure(1, weight=1)
        reader_section.grid_columnconfigure(0, weight=1)
        target_section.grid_columnconfigure(0, weight=1)
        reader_motors_frame.grid_columnconfigure(0, weight=1)
        target_motors_frame.grid_columnconfigure(0, weight=1)
    
    def create_monitor_tab(self):
        self.monitor_tab.grid_rowconfigure(2, weight=1)
        self.monitor_tab.grid_columnconfigure(0, weight=1)
        
        # Monitor controls
        monitor_controls = ttk.LabelFrame(self.monitor_tab, text="Monitor Controls")
        monitor_controls.grid(row=0, column=0, padx=15, pady=10, sticky="ew")
        
        self.start_monitor_btn = ttk.Button(monitor_controls, text="Start Monitoring", command=self.start_monitoring)
        self.start_monitor_btn.grid(row=0, column=0, padx=5, pady=5)
        
        self.stop_monitor_btn = ttk.Button(monitor_controls, text="Stop Monitoring", command=self.stop_monitoring)
        self.stop_monitor_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # Safety controls
        safety_frame = ttk.LabelFrame(self.monitor_tab, text="Safety Limits")
        safety_frame.grid(row=2, column=0, padx=15, pady=10, sticky="ew")
        
        ttk.Label(safety_frame, text="Max Position Jump (rad):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.max_jump_var = tk.StringVar(value="2.0")
        max_jump_entry = ttk.Entry(safety_frame, textvariable=self.max_jump_var, width=10)
        max_jump_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(safety_frame, text="Position Limit (±rad):").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.pos_limit_var = tk.StringVar(value="10.0")
        pos_limit_entry = ttk.Entry(safety_frame, textvariable=self.pos_limit_var, width=10)
        pos_limit_entry.grid(row=0, column=3, padx=5, pady=5)
        
        update_safety_btn = ttk.Button(safety_frame, text="Update Safety", command=self.update_safety_limits)
        update_safety_btn.grid(row=0, column=4, padx=5, pady=5)
        
        # Motor mapping section
        mapping_frame = ttk.LabelFrame(self.monitor_tab, text="Motor Mapping (Reader → Target)")
        mapping_frame.grid(row=3, column=0, padx=15, pady=10, sticky="ew")
        
        self.mapping_tree = ttk.Treeview(mapping_frame, columns=("ReaderID", "TargetID", "Status"), show="headings", height=4)
        self.mapping_tree.heading("ReaderID", text="Reader ID")
        self.mapping_tree.heading("TargetID", text="Target ID")
        self.mapping_tree.heading("Status", text="Status")
        self.mapping_tree.grid(row=0, column=0, columnspan=4, padx=5, pady=5, sticky="nsew")
        
        # Mapping controls
        mapping_controls = ttk.Frame(mapping_frame)
        mapping_controls.grid(row=1, column=0, columnspan=4, pady=5, sticky="ew")
        
        ttk.Label(mapping_controls, text="Reader Slave ID:").grid(row=0, column=0, padx=2, pady=2)
        self.map_reader_var = tk.StringVar(value="0x01")
        ttk.Entry(mapping_controls, textvariable=self.map_reader_var, width=8).grid(row=0, column=1, padx=2, pady=2)
        
        ttk.Label(mapping_controls, text="→ Target Slave ID:").grid(row=0, column=2, padx=2, pady=2)
        self.map_target_var = tk.StringVar(value="0x01")
        ttk.Entry(mapping_controls, textvariable=self.map_target_var, width=8).grid(row=0, column=3, padx=2, pady=2)
        
        ttk.Button(mapping_controls, text="Map", command=self.add_motor_mapping).grid(row=0, column=4, padx=5, pady=2)
        ttk.Button(mapping_controls, text="Unmap", command=self.remove_motor_mapping).grid(row=0, column=5, padx=5, pady=2)
        
        mapping_frame.grid_columnconfigure(0, weight=1)
        
        # Position monitoring display
        monitor_frame = ttk.LabelFrame(self.monitor_tab, text="Position Monitoring")
        monitor_frame.grid(row=2, column=0, padx=15, pady=10, sticky="nsew")
        
        # Position data display
        self.position_tree = ttk.Treeview(monitor_frame, columns=("ReaderID", "ReaderPos", "TargetID", "TargetPos", "Status"), show="headings")
        self.position_tree.heading("ReaderID", text="Reader ID")
        self.position_tree.heading("ReaderPos", text="Reader Position")
        self.position_tree.heading("TargetID", text="Target ID")
        self.position_tree.heading("TargetPos", text="Target Position")
        self.position_tree.heading("Status", text="Status")
        
        scrollbar = ttk.Scrollbar(monitor_frame, orient="vertical", command=self.position_tree.yview)
        self.position_tree.configure(yscrollcommand=scrollbar.set)
        
        self.position_tree.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        
        monitor_frame.grid_rowconfigure(0, weight=1)
        monitor_frame.grid_columnconfigure(0, weight=1)
    
    def create_reader_test_tab(self):
        # Reader test controls
        control_frame = ttk.LabelFrame(self.reader_test_tab, text="Reader Test Controls")
        control_frame.grid(row=0, column=0, padx=15, pady=10, sticky="ew")
        
        self.reader_test_tab.grid_columnconfigure(0, weight=1)
        
        self.start_reader_test_btn = ttk.Button(control_frame, text="Start Reader Test", command=self.start_reader_test)
        self.start_reader_test_btn.grid(row=0, column=0, padx=5, pady=5)
        
        self.stop_reader_test_btn = ttk.Button(control_frame, text="Stop Reader Test", command=self.stop_reader_test)
        self.stop_reader_test_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # Status display for reader test
        self.reader_test_status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.reader_test_status_var)
        status_label.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        
        # Reader test data display
        test_frame = ttk.LabelFrame(self.reader_test_tab, text="Reader Motor Data")
        test_frame.grid(row=1, column=0, padx=15, pady=10, sticky="nsew")
        
        self.reader_test_tab.grid_rowconfigure(1, weight=1)
        
        self.reader_test_tree = ttk.Treeview(test_frame, columns=("SlaveID", "MasterID", "Position", "Velocity", "Torque"), show="headings")
        self.reader_test_tree.heading("SlaveID", text="Slave ID")
        self.reader_test_tree.heading("MasterID", text="Master ID")
        self.reader_test_tree.heading("Position", text="Position (rad)")
        self.reader_test_tree.heading("Velocity", text="Velocity (rad/s)")
        self.reader_test_tree.heading("Torque", text="Torque (Nm)")
        
        # Column widths
        self.reader_test_tree.column("SlaveID", width=80)
        self.reader_test_tree.column("MasterID", width=80)
        self.reader_test_tree.column("Position", width=120)
        self.reader_test_tree.column("Velocity", width=120)
        self.reader_test_tree.column("Torque", width=120)
        
        test_scrollbar = ttk.Scrollbar(test_frame, orient="vertical", command=self.reader_test_tree.yview)
        self.reader_test_tree.configure(yscrollcommand=test_scrollbar.set)
        
        self.reader_test_tree.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        test_scrollbar.grid(row=0, column=1, sticky="ns")
        
        test_frame.grid_rowconfigure(0, weight=1)
        test_frame.grid_columnconfigure(0, weight=1)
    
    def create_target_test_tab(self):
        # Target test controls
        control_frame = ttk.LabelFrame(self.target_test_tab, text="Target Test Controls")
        control_frame.grid(row=0, column=0, padx=15, pady=10, sticky="ew")
        
        self.target_test_tab.grid_columnconfigure(0, weight=1)
        
        self.start_target_test_btn = ttk.Button(control_frame, text="Start Target Test", command=self.start_target_test)
        self.start_target_test_btn.grid(row=0, column=0, padx=5, pady=5)
        
        self.stop_target_test_btn = ttk.Button(control_frame, text="Stop Target Test", command=self.stop_target_test)
        self.stop_target_test_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # Status display for target test
        self.target_test_status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=self.target_test_status_var)
        status_label.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        
        # Target motor controls
        motor_control_frame = ttk.LabelFrame(self.target_test_tab, text="Manual Motor Control")
        motor_control_frame.grid(row=1, column=0, padx=15, pady=10, sticky="ew")
        
        ttk.Label(motor_control_frame, text="Motor ID:").grid(row=0, column=0, padx=5, pady=5)
        self.target_motor_select_var = tk.StringVar()
        self.target_motor_select_combo = ttk.Combobox(motor_control_frame, textvariable=self.target_motor_select_var, width=10, state="readonly")
        self.target_motor_select_combo.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(motor_control_frame, text="Position (rad):").grid(row=0, column=2, padx=5, pady=5)
        self.target_position_var = tk.StringVar(value="0.0")
        ttk.Entry(motor_control_frame, textvariable=self.target_position_var, width=10).grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(motor_control_frame, text="Move To", command=self.move_target_motor).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(motor_control_frame, text="Set Zero", command=self.set_target_zero).grid(row=0, column=5, padx=5, pady=5)
        
        # Target test data display
        test_frame = ttk.LabelFrame(self.target_test_tab, text="Target Motor Data")
        test_frame.grid(row=2, column=0, padx=15, pady=10, sticky="nsew")
        
        self.target_test_tab.grid_rowconfigure(2, weight=1)
        
        self.target_test_tree = ttk.Treeview(test_frame, columns=("SlaveID", "MasterID", "TargetPos", "CurrentPos", "Velocity", "Torque", "Status"), show="headings")
        self.target_test_tree.heading("SlaveID", text="Slave ID")
        self.target_test_tree.heading("MasterID", text="Master ID")
        self.target_test_tree.heading("TargetPos", text="Target Pos (rad)")
        self.target_test_tree.heading("CurrentPos", text="Current Pos (rad)")
        self.target_test_tree.heading("Velocity", text="Velocity (rad/s)")
        self.target_test_tree.heading("Torque", text="Torque (Nm)")
        self.target_test_tree.heading("Status", text="Status")
        
        # Column widths
        self.target_test_tree.column("SlaveID", width=80)
        self.target_test_tree.column("MasterID", width=80)
        self.target_test_tree.column("TargetPos", width=120)
        self.target_test_tree.column("CurrentPos", width=120)
        self.target_test_tree.column("Velocity", width=120)
        self.target_test_tree.column("Torque", width=120)
        self.target_test_tree.column("Status", width=100)
        
        target_scrollbar = ttk.Scrollbar(test_frame, orient="vertical", command=self.target_test_tree.yview)
        self.target_test_tree.configure(yscrollcommand=target_scrollbar.set)
        
        self.target_test_tree.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        target_scrollbar.grid(row=0, column=1, sticky="ns")
        
        test_frame.grid_rowconfigure(0, weight=1)
        test_frame.grid_columnconfigure(0, weight=1)
    
    def connect_reader(self):
        try:
            port = self.reader_port_var.get()
            baudrate = int(self.reader_baud_var.get())
            self.reader_serial_port = serial.Serial(port, baudrate, timeout=0.5)
            self.reader_motor_control = MotorControl(self.reader_serial_port)
            
            # Clear any existing motors
            self.reader_motors = {}
            
            # Load any pending reader motor configurations
            pending_count = 0
            for motor_config in self.pending_reader_configs:
                try:
                    self._load_reader_motor(motor_config)
                    pending_count += 1
                except Exception as e:
                    print(f"Error loading pending reader motor: {e}")
            
            self.reader_connect_btn.configure(text="Disconnect Reader", command=self.disconnect_reader)
            if pending_count > 0:
                self.update_status(f"Reader port connected successfully! Loaded {pending_count} motors from config.")
            else:
                self.update_status("Reader port connected successfully!")
            self.update_reader_motors_display()
        except Exception as e:
            self.update_status(f"Failed to connect reader: {str(e)}")
    
    def disconnect_reader(self):
        try:
            # Stop monitoring and reader test first if running
            if self.is_monitoring:
                self.stop_monitoring()
            if self.is_reader_testing:
                self.stop_reader_test()
                
            if self.reader_serial_port and self.reader_serial_port.is_open:
                self.reader_serial_port.close()
            self.reader_motors = {}
            self.reader_motor_control = None
            self.reader_serial_port = None
            # Clear mappings for disconnected reader motors
            self.motor_mapping = {}
            self.reader_connect_btn.configure(text="Connect Reader", command=self.connect_reader)
            self.update_status("Reader disconnected")
            self.update_reader_motors_display()
            self.update_mapping_display()
        except Exception as e:
            self.update_status(f"Error disconnecting reader: {str(e)}")
    
    def connect_target(self):
        try:
            port = self.target_port_var.get()
            baudrate = int(self.target_baud_var.get())
            self.target_serial_port = serial.Serial(port, baudrate, timeout=0.5)
            self.target_motor_control = MotorControl(self.target_serial_port)
            
            # Clear any existing motors
            self.target_motors = {}
            
            # Load any pending target motor configurations
            pending_count = 0
            for motor_config in self.pending_target_configs:
                try:
                    self._load_target_motor(motor_config)
                    pending_count += 1
                except Exception as e:
                    print(f"Error loading pending target motor: {e}")
            
            self.target_connect_btn.configure(text="Disconnect Target", command=self.disconnect_target)
            if pending_count > 0:
                self.update_status(f"Target port connected successfully! Loaded {pending_count} motors from config.")
            else:
                self.update_status("Target port connected successfully!")
            self.update_target_motors_display()
        except Exception as e:
            self.update_status(f"Failed to connect target: {str(e)}")
    
    def disconnect_target(self):
        try:
            if self.target_serial_port and self.target_serial_port.is_open:
                for motor in self.target_motors.values():
                    try:
                        self.target_motor_control.disable(motor)
                    except Exception as e:
                        print(f"Error disabling target motor 0x{motor.SlaveID:02x}: {e}")
                self.target_serial_port.close()
            self.target_motors = {}
            self.target_motor_control = None
            self.target_serial_port = None
            # Clear mappings for disconnected target motors
            self.motor_mapping = {}
            self.target_connect_btn.configure(text="Connect Target", command=self.connect_target)
            self.update_status("Target disconnected")
            self.update_target_motors_display()
            self.update_mapping_display()
        except Exception as e:
            self.update_status(f"Error disconnecting target: {str(e)}")
    
    def add_reader_motor(self):
        if not self.reader_motor_control:
            self.update_status("Connect reader port first")
            return
            
        try:
            motor_type_str = self.reader_motor_type_var.get()
            slave_id_str = self.reader_slave_var.get().strip()
            master_id_str = self.reader_master_var.get().strip()
            
            slave_id = int(slave_id_str, 16) if slave_id_str.startswith('0x') else int(slave_id_str)
            master_id = int(master_id_str, 16) if master_id_str.startswith('0x') else int(master_id_str)
            
            if slave_id in self.reader_motors:
                self.update_status(f"Reader motor 0x{slave_id:02x} already exists")
                return
                
            motor_type = self.get_motor_type_enum(motor_type_str)
            motor = Motor(motor_type, slave_id, master_id)
            
            # Add motor to control system and initialize (like BotArmJointTest)
            self.reader_motor_control.addMotor(motor)
            
            # Switch to POS_VEL control mode
            if self.reader_motor_control.switchControlMode(motor, Control_Type.POS_VEL):
                print(f"Reader motor 0x{slave_id:02x} switched to POS_VEL mode")
            
            # Read and display motor parameters (like BotArmJointTest)
            try:
                print(f"Motor 0x{slave_id:02x} sub_ver: {self.reader_motor_control.read_motor_param(motor, DM_variable.sub_ver)}")
                print(f"Motor 0x{slave_id:02x} Gr: {self.reader_motor_control.read_motor_param(motor, DM_variable.Gr)}")
                print(f"Motor 0x{slave_id:02x} PMAX: {self.reader_motor_control.read_motor_param(motor, DM_variable.PMAX)}")
                print(f"Motor 0x{slave_id:02x} VMAX: {self.reader_motor_control.read_motor_param(motor, DM_variable.VMAX)}")
                print(f"Motor 0x{slave_id:02x} TMAX: {self.reader_motor_control.read_motor_param(motor, DM_variable.TMAX)}")
            except Exception as e:
                print(f"Warning: Could not read parameters for motor 0x{slave_id:02x}: {e}")
            
            # Save motor parameters and set zero position (like BotArmJointTest)
            self.reader_motor_control.save_motor_param(motor)
            self.reader_motor_control.set_zero_position(motor)
            
            # Keep reader motors disabled initially
            self.reader_motors[slave_id] = motor
            self.update_reader_motors_display()
            self.update_mapping_display()
            self.update_status(f"Reader motor {motor_type_str} 0x{slave_id:02x} (master: 0x{master_id:02x}) added and initialized (disabled)")
            
        except ValueError:
            self.update_status("Invalid motor ID format")
        except Exception as e:
            self.update_status(f"Error adding reader motor: {str(e)}")
    
    def remove_reader_motor(self):
        selected = self.reader_motors_tree.selection()
        if not selected:
            self.update_status("Select a reader motor to remove")
            return
            
        try:
            item = self.reader_motors_tree.item(selected[0])
            slave_id_str = item['values'][1]
            slave_id = int(slave_id_str, 16)
            
            if slave_id in self.reader_motors:
                motor = self.reader_motors[slave_id]
                try:
                    self.reader_motor_control.disable(motor)
                except Exception as e:
                    print(f"Error disabling motor: {e}")
                    
                del self.reader_motors[slave_id]
                
                # Remove any mappings for this motor
                if slave_id in self.motor_mapping:
                    del self.motor_mapping[slave_id]
                    
                self.update_reader_motors_display()
                self.update_mapping_display()
                self.update_status(f"Reader motor 0x{slave_id:02x} removed")
                
        except Exception as e:
            self.update_status(f"Error removing reader motor: {str(e)}")
    
    def add_target_motor(self):
        if not self.target_motor_control:
            self.update_status("Connect target port first")
            return
            
        try:
            motor_type_str = self.target_motor_type_var.get()
            slave_id_str = self.target_slave_var.get().strip()
            master_id_str = self.target_master_var.get().strip()
            
            slave_id = int(slave_id_str, 16) if slave_id_str.startswith('0x') else int(slave_id_str)
            master_id = int(master_id_str, 16) if master_id_str.startswith('0x') else int(master_id_str)
                
            if slave_id in self.target_motors:
                self.update_status(f"Target motor 0x{slave_id:02x} already exists")
                return
                
            motor_type = self.get_motor_type_enum(motor_type_str)
            motor = Motor(motor_type, slave_id, master_id)
            
            # Add motor to control system and initialize (like BotArmJointTest)
            self.target_motor_control.addMotor(motor)
            
            # Switch to POS_VEL control mode
            if self.target_motor_control.switchControlMode(motor, Control_Type.POS_VEL):
                print(f"Target motor 0x{slave_id:02x} switched to POS_VEL mode")
            
            # Read and display motor parameters (like BotArmJointTest)
            try:
                print(f"Motor 0x{slave_id:02x} sub_ver: {self.target_motor_control.read_motor_param(motor, DM_variable.sub_ver)}")
                print(f"Motor 0x{slave_id:02x} Gr: {self.target_motor_control.read_motor_param(motor, DM_variable.Gr)}")
                print(f"Motor 0x{slave_id:02x} PMAX: {self.target_motor_control.read_motor_param(motor, DM_variable.PMAX)}")
                print(f"Motor 0x{slave_id:02x} VMAX: {self.target_motor_control.read_motor_param(motor, DM_variable.VMAX)}")
                print(f"Motor 0x{slave_id:02x} TMAX: {self.target_motor_control.read_motor_param(motor, DM_variable.TMAX)}")
            except Exception as e:
                print(f"Warning: Could not read parameters for motor 0x{slave_id:02x}: {e}")
            
            # Save motor parameters and set zero position (like BotArmJointTest)
            self.target_motor_control.save_motor_param(motor)
            self.target_motor_control.set_zero_position(motor)
            
            # Keep target motors disabled initially
            self.target_motors[slave_id] = motor
            self.update_target_motors_display()
            self.update_mapping_display()
            self.update_status(f"Target motor {motor_type_str} 0x{slave_id:02x} (master: 0x{master_id:02x}) added and initialized (disabled)")
            
        except ValueError:
            self.update_status("Invalid motor ID format")
        except Exception as e:
            self.update_status(f"Error adding target motor: {str(e)}")
    
    def remove_target_motor(self):
        selected = self.target_motors_tree.selection()
        if not selected:
            self.update_status("Select a target motor to remove")
            return
            
        try:
            item = self.target_motors_tree.item(selected[0])
            slave_id_str = item['values'][1]
            slave_id = int(slave_id_str, 16)
            
            if slave_id in self.target_motors:
                motor = self.target_motors[slave_id]
                try:
                    self.target_motor_control.disable(motor)
                except Exception as e:
                    print(f"Error disabling motor: {e}")
                    
                del self.target_motors[slave_id]
                
                # Remove any mappings to this motor
                to_remove = [k for k, v in self.motor_mapping.items() if v == slave_id]
                for k in to_remove:
                    del self.motor_mapping[k]
                    
                self.update_target_motors_display()
                self.update_mapping_display()
                self.update_status(f"Target motor 0x{slave_id:02x} removed")
                
        except Exception as e:
            self.update_status(f"Error removing target motor: {str(e)}")
    
    def add_motor_mapping(self):
        try:
            reader_id_str = self.map_reader_var.get().strip()
            target_id_str = self.map_target_var.get().strip()
            
            reader_id = int(reader_id_str, 16) if reader_id_str.startswith('0x') else int(reader_id_str)
            target_id = int(target_id_str, 16) if target_id_str.startswith('0x') else int(target_id_str)
            
            if reader_id not in self.reader_motors:
                self.update_status(f"Reader motor 0x{reader_id:02x} not found")
                return
                
            if target_id not in self.target_motors:
                self.update_status(f"Target motor 0x{target_id:02x} not found")
                return
                
            self.motor_mapping[reader_id] = target_id
            self.update_mapping_display()
            self.update_status(f"Mapped reader 0x{reader_id:02x} → target 0x{target_id:02x}")
            
        except ValueError:
            self.update_status("Invalid motor ID format")
        except Exception as e:
            self.update_status(f"Error adding mapping: {str(e)}")
    
    def remove_motor_mapping(self):
        selected = self.mapping_tree.selection()
        if not selected:
            self.update_status("Select a mapping to remove")
            return
            
        try:
            item = self.mapping_tree.item(selected[0])
            reader_id_str = item['values'][0]
            reader_id = int(reader_id_str, 16)
            
            if reader_id in self.motor_mapping:
                del self.motor_mapping[reader_id]
                self.update_mapping_display()
                self.update_status(f"Mapping for reader 0x{reader_id:02x} removed")
                
        except Exception as e:
            self.update_status(f"Error removing mapping: {str(e)}")
    
    def start_monitoring(self):
        if not self.reader_motor_control:
            self.update_status("Reader port must be connected")
            return
        if not self.target_motor_control:
            self.update_status("Target port must be connected")
            return
        if not self.motor_mapping:
            self.update_status("At least one motor mapping must be configured")
            return
        if self.is_monitoring:
            self.update_status("Monitoring already running")
            return
            
        # Validate all mappings before starting
        invalid_mappings = []
        for reader_id, target_id in self.motor_mapping.items():
            if reader_id not in self.reader_motors:
                invalid_mappings.append(f"Reader motor 0x{reader_id:02x} not found")
            if target_id not in self.target_motors:
                invalid_mappings.append(f"Target motor 0x{target_id:02x} not found")
                
        if invalid_mappings:
            self.update_status(f"Invalid mappings: {'; '.join(invalid_mappings)}")
            return
            
        # Disable reader motors and enable target motors for monitoring
        try:
            # Ensure reader motors are disabled (they should read position only)
            for reader_id in self.motor_mapping.keys():
                if reader_id in self.reader_motors:
                    reader_motor = self.reader_motors[reader_id]
                    try:
                        self.reader_motor_control.disable(reader_motor)
                        print(f"Reader motor 0x{reader_id:02x} disabled for monitoring")
                    except Exception as e:
                        print(f"Error disabling reader motor 0x{reader_id:02x}: {e}")
                    
            # Reset all motor positions to zero before starting monitoring
            print("Resetting all motor positions to zero...")
            
            # Reset reader motor positions to zero
            for reader_id in self.motor_mapping.keys():
                if reader_id in self.reader_motors:
                    reader_motor = self.reader_motors[reader_id]
                    try:
                        self.reader_motor_control.set_zero_position(reader_motor)
                        print(f"Reader motor 0x{reader_id:02x} position reset to zero")
                    except Exception as e:
                        print(f"Error resetting reader motor 0x{reader_id:02x} position: {e}")
            
            # Enable target motors that are mapped (they need to move) and reset their positions
            enabled_targets = 0
            for target_id in self.motor_mapping.values():
                if target_id in self.target_motors:
                    target_motor = self.target_motors[target_id]
                    try:
                        # Reset target motor position to zero
                        self.target_motor_control.set_zero_position(target_motor)
                        print(f"Target motor 0x{target_id:02x} position reset to zero")
                        
                        # Enable target motor
                        self.target_motor_control.enable(target_motor)
                        enabled_targets += 1
                        print(f"Target motor 0x{target_id:02x} enabled for monitoring")
                        
                        # Initialize position tracking with current position
                        self.target_motor_control.refresh_motor_status(target_motor)
                        self.last_target_positions[target_id] = target_motor.getPosition()
                    except Exception as e:
                        print(f"Error enabling target motor 0x{target_id:02x}: {e}")
                        
            if enabled_targets == 0:
                self.update_status("Failed to enable any target motors")
                return
                    
            self.is_monitoring = True
            self.monitor_thread = threading.Thread(target=self.monitoring_loop, daemon=True)
            self.monitor_thread.start()
            self.update_status(f"Position monitoring started - {len(self.motor_mapping)} mappings ({enabled_targets} target motors enabled)")
            
        except Exception as e:
            self.update_status(f"Error starting monitoring: {str(e)}")
            return
    
    def stop_monitoring(self):
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
            
        # Disable ALL motors when monitoring stops
        try:
            # Disable all target motors
            disabled_targets = 0
            for slave_id, target_motor in self.target_motors.items():
                try:
                    self.target_motor_control.disable(target_motor)
                    disabled_targets += 1
                    print(f"Target motor 0x{slave_id:02x} disabled")
                except Exception as e:
                    print(f"Error disabling target motor 0x{slave_id:02x}: {e}")
            
            # Disable all reader motors (ensure they're disabled)
            disabled_readers = 0
            for slave_id, reader_motor in self.reader_motors.items():
                try:
                    self.reader_motor_control.disable(reader_motor)
                    disabled_readers += 1
                    print(f"Reader motor 0x{slave_id:02x} disabled")
                except Exception as e:
                    print(f"Error disabling reader motor 0x{slave_id:02x}: {e}")
                    
        except Exception as e:
            print(f"Error during motor shutdown: {e}")
            
        self.update_status(f"Position monitoring stopped - All motors disabled ({disabled_targets} targets, {disabled_readers} readers)")
    
    def monitoring_loop(self):
        while self.is_monitoring:
            try:
                # Read positions from mapped reader motors only
                for reader_id, target_id in self.motor_mapping.items():
                    try:
                        # Check if both motors exist
                        if reader_id not in self.reader_motors or target_id not in self.target_motors:
                            continue
                            
                        reader_motor = self.reader_motors[reader_id]
                        target_motor = self.target_motors[target_id]
                        
                        # Get position from reader motor (same as BotArmJointTest)
                        self.reader_motor_control.refresh_motor_status(reader_motor)
                        reader_position = reader_motor.getPosition()
                        
                        # Safety check before sending position command
                        if self.is_safe_position_command(target_id, reader_position):
                            # Send position command to target motor
                            self.target_motor_control.control_Pos_Vel(target_motor, reader_position, 10.0)
                            # Update last position tracking
                            self.last_target_positions[target_id] = reader_position
                        else:
                            print(f"SAFETY: Skipping dangerous position command for target motor 0x{target_id:02x}")
                            # Continue with current position instead of jumping
                        
                        # Get target motor feedback
                        self.target_motor_control.refresh_motor_status(target_motor)
                        target_position = target_motor.getPosition()
                        
                        # Store position data with mapping info
                        self.position_data[f"{reader_id}->{target_id}"] = {
                            "reader_id": reader_id,
                            "target_id": target_id,
                            "reader_pos": reader_position,
                            "target_pos": target_position,
                            "timestamp": time.time()
                        }
                    
                    except Exception as e:
                        print(f"Error in monitoring loop for mapping 0x{reader_id:02x}->0x{target_id:02x}: {e}")
                
                # Update display
                self.root.after(0, self.update_position_display)
                
                time.sleep(0.01)  # 100Hz update rate
                
            except Exception as e:
                print(f"Error in monitoring loop: {e}")
                time.sleep(0.1)
    
    def update_position_display(self):
        # Clear existing items
        for item in self.position_tree.get_children():
            self.position_tree.delete(item)
        
        # Add current position data
        for _, data in self.position_data.items():
            status = "OK" if abs(data["reader_pos"] - data["target_pos"]) < 0.01 else "Following"
            self.position_tree.insert("", "end", values=(
                f"0x{data['reader_id']:02x}",
                f"{data['reader_pos']:.3f}",
                f"0x{data['target_id']:02x}",
                f"{data['target_pos']:.3f}",
                status
            ))
    
    def update_reader_motors_display(self):
        # Clear existing items
        for item in self.reader_motors_tree.get_children():
            self.reader_motors_tree.delete(item)
        
        # Add connected reader motors
        for slave_id, motor in self.reader_motors.items():
            self.reader_motors_tree.insert("", "end", values=(
                motor.MotorType.name,
                f"0x{slave_id:02x}",
                f"0x{motor.MasterID:02x}",
                "0.000"
            ))
        
        # Add pending reader motor configs (not yet connected)
        for motor_config in self.pending_reader_configs:
            slave_id = motor_config["slave_id"]
            if slave_id not in self.reader_motors:  # Only show if not already loaded
                self.reader_motors_tree.insert("", "end", values=(
                    f"{motor_config['motor_type']} (pending)",
                    f"0x{slave_id:02x}",
                    f"0x{motor_config['master_id']:02x}",
                    "Not connected"
                ))
    
    def update_target_motors_display(self):
        # Clear existing items
        for item in self.target_motors_tree.get_children():
            self.target_motors_tree.delete(item)
        
        # Add connected target motors
        for slave_id, motor in self.target_motors.items():
            self.target_motors_tree.insert("", "end", values=(
                motor.MotorType.name,
                f"0x{slave_id:02x}",
                f"0x{motor.MasterID:02x}",
                "0.000"
            ))
        
        # Add pending target motor configs (not yet connected)
        for motor_config in self.pending_target_configs:
            slave_id = motor_config["slave_id"]
            if slave_id not in self.target_motors:  # Only show if not already loaded
                self.target_motors_tree.insert("", "end", values=(
                    f"{motor_config['motor_type']} (pending)",
                    f"0x{slave_id:02x}",
                    f"0x{motor_config['master_id']:02x}",
                    "Not connected"
                ))
    
    def update_mapping_display(self):
        # Clear existing items
        for item in self.mapping_tree.get_children():
            self.mapping_tree.delete(item)
        
        # Add motor mappings
        for reader_id, target_id in self.motor_mapping.items():
            status = "Ready"
            if reader_id not in self.reader_motors:
                status = "Reader Missing"
            elif target_id not in self.target_motors:
                status = "Target Missing"
            elif self.is_monitoring:
                status = "Monitoring"
                
            self.mapping_tree.insert("", "end", values=(
                f"0x{reader_id:02x}",
                f"0x{target_id:02x}",
                status
            ))
    
    def start_reader_test(self):
        if not self.reader_motor_control:
            self.reader_test_status_var.set("Reader port must be connected")
            return
        if not self.reader_motors:
            self.reader_test_status_var.set("At least one reader motor must be added")
            return
        if self.is_reader_testing:
            self.reader_test_status_var.set("Reader test already running")
            return
        if self.is_monitoring:
            self.reader_test_status_var.set("Stop position monitoring first")
            return
            
        print(f"Starting reader test with {len(self.reader_motors)} motors:")
        for slave_id, motor in self.reader_motors.items():
            print(f"  - Motor 0x{slave_id:02x}: {motor.MotorType.name} (Master: 0x{motor.MasterID:02x})")
            
        # Reader motors are already initialized when added, just start reading
        try:
            if not self.reader_motors:
                self.reader_test_status_var.set("No reader motors available")
                return
                    
            self.is_reader_testing = True
            self.reader_test_thread = threading.Thread(target=self.reader_test_loop, daemon=True)
            self.reader_test_thread.start()
            self.reader_test_status_var.set(f"Reader test started - reading from {len(self.reader_motors)} motors (disabled)")
            
            print(f"Started reader test with {len(self.reader_motors)} motors (already initialized)")
            
        except Exception as e:
            self.reader_test_status_var.set(f"Error starting reader test: {str(e)}")
            return
    
    def stop_reader_test(self):
        self.is_reader_testing = False
        if self.reader_test_thread:
            self.reader_test_thread.join(timeout=1.0)
            
        # Reader motors were never enabled, so no need to disable them
        # Just clear the test data and stop reading
        try:
            for slave_id in self.reader_motors.keys():
                print(f"Stopped reading from motor 0x{slave_id:02x}")
                    
            # Clear the test data
            self.reader_test_data = {}
                    
        except Exception as e:
            print(f"Error during reader motor shutdown: {e}")
            
        self.reader_test_status_var.set("Reader test stopped (all reader motors disabled)")
    
    def reader_test_loop(self):
        while self.is_reader_testing:
            try:
                # Read data from all reader motors
                for slave_id, reader_motor in self.reader_motors.items():
                    try:
                        # Refresh motor status to get latest data from hardware
                        self.reader_motor_control.refresh_motor_status(reader_motor)
                        
                        # Get the motor values (same as BotArmJointTest)
                        position = reader_motor.getPosition()
                        velocity = reader_motor.getVelocity()
                        torque = reader_motor.getTorque()
                        
                        # Store reader test data
                        self.reader_test_data[slave_id] = {
                            "position": position,
                            "velocity": velocity,
                            "torque": torque,
                            "master_id": reader_motor.MasterID,
                            "timestamp": time.time()
                        }
                        
                        # Debug: Print readings for troubleshooting
                        if time.time() % 1 < 0.05:  # Print once per second
                            print(f"Motor 0x{slave_id:02x}: Pos={position:.3f}, Vel={velocity:.3f}, Torque={torque:.3f}")
                    
                    except Exception as e:
                        print(f"Error reading reader motor 0x{slave_id:02x}: {e}")
                        # Create placeholder data so we can see the motor in the display
                        if slave_id not in self.reader_test_data:
                            self.reader_test_data[slave_id] = {
                                "position": 0.0,
                                "velocity": 0.0,
                                "torque": 0.0,
                                "master_id": reader_motor.MasterID,
                                "timestamp": time.time()
                            }
                
                # Update display
                self.root.after(0, self.update_reader_test_display)
                
                time.sleep(0.1)  # 10Hz update rate for testing
                
            except Exception as e:
                print(f"Error in reader test loop: {e}")
                time.sleep(0.1)
    
    def update_reader_test_display(self):
        # Clear existing items
        for item in self.reader_test_tree.get_children():
            self.reader_test_tree.delete(item)
        
        # Add current reader test data
        if not self.reader_test_data:
            # Show placeholder if no data yet
            for slave_id, motor in self.reader_motors.items():
                self.reader_test_tree.insert("", "end", values=(
                    f"0x{slave_id:02x}",
                    f"0x{motor.MasterID:02x}",
                    "No Data",
                    "No Data",
                    "No Data"
                ))
        else:
            for slave_id, data in self.reader_test_data.items():
                self.reader_test_tree.insert("", "end", values=(
                    f"0x{slave_id:02x}",
                    f"0x{data['master_id']:02x}",
                    f"{data['position']:.3f}",
                    f"{data['velocity']:.3f}",
                    f"{data['torque']:.3f}"
                ))
    
    def start_target_test(self):
        if not self.target_motor_control:
            self.target_test_status_var.set("Target port must be connected")
            return
        if not self.target_motors:
            self.target_test_status_var.set("At least one target motor must be added")
            return
        if self.is_target_testing:
            self.target_test_status_var.set("Target test already running")
            return
        if self.is_monitoring:
            self.target_test_status_var.set("Stop position monitoring first")
            return
            
        print(f"Starting target test with {len(self.target_motors)} motors:")
        for slave_id, motor in self.target_motors.items():
            print(f"  - Motor 0x{slave_id:02x}: {motor.MotorType.name} (Master: 0x{motor.MasterID:02x})")
            
        # Enable all target motors for testing and update dropdown
        try:
            enabled_count = 0
            motor_options = []
            for slave_id, target_motor in self.target_motors.items():
                try:
                    # Initialize motor following DM_Motor_Test pattern
                    if self.target_motor_control.switchControlMode(target_motor, Control_Type.POS_VEL):
                        print(f"Motor 0x{slave_id:02x} switched to POS_VEL mode successfully")
                    
                    # Read and display motor parameters
                    print(f"M{slave_id:02x} sub_ver:", self.target_motor_control.read_motor_param(target_motor, DM_variable.sub_ver))
                    print(f"M{slave_id:02x} Gr:", self.target_motor_control.read_motor_param(target_motor, DM_variable.Gr))
                    print("PMAX:", self.target_motor_control.read_motor_param(target_motor, DM_variable.PMAX))
                    print("VMAX:", self.target_motor_control.read_motor_param(target_motor, DM_variable.VMAX))
                    print("TMAX:", self.target_motor_control.read_motor_param(target_motor, DM_variable.TMAX))
                    
                    # Set zero position and save parameters
                    self.target_motor_control.set_zero_position(target_motor)
                    self.target_motor_control.save_motor_param(target_motor)
                    
                    # Now enable the motor
                    self.target_motor_control.enable(target_motor)
                    enabled_count += 1
                    motor_options.append(f"0x{slave_id:02x}")
                    # Initialize target positions
                    self.target_test_positions[slave_id] = 0.0
                    print(f"Enabled target motor 0x{slave_id:02x} ({target_motor.MotorType.name})")
                except Exception as e:
                    print(f"Failed to initialize target motor 0x{slave_id:02x}: {e}")
                    
            if enabled_count == 0:
                self.target_test_status_var.set("Failed to enable any target motors")
                return
                
            # Update motor selection dropdown
            self.target_motor_select_combo['values'] = motor_options
            if motor_options:
                self.target_motor_select_combo.set(motor_options[0])
                    
            self.is_target_testing = True
            self.target_test_thread = threading.Thread(target=self.target_test_loop, daemon=True)
            self.target_test_thread.start()
            self.target_test_status_var.set(f"Target test started - enabled {enabled_count}/{len(self.target_motors)} motors")
            
        except Exception as e:
            self.target_test_status_var.set(f"Error starting target test: {str(e)}")
            return
    
    def stop_target_test(self):
        self.is_target_testing = False
        if self.target_test_thread:
            self.target_test_thread.join(timeout=1.0)
            
        # Disable all target motors when test stops
        try:
            for slave_id, target_motor in self.target_motors.items():
                try:
                    self.target_motor_control.disable(target_motor)
                    print(f"Disabled target motor 0x{slave_id:02x}")
                except Exception as e:
                    print(f"Error disabling target motor 0x{slave_id:02x}: {e}")
                    
            # Clear test data and dropdown
            self.target_test_data = {}
            self.target_test_positions = {}
            self.target_motor_select_combo['values'] = []
            self.target_motor_select_combo.set("")
            
        except Exception as e:
            print(f"Error during target motor shutdown: {e}")
            
        self.target_test_status_var.set("Target test stopped (all target motors disabled)")
    
    def target_test_loop(self):
        while self.is_target_testing:
            try:
                # Read data from all target motors
                for slave_id, target_motor in self.target_motors.items():
                    try:
                        # Refresh motor status to get latest data from hardware
                        self.target_motor_control.refresh_motor_status(target_motor)
                        
                        # Get the motor values
                        current_position = target_motor.getPosition()
                        velocity = target_motor.getVelocity()
                        torque = target_motor.getTorque()
                        target_position = self.target_test_positions.get(slave_id, 0.0)
                        
                        # Calculate status
                        error = abs(current_position - target_position)
                        if error < 0.01:
                            status = "At Target"
                        elif abs(velocity) > 0.01:
                            status = "Moving"
                        else:
                            status = "Stopped"
                        
                        # Store target test data
                        self.target_test_data[slave_id] = {
                            "target_position": target_position,
                            "current_position": current_position,
                            "velocity": velocity,
                            "torque": torque,
                            "status": status,
                            "master_id": target_motor.MasterID,
                            "timestamp": time.time()
                        }
                    
                    except Exception as e:
                        print(f"Error reading target motor 0x{slave_id:02x}: {e}")
                        # Create placeholder data
                        if slave_id not in self.target_test_data:
                            self.target_test_data[slave_id] = {
                                "target_position": 0.0,
                                "current_position": 0.0,
                                "velocity": 0.0,
                                "torque": 0.0,
                                "status": "Error",
                                "master_id": target_motor.MasterID,
                                "timestamp": time.time()
                            }
                
                # Update display
                self.root.after(0, self.update_target_test_display)
                
                time.sleep(0.1)  # 10Hz update rate for testing
                
            except Exception as e:
                print(f"Error in target test loop: {e}")
                time.sleep(0.1)
    
    def update_target_test_display(self):
        # Clear existing items
        for item in self.target_test_tree.get_children():
            self.target_test_tree.delete(item)
        
        # Add current target test data
        if not self.target_test_data:
            # Show placeholder if no data yet
            for slave_id, motor in self.target_motors.items():
                self.target_test_tree.insert("", "end", values=(
                    f"0x{slave_id:02x}",
                    f"0x{motor.MasterID:02x}",
                    "No Data",
                    "No Data",
                    "No Data",
                    "No Data",
                    "No Data"
                ))
        else:
            for slave_id, data in self.target_test_data.items():
                self.target_test_tree.insert("", "end", values=(
                    f"0x{slave_id:02x}",
                    f"0x{data['master_id']:02x}",
                    f"{data['target_position']:.3f}",
                    f"{data['current_position']:.3f}",
                    f"{data['velocity']:.3f}",
                    f"{data['torque']:.3f}",
                    data['status']
                ))
    
    def move_target_motor(self):
        if not self.is_target_testing:
            self.target_test_status_var.set("Start target test first")
            return
            
        selected_motor = self.target_motor_select_var.get()
        if not selected_motor:
            self.target_test_status_var.set("Select a motor first")
            return
            
        try:
            # Parse motor ID
            slave_id = int(selected_motor, 16)
            target_position = float(self.target_position_var.get())
            
            if slave_id not in self.target_motors:
                self.target_test_status_var.set(f"Motor {selected_motor} not found")
                return
                
            motor = self.target_motors[slave_id]
            
            # Get current position before moving
            self.target_motor_control.refresh_motor_status(motor)
            current_pos = motor.getPosition()
            
            # Safety check before sending position command
            if not self.is_safe_position_command(slave_id, target_position):
                self.target_test_status_var.set(f"SAFETY: Blocked dangerous position {target_position:.3f} for motor {selected_motor}")
                return
            
            # Send position command
            self.target_motor_control.control_Pos_Vel(motor, target_position, 10.0)
            # Update last position tracking
            self.last_target_positions[slave_id] = target_position
            
            # Update target position tracking
            self.target_test_positions[slave_id] = target_position
            
            # Wait a moment and check if motor started moving
            time.sleep(1)
            self.target_motor_control.refresh_motor_status(motor)
            new_pos = motor.getPosition()
            velocity = motor.getVelocity()
            
            movement = abs(new_pos - current_pos)
            
            self.target_test_status_var.set(f"Moving motor {selected_motor} to {target_position:.3f} rad")
            print(f"Commanded motor 0x{slave_id:02x} to position {target_position:.3f} rad")
            print(f"  Current position: {current_pos:.3f} -> {new_pos:.3f} (moved {movement:.3f})")
            print(f"  Current velocity: {velocity:.3f} rad/s")
            
            if abs(velocity) < 0.01 and movement < 0.001:
                print(f"  WARNING: Motor may not be moving! Check:")
                print(f"    - Motor power/connections")
                print(f"    - Motor enable status") 
                print(f"    - Position limits")
                print(f"    - Motor configuration")
            
        except ValueError:
            self.target_test_status_var.set("Invalid position value")
        except Exception as e:
            self.target_test_status_var.set(f"Error moving motor: {str(e)}")
    
    def set_target_zero(self):
        if not self.is_target_testing:
            self.target_test_status_var.set("Start target test first")
            return
            
        selected_motor = self.target_motor_select_var.get()
        if not selected_motor:
            self.target_test_status_var.set("Select a motor first")
            return
            
        try:
            # Parse motor ID
            slave_id = int(selected_motor, 16)
            
            if slave_id not in self.target_motors:
                self.target_test_status_var.set(f"Motor {selected_motor} not found")
                return
                
            motor = self.target_motors[slave_id]
            
            # Set zero position
            self.target_motor_control.set_zero_position(motor)
            
            # Update target position tracking
            self.target_test_positions[slave_id] = 0.0
            self.target_position_var.set("0.0")
            
            self.target_test_status_var.set(f"Set zero position for motor {selected_motor}")
            print(f"Set zero position for motor 0x{slave_id:02x}")
            
        except Exception as e:
            self.target_test_status_var.set(f"Error setting zero: {str(e)}")
    
    def start_port_monitoring(self):
        """Start the USB port monitoring thread"""
        self.is_monitoring_ports = True
        self.port_refresh_thread = threading.Thread(target=self.port_monitoring_loop, daemon=True)
        self.port_refresh_thread.start()
        # Initial port scan
        self.refresh_ports()
    
    def stop_port_monitoring(self):
        """Stop the USB port monitoring thread"""
        self.is_monitoring_ports = False
        if self.port_refresh_thread:
            self.port_refresh_thread.join(timeout=1.0)
    
    def port_monitoring_loop(self):
        """Background thread that refreshes port list every 2 seconds"""
        while self.is_monitoring_ports:
            try:
                self.refresh_ports()
                time.sleep(2.0)  # Refresh every 2 seconds
            except Exception as e:
                print(f"Error in port monitoring: {e}")
                time.sleep(2.0)
    
    def refresh_ports_manual(self):
        """Manual refresh triggered by button"""
        self.refresh_ports()
    
    def refresh_ports(self):
        """Scan for available USB ports and update the display"""
        try:
            # Get list of available serial ports
            ports = list(serial.tools.list_ports.comports())
            
            # Update the available ports list
            self.available_ports = []
            for port in ports:
                port_info = {
                    'device': port.device,
                    'description': port.description or 'Unknown',
                    'hwid': port.hwid or 'Unknown'
                }
                self.available_ports.append(port_info)
            
            # Update combo boxes with port names
            port_names = [port['device'] for port in self.available_ports]
            self.reader_port_combo['values'] = port_names
            self.target_port_combo['values'] = port_names
            
            # Update the GUI display
            self.root.after(0, self.update_ports_display)
            
        except Exception as e:
            print(f"Error refreshing ports: {e}")
    
    def update_ports_display(self):
        """Update the ports treeview with current port information"""
        # Clear existing items
        for item in self.ports_tree.get_children():
            self.ports_tree.delete(item)
        
        # Add current ports
        for port_info in self.available_ports:
            self.ports_tree.insert("", "end", values=(
                port_info['device'],
                port_info['description'],
                port_info['hwid']
            ))
        
        # Update status
        port_count = len(self.available_ports)
        self.port_status_var.set(f"Auto-refresh: ON (every 2s) - {port_count} ports found")
    
    def on_port_double_click(self, _):
        """Handle double-click on port list to fill port fields"""
        selected = self.ports_tree.selection()
        if selected:
            item = self.ports_tree.item(selected[0])
            port_name = item['values'][0]
            
            # Simple dialog to choose which port to set
            from tkinter import messagebox
            choice = messagebox.askyesnocancel(
                "Select Port", 
                f"Set '{port_name}' as:\n\nYes = Reader Port\nNo = Target Port\nCancel = Do nothing"
            )
            
            if choice is True:  # Yes - Reader port
                self.reader_port_var.set(port_name)
                self.update_status(f"Reader port set to {port_name}")
            elif choice is False:  # No - Target port
                self.target_port_var.set(port_name)
                self.update_status(f"Target port set to {port_name}")
    
    def update_status(self, message):
        self.status_var.set(message)
        print(f"Status: {message}")
    
    def save_config(self):
        print("=== SAVE CONFIG DEBUG ===")
        print(f"Reader motors: {list(self.reader_motors.keys())}")
        print(f"Target motors: {list(self.target_motors.keys())}")
        print(f"Motor mapping: {self.motor_mapping}")
        
        config = {
            "reader_port": self.reader_port_var.get(),
            "reader_baudrate": self.reader_baud_var.get(),
            "target_port": self.target_port_var.get(),
            "target_baudrate": self.target_baud_var.get(),
            "reader_motors": [{"slave_id": k, "master_id": v.MasterID, "motor_type": v.MotorType.name} for k, v in self.reader_motors.items()],
            "target_motors": [{"slave_id": k, "master_id": v.MasterID, "motor_type": v.MotorType.name} for k, v in self.target_motors.items()],
            "motor_mapping": self.motor_mapping
        }
        print(f"Config to save: {config}")
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            self.update_status("Configuration saved")
        except Exception as e:
            self.update_status(f"Error saving config: {str(e)}")
    
    def load_config(self):
        print(f"Loading config from: {self.config_file}")
        try:
            if os.path.exists(self.config_file):
                print("Config file exists, loading...")
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                print(f"Loaded config: {config}")
                
                self.reader_port_var.set(config.get("reader_port", "/dev/ttyACM1"))
                self.reader_baud_var.set(config.get("reader_baudrate", "921600"))
                self.target_port_var.set(config.get("target_port", "/dev/ttyACM0"))
                self.target_baud_var.set(config.get("target_baudrate", "921600"))
                
                # Load motor mapping
                self.motor_mapping = config.get("motor_mapping", {})
                # Convert string keys back to integers if needed
                if self.motor_mapping and isinstance(list(self.motor_mapping.keys())[0], str):
                    self.motor_mapping = {int(k): int(v) for k, v in self.motor_mapping.items()}
                    
                # Load saved motor configurations
                self.load_saved_motors(config)
                    
                self.update_mapping_display()
                
                self.update_status("Configuration loaded")
        except Exception as e:
            self.update_status(f"Error loading config: {str(e)}")
    
    def load_saved_motors(self, config):
        """Load saved motor configurations and recreate motor objects"""
        print(f"load_saved_motors called with config: {config}")
        try:
            # Store all motor configurations
            self.pending_reader_configs = config.get("reader_motors", [])
            self.pending_target_configs = config.get("target_motors", [])
            print(f"Pending reader configs: {self.pending_reader_configs}")
            print(f"Pending target configs: {self.pending_target_configs}")
            
            # Load reader motors immediately if connected
            for motor_config in self.pending_reader_configs:
                if self.reader_motor_control:  # Only load if reader port is connected
                    self._load_reader_motor(motor_config)
            
            # Load target motors immediately if connected
            for motor_config in self.pending_target_configs:
                if self.target_motor_control:  # Only load if target port is connected
                    self._load_target_motor(motor_config)
                    
            # Update displays (will show pending configs even if not connected)
            self.update_reader_motors_display()
            self.update_target_motors_display()
            
            if self.pending_reader_configs or self.pending_target_configs:
                self.update_status(f"Loaded {len(self.pending_reader_configs)} reader and {len(self.pending_target_configs)} target motor configs")
                
        except Exception as e:
            print(f"Error loading saved motors: {e}")
            self.update_status(f"Error loading saved motors: {str(e)}")
    
    def _load_reader_motor(self, motor_config):
        """Load a single reader motor from config"""
        slave_id = motor_config["slave_id"]
        master_id = motor_config["master_id"]
        motor_type_name = motor_config["motor_type"]
        
        motor_type = self.get_motor_type_enum(motor_type_name)
        motor = Motor(motor_type, slave_id, master_id)
        
        # Initialize the loaded motor (like when manually added)
        self.reader_motor_control.addMotor(motor)
        if self.reader_motor_control.switchControlMode(motor, Control_Type.POS_VEL):
            print(f"Loaded reader motor 0x{slave_id:02x} switched to POS_VEL mode")
        self.reader_motor_control.save_motor_param(motor)
        self.reader_motor_control.set_zero_position(motor)
        
        self.reader_motors[slave_id] = motor
        print(f"Loaded reader motor {motor_type_name} 0x{slave_id:02x} (master: 0x{master_id:02x})")
    
    def _load_target_motor(self, motor_config):
        """Load a single target motor from config"""
        slave_id = motor_config["slave_id"]
        master_id = motor_config["master_id"]
        motor_type_name = motor_config["motor_type"]
        
        motor_type = self.get_motor_type_enum(motor_type_name)
        motor = Motor(motor_type, slave_id, master_id)
        
        # Initialize the loaded motor (like when manually added)
        self.target_motor_control.addMotor(motor)
        if self.target_motor_control.switchControlMode(motor, Control_Type.POS_VEL):
            print(f"Loaded target motor 0x{slave_id:02x} switched to POS_VEL mode")
        self.target_motor_control.save_motor_param(motor)
        self.target_motor_control.set_zero_position(motor)
        
        self.target_motors[slave_id] = motor
        print(f"Loaded target motor {motor_type_name} 0x{slave_id:02x} (master: 0x{master_id:02x})")
    
    def is_safe_position_command(self, target_id, new_position):
        """Check if a position command is safe to prevent dangerous jumps"""
        # Absolute position limit check
        if abs(new_position) > self.position_limit:
            print(f"WARNING: Position {new_position:.3f} exceeds limit ±{self.position_limit} for motor 0x{target_id:02x}")
            return False
        
        # Position jump check
        if target_id in self.last_target_positions:
            last_pos = self.last_target_positions[target_id]
            position_jump = abs(new_position - last_pos)
            
            if position_jump > self.max_position_jump:
                print(f"DANGER: Large position jump detected for motor 0x{target_id:02x}")
                print(f"  Last position: {last_pos:.3f} rad")
                print(f"  New position: {new_position:.3f} rad") 
                print(f"  Jump: {position_jump:.3f} rad (max allowed: {self.max_position_jump} rad)")
                print(f"  BLOCKING COMMAND to prevent dangerous movement!")
                return False
        
        return True
    
    def update_safety_limits(self):
        """Update safety limits from GUI values"""
        try:
            self.max_position_jump = float(self.max_jump_var.get())
            self.position_limit = float(self.pos_limit_var.get())
            self.update_status(f"Safety limits updated: Jump={self.max_position_jump:.1f}rad, Limit=±{self.position_limit:.1f}rad")
            print(f"Safety limits updated: max_jump={self.max_position_jump}, position_limit={self.position_limit}")
        except ValueError:
            self.update_status("Invalid safety limit values - using defaults")
            self.max_position_jump = 2.0
            self.position_limit = 10.0
    
    def on_closing(self):
        # Prevent double-execution from atexit callback
        if hasattr(self, '_closing_called'):
            return
        self._closing_called = True
        
        self.stop_monitoring()
        if self.is_reader_testing:
            self.stop_reader_test()
        if self.is_target_testing:
            self.stop_target_test()
        self.stop_port_monitoring()
        # Save config BEFORE disconnecting (which clears motor dictionaries)
        self.save_config()
        self.disconnect_reader()
        self.disconnect_target()
        try:
            self.root.destroy()
        except:
            pass  # Ignore destroy errors
    
    def get_motor_type_enum(self, type_string):
        """Convert motor type string to DM_Motor_Type enum"""
        motor_type_map = {
            "DM4310": DM_Motor_Type.DM4310,
            "DM4310_48V": DM_Motor_Type.DM4310_48V,
            "DM4340": DM_Motor_Type.DM4340,
            "DM4340_48V": DM_Motor_Type.DM4340_48V,
            "DM6006": DM_Motor_Type.DM6006,
            "DM8006": DM_Motor_Type.DM8006,
            "DM8009": DM_Motor_Type.DM8009,
            "DM10010": DM_Motor_Type.DM10010,
            "DM10010L": DM_Motor_Type.DM10010L,
            "DMG6220": DM_Motor_Type.DMG6220,
            "DMH3510": DM_Motor_Type.DMH3510,
            "DMH6215": DM_Motor_Type.DMH6215
        }
        return motor_type_map.get(type_string, DM_Motor_Type.DM4310)

def main():
    root = tk.Tk()
    _ = PositionReaderGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()