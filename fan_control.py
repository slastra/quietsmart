#!/usr/bin/env python3
"""
Fan Control GUI - Control ESP32 fan via MQTT
"""

import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt
import json
import threading
import time

class FanControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Fan Controller")
        self.root.geometry("400x500")
        
        # MQTT settings
        self.mqtt_broker = "desk.local"
        self.mqtt_port = 1883
        self.mqtt_client = None
        self.connected = False
        
        # Fan state
        self.fan_enabled = False
        self.fan_speed = 0
        self.fan_rpm = 0
        
        self.setup_ui()
        self.setup_mqtt()
        
    def setup_ui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Fan Control", 
                               font=('Arial', 20, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Connection status
        self.status_label = ttk.Label(main_frame, text="Connecting to MQTT...", 
                                     foreground="orange")
        self.status_label.grid(row=1, column=0, columnspan=2, pady=5)
        
        # Speed control frame
        speed_frame = ttk.LabelFrame(main_frame, text="Speed Control", padding="10")
        speed_frame.grid(row=2, column=0, columnspan=2, pady=20, sticky=(tk.W, tk.E))
        
        # Speed slider
        self.speed_var = tk.IntVar(value=0)
        self.speed_slider = ttk.Scale(speed_frame, from_=0, to=100, 
                                     orient=tk.HORIZONTAL, variable=self.speed_var,
                                     command=self.on_speed_change)
        self.speed_slider.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Speed label
        self.speed_label = ttk.Label(speed_frame, text="Speed: 0%", 
                                    font=('Arial', 14))
        self.speed_label.grid(row=1, column=0, columnspan=2)
        
        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=2, pady=20)
        
        self.on_button = ttk.Button(button_frame, text="Turn ON", 
                                   command=self.turn_on, width=15)
        self.on_button.grid(row=0, column=0, padx=5)
        
        self.off_button = ttk.Button(button_frame, text="Turn OFF", 
                                    command=self.turn_off, width=15)
        self.off_button.grid(row=0, column=1, padx=5)
        
        # Status frame
        status_frame = ttk.LabelFrame(main_frame, text="Fan Status", padding="10")
        status_frame.grid(row=4, column=0, columnspan=2, pady=20, sticky=(tk.W, tk.E))
        
        # Fan status
        self.fan_status_label = ttk.Label(status_frame, text="Fan: OFF", 
                                         font=('Arial', 12))
        self.fan_status_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        
        # RPM display
        self.rpm_label = ttk.Label(status_frame, text="RPM: 0", 
                                  font=('Arial', 12))
        self.rpm_label.grid(row=1, column=0, sticky=tk.W, pady=2)
        
        # Last update
        self.update_label = ttk.Label(status_frame, text="Last update: Never", 
                                     font=('Arial', 10))
        self.update_label.grid(row=2, column=0, sticky=tk.W, pady=2)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        speed_frame.columnconfigure(0, weight=1)
        
    def setup_mqtt(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Start connection in background thread
        threading.Thread(target=self.connect_mqtt, daemon=True).start()
        
    def connect_mqtt(self):
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_forever()
        except Exception as e:
            self.root.after(0, self.update_status, f"Connection failed: {str(e)}", "red")
            
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            client.subscribe("fan/status")
            self.root.after(0, self.update_status, "Connected to MQTT", "green")
        else:
            self.root.after(0, self.update_status, f"Connection failed: RC={rc}", "red")
            
    def on_mqtt_disconnect(self, client, userdata, rc):
        self.connected = False
        self.root.after(0, self.update_status, "Disconnected from MQTT", "red")
        
    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            self.fan_enabled = data.get('enabled', False)
            self.fan_speed = data.get('speed', 0)
            self.fan_rpm = data.get('rpm', 0)
            
            # Update UI in main thread
            self.root.after(0, self.update_ui_status)
        except Exception as e:
            print(f"Error parsing message: {e}")
            
    def update_ui_status(self):
        # Update fan status
        status_text = f"Fan: {'ON' if self.fan_enabled else 'OFF'}"
        self.fan_status_label.config(text=status_text, 
                                    foreground="green" if self.fan_enabled else "red")
        
        # Update speed slider without triggering callback
        self.speed_slider.set(self.fan_speed)
        self.speed_label.config(text=f"Speed: {self.fan_speed}%")
        
        # Update RPM
        self.rpm_label.config(text=f"RPM: {self.fan_rpm}")
        
        # Update timestamp
        current_time = time.strftime("%H:%M:%S")
        self.update_label.config(text=f"Last update: {current_time}")
        
    def update_status(self, text, color):
        self.status_label.config(text=text, foreground=color)
        
    def on_speed_change(self, value):
        speed = int(float(value))
        self.speed_label.config(text=f"Speed: {speed}%")
        
    def turn_on(self):
        if self.connected:
            command = {"action": "on"}
            self.mqtt_client.publish("fan/control", json.dumps(command))
            
    def turn_off(self):
        if self.connected:
            command = {"action": "off"}
            self.mqtt_client.publish("fan/control", json.dumps(command))
            
    def send_speed(self):
        if self.connected and self.fan_enabled:
            speed = self.speed_var.get()
            command = {"speed": speed}
            self.mqtt_client.publish("fan/control", json.dumps(command))

def main():
    root = tk.Tk()
    app = FanControlGUI(root)
    
    # Bind slider release to send speed command
    app.speed_slider.bind("<ButtonRelease-1>", lambda e: app.send_speed())
    
    root.mainloop()

if __name__ == "__main__":
    main()