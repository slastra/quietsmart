#!/usr/bin/env python3
"""
ESP32 Fan Controller - GTK4 Edition
Modern, beautiful interface for controlling ESP32 fan via MQTT
"""

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw, GLib, Gio, Gdk
import paho.mqtt.client as mqtt
import json
import threading
import time
from collections import deque
import cairo

class FanControlWindow(Adw.ApplicationWindow):
    def __init__(self, app):
        super().__init__(application=app)
        
        # Window setup
        self.set_title("ESP32 Fan Controller")
        self.set_default_size(350, 400)
        self.set_size_request(300, 350)  # Set minimum size
        self.set_resizable(True)  # Allow resizing but respect minimum
        
        # MQTT settings
        self.mqtt_broker = "hostname.local"
        self.mqtt_port = 1883
        self.mqtt_client = None
        self.connected = False
        
        # Fan state
        self.fan_speed = 0
        self.fan_rpm = 0
        self.temperature_c = 0.0
        self.humidity = 0.0
        self.first_update_received = False
        
        # Animation state
        self.displayed_rpm = 0
        self.target_rpm = 0
        self.rpm_animation_timer = None
        self.displayed_temp = 0.0
        self.target_temp = 0.0
        self.temp_animation_timer = None
        self.displayed_humidity = 0.0
        self.target_humidity = 0.0
        self.humidity_animation_timer = None
        
        # Data history for graphs (store last 10 minutes at 5-second intervals)
        self.max_history_points = 120  # 10 minutes / 5 seconds
        self.history_time = deque(maxlen=self.max_history_points)
        self.history_speed = deque(maxlen=self.max_history_points)
        self.history_rpm = deque(maxlen=self.max_history_points)
        self.history_temp = deque(maxlen=self.max_history_points)
        self.history_humidity = deque(maxlen=self.max_history_points)
        
        self.setup_ui()
        self.setup_mqtt()
        
    def setup_ui(self):
        # Create the main toolbar view (Adwaita pattern)
        toolbar_view = Adw.ToolbarView()
        self.set_content(toolbar_view)
        
        # Header bar
        header_bar = Adw.HeaderBar()
        header_bar.set_title_widget(Adw.WindowTitle(title="Fan Controller", subtitle="ESP32 Control Panel"))
        toolbar_view.add_top_bar(header_bar)
        
        # Main content container with separator
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        
        # Add separator line under header
        separator = Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL)
        main_box.append(separator)
        
        toolbar_view.set_content(main_box)
        
        # Loading spinner - centered in window
        self.loading_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=16)
        self.loading_box.set_halign(Gtk.Align.CENTER)
        self.loading_box.set_valign(Gtk.Align.CENTER)
        self.loading_box.set_vexpand(True)
        
        self.spinner = Gtk.Spinner()
        self.spinner.set_size_request(32, 32)  # Make spinner larger
        self.spinner.start()
        self.loading_box.append(self.spinner)
        
        loading_label = Gtk.Label()
        loading_label.set_text("Connecting to ESP32...")
        loading_label.add_css_class("dim-label")
        self.loading_box.append(loading_label)
        
        main_box.append(self.loading_box)
        
        # Content area with compact sizing (initially hidden)
        self.content_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        self.content_box.set_margin_top(12)
        self.content_box.set_margin_bottom(12)
        self.content_box.set_margin_start(16)
        self.content_box.set_margin_end(16)
        self.content_box.set_vexpand(True)
        self.content_box.set_visible(False)  # Hide initially
        main_box.append(self.content_box)
        
        # Fan status card
        self.create_status_card(self.content_box)
        
        # Speed control card
        self.create_speed_control_card(self.content_box)
        
        # History graph card
        self.create_history_graph_card(self.content_box)
        
        
    def create_status_card(self, parent):
        # Status group with indicator
        status_group = Adw.PreferencesGroup()
        status_group.set_title("Status")
        
        # Activity indicator (small dot) - positioned on the right
        self.activity_indicator = Gtk.Label()
        self.activity_indicator.set_text("●")
        self.activity_indicator.add_css_class("activity-dot")
        self.activity_indicator.set_visible(False)
        self.activity_indicator.set_halign(Gtk.Align.END)
        
        # Set indicator as header suffix (right side)
        status_group.set_header_suffix(self.activity_indicator)
        
        parent.append(status_group)
        
        
        # Speed row
        self.speed_row = Adw.ActionRow()
        self.speed_row.set_title("Speed")
        
        self.speed_label = Gtk.Label()
        self.speed_label.set_text("0%")
        self.speed_label.add_css_class("title-2")
        self.speed_label.set_size_request(50, -1)  # Fixed width
        self.speed_label.set_xalign(1.0)  # Right align
        self.speed_row.add_suffix(self.speed_label)
        
        status_group.add(self.speed_row)
        
        # RPM row
        self.rpm_row = Adw.ActionRow()
        self.rpm_row.set_title("RPM")
        
        self.rpm_label = Gtk.Label()
        self.rpm_label.set_text("0")
        self.rpm_label.add_css_class("title-2")
        self.rpm_label.set_size_request(70, -1)  # Fixed width for larger numbers
        self.rpm_label.set_xalign(1.0)  # Right align
        self.rpm_row.add_suffix(self.rpm_label)
        
        status_group.add(self.rpm_row)
        
        # Temperature row
        self.temp_row = Adw.ActionRow()
        self.temp_row.set_title("Temperature")
        
        self.temp_label = Gtk.Label()
        self.temp_label.set_text("32.0°F")
        self.temp_label.add_css_class("title-2")
        self.temp_label.set_size_request(80, -1)  # Fixed width for temp display
        self.temp_label.set_xalign(1.0)  # Right align
        self.temp_row.add_suffix(self.temp_label)
        
        status_group.add(self.temp_row)
        
        # Humidity row
        self.humidity_row = Adw.ActionRow()
        self.humidity_row.set_title("Humidity")
        
        self.humidity_label = Gtk.Label()
        self.humidity_label.set_text("0.0%")
        self.humidity_label.add_css_class("title-2")
        self.humidity_label.set_size_request(70, -1)  # Fixed width
        self.humidity_label.set_xalign(1.0)  # Right align
        self.humidity_row.add_suffix(self.humidity_label)
        
        status_group.add(self.humidity_row)
        
        
    def create_speed_control_card(self, parent):
        # Speed control group
        speed_group = Adw.PreferencesGroup()
        speed_group.set_title("Control")
        
        parent.append(speed_group)
        
        # Speed slider spanning full width
        self.speed_scale = Gtk.Scale()
        self.speed_scale.set_range(0, 100)
        self.speed_scale.set_value(0)
        self.speed_scale.set_digits(0)
        self.speed_scale.set_hexpand(True)
        self.speed_scale.set_margin_start(16)
        self.speed_scale.set_margin_end(16)
        self.speed_scale.set_margin_top(8)
        self.speed_scale.set_margin_bottom(8)
        self.speed_scale.add_mark(0, Gtk.PositionType.BOTTOM, "0")
        self.speed_scale.add_mark(50, Gtk.PositionType.BOTTOM, "50")
        self.speed_scale.add_mark(100, Gtk.PositionType.BOTTOM, "100")
        
        # Connect slider events
        self.speed_scale.connect("change-value", self.on_speed_change)
        
        # Timer for delayed speed sending
        self.speed_timer = None
        self.user_interacting = False
        
        # Add slider directly to group instead of using ActionRow
        speed_group.add(self.speed_scale)
        
    def create_history_graph_card(self, parent):
        # History graph group
        graph_group = Adw.PreferencesGroup()
        graph_group.set_title("History")
        graph_group.set_description("Last 10 minutes")
        
        parent.append(graph_group)
        
        # Create a frame for the graph with Adwaita styling
        graph_frame = Gtk.Frame()
        graph_frame.add_css_class("card")  # Adwaita card style
        
        # Create drawing area for graph
        self.graph_area = Gtk.DrawingArea()
        self.graph_area.set_size_request(-1, 200)  # Height of 200px
        self.graph_area.set_draw_func(self.draw_graph)
        
        # Add drawing area to frame
        graph_frame.set_child(self.graph_area)
        
        # Add frame to group
        graph_group.add(graph_frame)
        
    def draw_graph(self, area, cr, width, height):
        # Don't draw if no data
        if len(self.history_time) < 2:
            return
        
        # Margins - more right margin for labels
        margin_left = 10
        margin_right = 80  # Space for labels
        margin_top = 10
        margin_bottom = 10
        
        graph_width = width - margin_left - margin_right
        graph_height = height - margin_top - margin_bottom
        
        # Grid
        cr.set_source_rgba(0.5, 0.5, 0.5, 0.3)
        cr.set_line_width(0.5)
        
        # Horizontal grid lines
        for i in range(5):
            y = margin_top + (i * graph_height / 4)
            cr.move_to(margin_left, y)
            cr.line_to(width - margin_right, y)
            cr.stroke()
        
        # Get time range
        current_time = time.time()
        start_time = current_time - (10 * 60)  # 10 minutes ago
        
        # Draw each metric and collect end positions
        end_positions = []
        
        result = self.draw_metric_line(cr, self.history_speed, 0, 100, 
                                      margin_left, margin_top, graph_width, graph_height,
                                      0.2, 0.6, 1.0, 2)  # Blue for speed
        if result:
            end_positions.append((*result, 0.2, 0.6, 1.0, "Speed", "%"))
        
        result = self.draw_metric_line(cr, self.history_rpm, 0, 5000,
                                      margin_left, margin_top, graph_width, graph_height,
                                      0.2, 0.8, 0.2, 2)  # Green for RPM
        if result:
            end_positions.append((*result, 0.2, 0.8, 0.2, "RPM", ""))
        
        result = self.draw_metric_line(cr, self.history_temp, 32, 120,
                                      margin_left, margin_top, graph_width, graph_height,
                                      1.0, 0.3, 0.3, 2)  # Red for temperature
        if result:
            end_positions.append((*result, 1.0, 0.3, 0.3, "Temp", "°F"))
        
        result = self.draw_metric_line(cr, self.history_humidity, 0, 100,
                                      margin_left, margin_top, graph_width, graph_height,
                                      0.2, 0.8, 0.8, 2)  # Cyan for humidity
        if result:
            end_positions.append((*result, 0.2, 0.8, 0.8, "Humidity", "%"))
        
        # Draw labels at end positions
        self.draw_end_labels(cr, end_positions, width)
    
    def draw_metric_line(self, cr, data, min_val, max_val, x, y, width, height, r, g, b, line_width):
        if len(data) < 2:
            return
            
        cr.set_source_rgba(r, g, b, 0.8)
        cr.set_line_width(line_width)
        
        # Scale values, clamping to valid range
        scaled_data = []
        for val in data:
            clamped_val = max(min_val, min(val, max_val))
            scaled = (clamped_val - min_val) / (max_val - min_val)
            scaled_data.append(scaled)
        
        # Draw line
        last_x = 0
        last_y = 0
        for i, value in enumerate(scaled_data):
            px = x + (i / max(1, len(data) - 1)) * width
            py = y + height - (value * height)
            
            if i == 0:
                cr.move_to(px, py)
            else:
                cr.line_to(px, py)
            
            if i == len(scaled_data) - 1:
                last_x = px
                last_y = py
        
        cr.stroke()
        
        # Draw dot at the end
        cr.arc(last_x, last_y, 4, 0, 2 * 3.14159)
        cr.fill()
        
        # Return position for label
        return last_x, last_y, data[-1]
    
    def draw_end_labels(self, cr, end_positions, width):
        """Draw value labels at the end of each line"""
        cr.select_font_face("sans-serif", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(10)
        
        # Sort positions by Y coordinate to prevent overlap
        end_positions.sort(key=lambda x: x[1])
        
        # Draw each label
        last_y = -20  # Track last Y position to avoid overlap
        for x, y, value, r, g, b, name, suffix in end_positions:
            # Adjust Y if too close to previous label
            if abs(y - last_y) < 18:
                y = last_y + 18
            last_y = y
            
            # Just show the metric name
            text = name
            
            # Draw text without background, aligned to right edge
            cr.set_source_rgb(r * 0.8, g * 0.8, b * 0.8)  # Slightly darker for readability
            cr.move_to(x + 10, y + 3)
            cr.show_text(text)
    
        
    def setup_mqtt(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_log = self.on_mqtt_log  # Add logging
        
        # Start connection in background thread
        threading.Thread(target=self.connect_mqtt, daemon=True).start()
        
    def connect_mqtt(self):
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_forever()
        except Exception as e:
            GLib.idle_add(self.update_connection_status, f"Connection failed: {str(e)}", False)
            
    def on_mqtt_connect(self, client, userdata, flags, rc):
        print(f"MQTT Connect result: {rc}")
        if rc == 0:
            self.connected = True
            result = client.subscribe("fan/status")
            print(f"Subscribed to fan/status: {result}")
            GLib.idle_add(self.update_connection_status, "Connected to MQTT broker", True)
        else:
            GLib.idle_add(self.update_connection_status, f"Connection failed: RC={rc}", False)
            
    def on_mqtt_disconnect(self, client, userdata, rc):
        self.connected = False
        GLib.idle_add(self.update_connection_status, "Disconnected from MQTT", False)
        
    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            print(f"Received MQTT data: {data}")  # Debug output
            
            self.fan_speed = data.get('speed', 0)
            self.fan_rpm = data.get('rpm', 0)
            self.temperature_c = data.get('temp_c', 0.0)
            self.humidity = data.get('humidity', 0.0)
            
            print(f"Parsed - Speed: {self.fan_speed}%, RPM: {self.fan_rpm}, Temp: {self.temperature_c:.1f}°C, Humidity: {self.humidity:.1f}%")  # Debug output
            
            # Update UI in main thread
            GLib.idle_add(self.update_fan_status)
            
            # Hide spinner on first update
            if not self.first_update_received:
                self.first_update_received = True
                GLib.idle_add(self.hide_loading_spinner)
        except Exception as e:
            print(f"Error parsing message: {e}")
    
    def on_mqtt_log(self, client, userdata, level, buf):
        print(f"MQTT Log: {buf}")
            
    def update_connection_status(self, message, connected):
        # Connection status is now handled by spinner visibility
        if not connected:
            print(f"Connection issue: {message}")
        
    def update_fan_status(self):
        # Update speed (without triggering callback) - but only if user isn't interacting
        if not self.user_interacting:
            self.speed_scale.set_value(self.fan_speed)
            self.speed_label.set_text(f"{self.fan_speed}%")
        else:
            print(f"Skipping slider update - user is interacting (MQTT: {self.fan_speed}%, Slider: {int(self.speed_scale.get_value())}%)")
        
        # Update values with animation
        self.animate_rpm_to(self.fan_rpm)
        self.animate_temp_to(self.temperature_c)
        self.animate_humidity_to(self.humidity)
        
        # Add data to history
        current_time = time.time()
        self.history_time.append(current_time)
        self.history_speed.append(self.fan_speed)
        self.history_rpm.append(self.fan_rpm)
        temp_f = self.temperature_c * 9/5 + 32
        self.history_temp.append(temp_f)
        self.history_humidity.append(self.humidity)
        
        # Redraw graph
        if hasattr(self, 'graph_area'):
            self.graph_area.queue_draw()
        
        # Blink the activity indicator (only if not first update)
        if self.first_update_received:
            self.blink_activity_indicator()
        
    def on_speed_change(self, scale, scroll_type, value):
        # Update display immediately
        self.speed_label.set_text(f"{int(value)}%")
        print(f"Speed change event: {value}")
        
        # Set user interaction flag
        self.user_interacting = True
        
        # Cancel any existing timer
        if self.speed_timer:
            GLib.source_remove(self.speed_timer)
        
        # Set a new timer to send the command after user stops dragging (reduced to 150ms)
        self.speed_timer = GLib.timeout_add(150, self.send_delayed_speed_command, int(value))
        
        return False
    
    def send_delayed_speed_command(self, speed):
        # This gets called 150ms after the user stops moving the slider
        print(f"Sending delayed speed command: {speed}")
        if self.connected:
            self.send_speed_command(speed)
        
        # Clear interaction flag after 2 seconds to resume MQTT updates
        GLib.timeout_add(2000, self.clear_user_interaction)
        
        self.speed_timer = None
        return False  # Don't repeat the timer
    
    def clear_user_interaction(self):
        self.user_interacting = False
        print("User interaction cleared - resuming MQTT slider updates")
        return False
    
    def blink_activity_indicator(self):
        """Show and animate the activity indicator briefly"""
        self.activity_indicator.set_visible(True)
        self.activity_indicator.add_css_class("blink")
        
        # Hide and remove blink class after 1 second
        GLib.timeout_add(1000, self.hide_activity_indicator)
    
    def hide_activity_indicator(self):
        """Hide the activity indicator"""
        self.activity_indicator.remove_css_class("blink")
        self.activity_indicator.set_visible(False)
        return False
    
    def animate_rpm_to(self, target_rpm):
        """Animate RPM counting from current displayed value to target"""
        # Cancel any existing animation
        if self.rpm_animation_timer:
            GLib.source_remove(self.rpm_animation_timer)
            self.rpm_animation_timer = None
        
        self.target_rpm = target_rpm
        
        # If this is the first update or no animation needed
        if not self.first_update_received or self.displayed_rpm == target_rpm:
            self.displayed_rpm = target_rpm
            self.rpm_label.set_text(f"{self.displayed_rpm:,}")
            return
        
        # Start animation
        self.animate_rpm_step()
    
    def animate_rpm_step(self):
        """Single step of RPM animation"""
        diff = self.target_rpm - self.displayed_rpm
        
        # If we've reached the target, stop
        if diff == 0:
            self.rpm_animation_timer = None
            return False
        
        # Always step by 1 in the correct direction
        step = 1 if diff > 0 else -1
        self.displayed_rpm += step
        self.rpm_label.set_text(f"{self.displayed_rpm:,}")
        
        # Calculate delay to complete animation in ~2.5 seconds (before next update)
        # This leaves 0.5 seconds buffer before the 3-second update interval
        remaining_steps = max(1, abs(diff))
        delay = min(50, max(1, int(2500 / remaining_steps)))  # Cap at 50ms, min 1ms
        
        # Continue animation
        self.rpm_animation_timer = GLib.timeout_add(delay, self.animate_rpm_step)
        return False
    
    def animate_temp_to(self, target_temp):
        """Animate temperature counting from current displayed value to target"""
        # Cancel any existing animation
        if self.temp_animation_timer:
            GLib.source_remove(self.temp_animation_timer)
            self.temp_animation_timer = None
        
        self.target_temp = target_temp
        
        # If this is the first update or no animation needed
        if not self.first_update_received or abs(self.displayed_temp - target_temp) < 0.1:
            self.displayed_temp = target_temp
            temp_f = self.displayed_temp * 9/5 + 32
            self.temp_label.set_text(f"{temp_f:.1f}°F")
            return
        
        # Start animation
        self.animate_temp_step()
    
    def animate_temp_step(self):
        """Single step of temperature animation"""
        diff = self.target_temp - self.displayed_temp
        
        # If we've reached the target, stop
        if abs(diff) < 0.1:
            self.displayed_temp = self.target_temp
            temp_f = self.displayed_temp * 9/5 + 32
            self.temp_label.set_text(f"{temp_f:.1f}°F")
            self.temp_animation_timer = None
            return False
        
        # Step by 0.1 degrees in the correct direction
        step = 0.1 if diff > 0 else -0.1
        self.displayed_temp += step
        temp_f = self.displayed_temp * 9/5 + 32
        self.temp_label.set_text(f"{temp_f:.1f}°F")
        
        # Calculate delay to complete animation in ~2.5 seconds
        remaining_steps = max(1, int(abs(diff) / 0.1))
        delay = min(50, max(10, int(2500 / remaining_steps)))  # Cap at 50ms, min 10ms
        
        # Continue animation
        self.temp_animation_timer = GLib.timeout_add(delay, self.animate_temp_step)
        return False
    
    def animate_humidity_to(self, target_humidity):
        """Animate humidity counting from current displayed value to target"""
        # Cancel any existing animation
        if self.humidity_animation_timer:
            GLib.source_remove(self.humidity_animation_timer)
            self.humidity_animation_timer = None
        
        self.target_humidity = target_humidity
        
        # If this is the first update or no animation needed
        if not self.first_update_received or abs(self.displayed_humidity - target_humidity) < 0.1:
            self.displayed_humidity = target_humidity
            self.humidity_label.set_text(f"{self.displayed_humidity:.1f}%")
            return
        
        # Start animation
        self.animate_humidity_step()
    
    def animate_humidity_step(self):
        """Single step of humidity animation"""
        diff = self.target_humidity - self.displayed_humidity
        
        # If we've reached the target, stop
        if abs(diff) < 0.1:
            self.displayed_humidity = self.target_humidity
            self.humidity_label.set_text(f"{self.displayed_humidity:.1f}%")
            self.humidity_animation_timer = None
            return False
        
        # Step by 0.1% in the correct direction
        step = 0.1 if diff > 0 else -0.1
        self.displayed_humidity += step
        self.humidity_label.set_text(f"{self.displayed_humidity:.1f}%")
        
        # Calculate delay to complete animation in ~2.5 seconds
        remaining_steps = max(1, int(abs(diff) / 0.1))
        delay = min(50, max(10, int(2500 / remaining_steps)))  # Cap at 50ms, min 10ms
        
        # Continue animation
        self.humidity_animation_timer = GLib.timeout_add(delay, self.animate_humidity_step)
        return False  # Return False to stop the current timer
    
    def hide_loading_spinner(self):
        """Hide the loading spinner and show content after first successful update"""
        self.spinner.stop()
        self.loading_box.set_visible(False)
        self.content_box.set_visible(True)
        # Initialize displayed values to current values to avoid animation on first load
        self.displayed_rpm = self.fan_rpm
        self.displayed_temp = self.temperature_c
        self.displayed_humidity = self.humidity
        
            
    def send_speed_command(self, speed):
        if self.connected:
            # Ensure speed is sent as an integer, not string
            speed_int = int(speed)
            command = {"speed": speed_int}
            print(f"Sending speed command: speed={speed_int}, type={type(speed_int)}")
            self.send_command(command)
            
    def send_command(self, command):
        if self.connected:
            # Use compact JSON without spaces to match ESP32 parser
            json_str = json.dumps(command, separators=(',', ':'))
            print(f"Sending command: {json_str}")  # Debug output
            self.mqtt_client.publish("fan/control", json_str)

class FanControlApp(Adw.Application):
    def __init__(self):
        super().__init__(application_id="com.esp32.fancontrol")
        self.connect("activate", self.on_activate)
        
    def on_activate(self, app):
        # Apply custom CSS
        self.load_css()
        
        # Create and show window
        self.window = FanControlWindow(self)
        self.window.present()
        
    def load_css(self):
        css_provider = Gtk.CssProvider()
        css_provider.load_from_data("""
            .success {
                color: @success_color;
            }
            
            .error {
                color: @error_color;
            }
            
            .title-2 {
                font-weight: bold;
                font-size: 1.2em;
            }
            
            .caption {
                opacity: 0.7;
                font-size: 0.9em;
            }
            
            .pill {
                border-radius: 9999px;
            }
            
            scale {
                margin: 12px;
            }
            
            scale mark {
                font-size: 0.8em;
            }
            
            .dim-label {
                opacity: 0.7;
            }
            
            .activity-dot {
                color: @success_color;
                font-size: 12px;
                margin-top: -2px;
            }
            
            .blink {
                animation: blink-animation 0.5s ease-in-out infinite;
            }
            
            @keyframes blink-animation {
                0%, 100% { opacity: 1; }
                50% { opacity: 0.3; }
            }
        """.encode())
        
        # Get the default display
        from gi.repository import Gdk
        display = Gdk.Display.get_default()
        if display:
            Gtk.StyleContext.add_provider_for_display(
                display,
                css_provider,
                Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
            )

def main():
    app = FanControlApp()
    return app.run()

if __name__ == "__main__":
    main()
