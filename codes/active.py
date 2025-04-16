#!/usr/bin/env python3

import asyncio
import math
import time
import json
import os
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, 
                            PositionNedYaw, VelocityNedYaw)
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import LandedState

# Configuration
VELOCITY = 1.0       # m/s default translation
YAW_SPEED = 30.0     # deg/s default rotation
TAKEOFF_ALT = 3.0    # m default takeoff altitude
DEFAULT_ORBIT_RADIUS = 5.0  # m default orbit radius
CMD_UPDATE_RATE = 0.05  # 20Hz command update rate

# Colors for terminal output
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

class DroneController:
    def __init__(self):
        self.drone = System()
        self.offboard_active = False
        self.armed = False
        self.telemetry_task = None
        self.command_active = False
        self.current_command = None
        self.log_file = f"drone_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        self.home_position = None
        self.objects_detected = []

    async def connect(self, connection_string="udp://:14540"):
        """Connect to the drone"""
        print(f"{Colors.BLUE}🔌 Connecting to drone at {connection_string}...{Colors.ENDC}")
        await self.drone.connect(system_address=connection_string)
        
        print(f"{Colors.YELLOW}⏳ Waiting for connection...{Colors.ENDC}")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"{Colors.GREEN}✅ Connected to drone!{Colors.ENDC}")
                break
        
        print(f"{Colors.YELLOW}⏳ Waiting for GPS and home position...{Colors.ENDC}")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"{Colors.GREEN}✅ GPS & Home position OK!{Colors.ENDC}")
                break
                
        # Get home position for reference
        async for home in self.drone.telemetry.home():
            self.home_position = home
            self.log(f"Home position: Lat={home.latitude_deg:.6f}, Lon={home.longitude_deg:.6f}, Alt={home.absolute_altitude_m:.2f}m")
            break
        
        # Start telemetry task
        self.telemetry_task = asyncio.create_task(self.monitor_telemetry())
        
        return True

    async def arm(self):
        """Arm the drone"""
        if self.armed:
            print(f"{Colors.YELLOW}⚠️ Drone is already armed{Colors.ENDC}")
            return True
            
        print(f"{Colors.BLUE}🛡️ Arming...{Colors.ENDC}")
        
        # Try to arm a few times if it fails
        for i in range(3):
            try:
                await self.drone.action.arm()
                self.armed = True
                print(f"{Colors.GREEN}✅ Armed successfully!{Colors.ENDC}")
                return True
            except ActionError as e:
                print(f"{Colors.RED}❌ Arming failed (attempt {i+1}/3): {e}{Colors.ENDC}")
                await asyncio.sleep(1)
        
        print(f"{Colors.RED}❌ Failed to arm after multiple attempts{Colors.ENDC}")
        return False

    async def disarm(self):
        """Disarm the drone"""
        if not self.armed:
            print(f"{Colors.YELLOW}⚠️ Drone is already disarmed{Colors.ENDC}")
            return True
            
        print(f"{Colors.BLUE}🛡️ Disarming...{Colors.ENDC}")
        
        try:
            # Stop offboard mode if active
            if self.offboard_active:
                await self.stop_offboard()
                
            await self.drone.action.disarm()
            self.armed = False
            print(f"{Colors.GREEN}✅ Disarmed successfully!{Colors.ENDC}")
            return True
        except ActionError as e:
            print(f"{Colors.RED}❌ Disarming failed: {e}{Colors.ENDC}")
            return False

    async def takeoff(self, altitude=TAKEOFF_ALT):
        """Take off to a specified altitude"""
        if not self.armed:
            success = await self.arm()
            if not success:
                return False
                
        print(f"{Colors.BLUE}🚀 Taking off to {altitude}m...{Colors.ENDC}")
        
        try:
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            
            # Monitor altitude until takeoff is complete
            print(f"{Colors.YELLOW}⏳ Monitoring takeoff progress...{Colors.ENDC}")
            
            takeoff_complete = False
            timeout = time.time() + 30  # 30 second timeout
            
            while not takeoff_complete and time.time() < timeout:
                async for position in self.drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    print(f"  Current altitude: {current_alt:.2f}m / {altitude:.2f}m")
                    
                    if current_alt >= altitude * 0.95:  # Within 95% of target
                        takeoff_complete = True
                        break
                        
                await asyncio.sleep(1)
                
            if takeoff_complete:
                print(f"{Colors.GREEN}✅ Takeoff complete!{Colors.ENDC}")
                return True
            else:
                print(f"{Colors.RED}⚠️ Takeoff timed out. Current altitude may be below target.{Colors.ENDC}")
                return False
                
        except ActionError as e:
            print(f"{Colors.RED}❌ Takeoff failed: {e}{Colors.ENDC}")
            return False

    async def land(self):
        """Land the drone"""
        print(f"{Colors.BLUE}🛬 Landing...{Colors.ENDC}")
        
        try:
            # Stop any active command
            if self.command_active:
                await self.stop_current_command()
                
            # Stop offboard mode if active
            if self.offboard_active:
                await self.stop_offboard()
                
            await self.drone.action.land()
            
            # Monitor landing progress
            print(f"{Colors.YELLOW}⏳ Monitoring landing progress...{Colors.ENDC}")
            landing_complete = False
            
            async for landed_state in self.drone.telemetry.landed_state():
                if landed_state == LandedState.ON_GROUND:
                    landing_complete = True
                    break
                await asyncio.sleep(1)
                
            if landing_complete:
                print(f"{Colors.GREEN}✅ Landed successfully!{Colors.ENDC}")
                await self.disarm()
                return True
            else:
                print(f"{Colors.RED}⚠️ Landing state detection timed out{Colors.ENDC}")
                return False
                
        except ActionError as e:
            print(f"{Colors.RED}❌ Landing failed: {e}{Colors.ENDC}")
            return False

    async def return_to_launch(self):
        """Return to launch position and land"""
        print(f"{Colors.BLUE}🏠 Returning to launch position...{Colors.ENDC}")
        
        try:
            # Stop any active command
            if self.command_active:
                await self.stop_current_command()
                
            # Stop offboard mode if active
            if self.offboard_active:
                await self.stop_offboard()
                
            await self.drone.action.return_to_launch()
            print(f"{Colors.GREEN}✅ Return to launch initiated{Colors.ENDC}")
            return True
            
        except ActionError as e:
            print(f"{Colors.RED}❌ Return to launch failed: {e}{Colors.ENDC}")
            return False

    async def init_offboard(self):
        """Initialize offboard mode"""
        if self.offboard_active:
            return True
            
        if not self.armed:
            success = await self.arm()
            if not success:
                return False
                
        print(f"{Colors.BLUE}🟢 Initializing Offboard mode...{Colors.ENDC}")
        
        # Set initial setpoint (hover)
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            
        try:
            await self.drone.offboard.start()
            self.offboard_active = True
            print(f"{Colors.GREEN}✅ Offboard mode started{Colors.ENDC}")
            return True
            
        except OffboardError as e:
            print(f"{Colors.RED}❌ Offboard start failed: {e._result.result}{Colors.ENDC}")
            return False

    async def stop_offboard(self):
        """Stop offboard mode"""
        if not self.offboard_active:
            return True
            
        print(f"{Colors.BLUE}🛑 Stopping Offboard mode...{Colors.ENDC}")
        
        try:
            await self.drone.offboard.stop()
            self.offboard_active = False
            print(f"{Colors.GREEN}✅ Offboard mode stopped{Colors.ENDC}")
            return True
            
        except OffboardError as e:
            print(f"{Colors.RED}❌ Offboard stop failed: {e._result.result}{Colors.ENDC}")
            return False

    async def move_direction(self, direction, value):
        """Move in specified direction"""
        if not self.offboard_active:
            success = await self.init_offboard()
            if not success:
                return False
                
        # Calculate duration based on distance and velocity
        duration = value / VELOCITY
        
        # Map direction to velocity components
        velocity_map = {
            "forward":  (VELOCITY, 0.0, 0.0, 0.0),
            "backward": (-VELOCITY, 0.0, 0.0, 0.0),
            "right":    (0.0, VELOCITY, 0.0, 0.0),
            "left":     (0.0, -VELOCITY, 0.0, 0.0),
            "up":       (0.0, 0.0, -VELOCITY, 0.0),
            "down":     (0.0, 0.0, VELOCITY, 0.0),
            "yaw_right": (0.0, 0.0, 0.0, YAW_SPEED),
            "yaw_left":  (0.0, 0.0, 0.0, -YAW_SPEED),
        }
        
        if direction not in velocity_map:
            print(f"{Colors.RED}❌ Unknown direction: {direction}{Colors.ENDC}")
            return False
            
        vx, vy, vz, yaw_rate = velocity_map[direction]
        
        # For yaw, duration is angle / yaw_rate
        if direction in ["yaw_right", "yaw_left"]:
            duration = value / YAW_SPEED
            
        print(f"{Colors.BLUE}🚁 Moving {direction} for {duration:.1f}s at {VELOCITY:.1f}m/s{Colors.ENDC}")
        
        # Set current command
        self.command_active = True
        self.current_command = f"move_{direction}"
        
        # Get start position for tracking
        pos_start = await self.drone.telemetry.position().__anext__()
        
        # Stream velocity commands
        end_time = asyncio.get_event_loop().time() + duration
        
        try:
            while asyncio.get_event_loop().time() < end_time and self.command_active:
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(vx, vy, vz, yaw_rate))
                await asyncio.sleep(CMD_UPDATE_RATE)
                
            # Stop movement
            if self.command_active:
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    
            self.command_active = False
            
            # Get final position for distance calculation
            pos_end = await self.drone.telemetry.position().__anext__()
            
            # Calculate approximate distance traveled (simple for demonstration)
            distance = math.sqrt(
                (pos_end.latitude_deg - pos_start.latitude_deg) ** 2 * 111111 ** 2 +
                (pos_end.longitude_deg - pos_start.longitude_deg) ** 2 * (111111 * math.cos(math.radians(pos_start.latitude_deg))) ** 2
            )
            
            print(f"{Colors.GREEN}✅ Movement complete. Approximate distance: {distance:.2f}m{Colors.ENDC}")
            return True
            
        except OffboardError as e:
            print(f"{Colors.RED}❌ Movement command failed: {e._result.result}{Colors.ENDC}")
            self.command_active = False
            return False

    async def stop_current_command(self):
        """Stop current active command"""
        if not self.command_active:
            return True
            
        print(f"{Colors.YELLOW}🛑 Stopping current command...{Colors.ENDC}")
        self.command_active = False
        
        if self.offboard_active:
            # Send hover command
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                
        print(f"{Colors.GREEN}✅ Command stopped{Colors.ENDC}")
        return True

    async def orbit(self, radius=DEFAULT_ORBIT_RADIUS, speed=2.0, clockwise=True, center_lat=None, center_lon=None):
        """Orbit around a point using do_orbit"""
        print(f"{Colors.BLUE}🔄 Starting orbit maneuver...{Colors.ENDC}")
        
        try:
            # Get current position if no center specified
            if center_lat is None or center_lon is None:
                position = await self.drone.telemetry.position().__anext__()
                center_lat = position.latitude_deg
                center_lon = position.longitude_deg
                
            # Get current altitude
            position = await self.drone.telemetry.position().__anext__()
            altitude = position.absolute_altitude_m
                
            print(f"{Colors.YELLOW}⏺️ Orbiting around Lat={center_lat:.6f}, Lon={center_lon:.6f}{Colors.ENDC}")
            print(f"{Colors.YELLOW}⏺️ Radius={radius}m, Speed={speed}m/s, Direction={'Clockwise' if clockwise else 'Counter-clockwise'}{Colors.ENDC}")
            
            # Configure orbit parameters
            yaw_behavior = 3  # Camera pointing at center (HOLD_FRONT_TO_CIRCLE_CENTER)
            
            # Stop offboard mode if active (do_orbit doesn't use offboard)
            if self.offboard_active:
                await self.stop_offboard()
                
            # Set current command
            self.command_active = True
            self.current_command = "orbit"
            
            # Start orbit
            await self.drone.action.do_orbit(
                radius_m=radius,
                velocity_ms=speed,
                is_clockwise=clockwise,
                yaw_behavior=yaw_behavior,
                latitude_deg=center_lat,
                longitude_deg=center_lon,
                absolute_altitude_m=altitude
            )
            
            print(f"{Colors.GREEN}✅ Orbit started. Use 'stop' command to end.{Colors.ENDC}")
            return True
            
        except ActionError as e:
            print(f"{Colors.RED}❌ Orbit command failed: {e}{Colors.ENDC}")
            self.command_active = False
            return False

    async def goto_position(self, lat, lon, alt=None, yaw_deg=None):
        """Go to a specific GPS position"""
        print(f"{Colors.BLUE}🎯 Moving to position...{Colors.ENDC}")
        
        try:
            # Get current altitude if not specified
            if alt is None:
                position = await self.drone.telemetry.position().__anext__()
                alt = position.absolute_altitude_m
                
            # Get current yaw if not specified
            if yaw_deg is None:
                attitude = await self.drone.telemetry.attitude_euler().__anext__()
                yaw_deg = attitude.yaw_deg
                
            # Stop offboard mode if active (goto doesn't use offboard)
            if self.offboard_active:
                await self.stop_offboard()
                
            print(f"{Colors.YELLOW}⏺️ Going to Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}{Colors.ENDC}")
            
            # Set current command
            self.command_active = True
            self.current_command = "goto"
            
            # Issue goto command
            await self.drone.action.goto_location(lat, lon, alt, yaw_deg)
            
            print(f"{Colors.GREEN}✅ Goto command issued. Use 'status' to monitor progress.{Colors.ENDC}")
            return True
            
        except ActionError as e:
            print(f"{Colors.RED}❌ Goto command failed: {e}{Colors.ENDC}")
            self.command_active = False
            return False

    async def monitor_telemetry(self):
        """Monitor and log telemetry data"""
        try:
            while True:
                # Position
                position = await self.drone.telemetry.position().__anext__()
                
                # Battery
                battery = await self.drone.telemetry.battery().__anext__()
                
                # Flight mode
                flight_mode = await self.drone.telemetry.flight_mode().__anext__()
                
                # Only log telemetry every ~5 seconds to avoid spam
                self.log(f"Position: Lat={position.latitude_deg:.6f}, Lon={position.longitude_deg:.6f}, " +
                      f"RelAlt={position.relative_altitude_m:.2f}m, AbsAlt={position.absolute_altitude_m:.2f}m")
                self.log(f"Battery: {battery.remaining_percent*100:.1f}%, Voltage={battery.voltage_v:.2f}V")
                self.log(f"Flight mode: {flight_mode}")
                
                await asyncio.sleep(5)
                
        except asyncio.CancelledError:
            # Task was cancelled, clean exit
            pass
        except Exception as e:
            print(f"{Colors.RED}❌ Telemetry monitoring error: {e}{Colors.ENDC}")

    async def check_target_detection(self, target_class="person"):
        """Check for detected target objects from JSON file"""
        try:
            detection_file = "target_detection.json"
            if not os.path.exists(detection_file):
                print(f"{Colors.YELLOW}⚠️ Detection file not found: {detection_file}{Colors.ENDC}")
                return None
                
            with open(detection_file, "r") as f:
                detections = json.load(f)
                
            # Find target with highest confidence
            best_target = None
            highest_conf = 0
            
            for detection in detections:
                if detection["class"] == target_class and detection["confidence"] > highest_conf:
                    best_target = detection
                    highest_conf = detection["confidence"]
                    
            if best_target and highest_conf > 0.5:
                print(f"{Colors.GREEN}✅ Found {target_class} with confidence {highest_conf:.2f}{Colors.ENDC}")
                return best_target
            else:
                print(f"{Colors.YELLOW}⚠️ No {target_class} detected with sufficient confidence{Colors.ENDC}")
                return None
                
        except Exception as e:
            print(f"{Colors.RED}❌ Error reading detection file: {e}{Colors.ENDC}")
            return None

    async def goto_object(self, object_class="person"):
        """Go to a detected object"""
        print(f"{Colors.BLUE}🔍 Searching for {object_class}...{Colors.ENDC}")
        
        target = await self.check_target_detection(object_class)
        if not target:
            print(f"{Colors.RED}❌ Target {object_class} not detected{Colors.ENDC}")
            return False
            
        print(f"{Colors.GREEN}✅ Target detected at pixel position: x={target['x']}, y={target['y']}{Colors.ENDC}")
        
        # Here you would convert pixel coordinates to world coordinates
        # For this example, we'll use a placeholder calculation
        # In a real system, this would use camera parameters and drone position
        
        # Get current position
        position = await self.drone.telemetry.position().__anext__()
        attitude = await self.drone.telemetry.attitude_euler().__anext__()
        
        # Very simplified conversion (not accurate, just for example)
        # You would need actual camera parameters and proper math
        lat_offset = (target['x'] - 320) / 10000.0  # Simplified example
        lon_offset = (target['y'] - 240) / 10000.0  # Simplified example
        
        target_lat = position.latitude_deg + lat_offset
        target_lon = position.longitude_deg + lon_offset
        
        print(f"{Colors.YELLOW}⏺️ Estimated target position: Lat={target_lat:.6f}, Lon={target_lon:.6f}{Colors.ENDC}")
        
        # Go to the target position
        success = await self.goto_position(target_lat, target_lon)
        
        if success:
            print(f"{Colors.GREEN}✅ Moving to {object_class} position{Colors.ENDC}")
            
            # Wait to reach the target
            await asyncio.sleep(10)
            
            print(f"{Colors.GREEN}✅ Reached target location{Colors.ENDC}")
            
            # Optional: orbit around target
            await self.orbit(radius=5.0, center_lat=target_lat, center_lon=target_lon)
            
        return success

    def log(self, message):
        """Log message to file and console"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        
        with open(self.log_file, "a") as f:
            f.write(log_message + "\n")

    async def print_status(self):
        """Print current drone status"""
        try:
            # Position
            position = await self.drone.telemetry.position().__anext__()
            
            # Battery
            battery = await self.drone.telemetry.battery().__anext__()
            
            # Attitude
            attitude = await self.drone.telemetry.attitude_euler().__anext__()
            
            # Flight mode
            flight_mode = await self.drone.telemetry.flight_mode().__anext__()
            
            # GPS info
            gps_info = await self.drone.telemetry.gps_info().__anext__()
            
            # In air status
            in_air = await self.drone.telemetry.in_air().__anext__()
            
            # Home position
            if self.home_position:
                home_lat = self.home_position.latitude_deg
                home_lon = self.home_position.longitude_deg
                home_alt = self.home_position.absolute_altitude_m
                
                # Calculate distance from home
                distance_from_home = math.sqrt(
                    (position.latitude_deg - home_lat) ** 2 * 111111 ** 2 +
                    (position.longitude_deg - home_lon) ** 2 * (111111 * math.cos(math.radians(home_lat))) ** 2
                )
            else:
                distance_from_home = "Unknown"
                
            # Print status report
            print(f"\n{Colors.BOLD}{Colors.HEADER}==== DRONE STATUS ===={Colors.ENDC}")
            print(f"{Colors.BOLD}Position:{Colors.ENDC}")
            print(f"  Latitude:  {position.latitude_deg:.6f}°")
            print(f"  Longitude: {position.longitude_deg:.6f}°")
            print(f"  Altitude (rel): {position.relative_altitude_m:.2f}m")
            print(f"  Altitude (abs): {position.absolute_altitude_m:.2f}m")
            print(f"  Distance from home: {distance_from_home if isinstance(distance_from_home, str) else f'{distance_from_home:.2f}m'}")
            
            print(f"{Colors.BOLD}Attitude:{Colors.ENDC}")
            print(f"  Roll:  {attitude.roll_deg:.2f}°")
            print(f"  Pitch: {attitude.pitch_deg:.2f}°")
            print(f"  Yaw:   {attitude.yaw_deg:.2f}°")
            
            print(f"{Colors.BOLD}Battery:{Colors.ENDC}")
            print(f"  Level: {battery.remaining_percent*100:.1f}%")
            print(f"  Voltage: {battery.voltage_v:.2f}V")
            
            print(f"{Colors.BOLD}GPS:{Colors.ENDC}")
            print(f"  Fix type: {gps_info.fix_type}")
            print(f"  Satellites: {gps_info.num_satellites}")
            
            print(f"{Colors.BOLD}Status:{Colors.ENDC}")
            print(f"  Flight mode: {flight_mode}")
            print(f"  Armed: {self.armed}")
            print(f"  In air: {in_air}")
            print(f"  Offboard active: {self.offboard_active}")
            print(f"  Command active: {self.command_active} ({self.current_command if self.command_active else 'None'})")
            print(f"{Colors.BOLD}{Colors.HEADER}======================{Colors.ENDC}\n")
            
            return True
            
        except Exception as e:
            print(f"{Colors.RED}❌ Error getting status: {e}{Colors.ENDC}")
            return False

async def print_help():
    """Print help menu"""
    help_text = f"""
{Colors.BOLD}{Colors.HEADER}==== DRONE CONTROLLER COMMANDS ===={Colors.ENDC}

{Colors.BOLD}Basic Commands:{Colors.ENDC}
  {Colors.GREEN}help{Colors.ENDC}                     - Show this help menu
  {Colors.GREEN}connect{Colors.ENDC} [address]        - Connect to drone (default: udp://:14540)
  {Colors.GREEN}arm{Colors.ENDC}                      - Arm the drone
  {Colors.GREEN}disarm{Colors.ENDC}                   - Disarm the drone
  {Colors.GREEN}takeoff{Colors.ENDC} [altitude]       - Take off to specified altitude (default: 3m)
  {Colors.GREEN}land{Colors.ENDC}                     - Land the drone
  {Colors.GREEN}rtl{Colors.ENDC}                      - Return to launch position
  {Colors.GREEN}stop{Colors.ENDC}                     - Stop current movement (hover)
  {Colors.GREEN}status{Colors.ENDC}                   - Show drone status
  {Colors.GREEN}exit{Colors.ENDC}                     - Exit the program

{Colors.BOLD}Movement Commands:{Colors.ENDC}
  {Colors.GREEN}forward{Colors.ENDC} <distance>       - Move forward specified distance in meters
  {Colors.GREEN}backward{Colors.ENDC} <distance>      - Move backward specified distance in meters
  {Colors.GREEN}left{Colors.ENDC} <distance>          - Move left specified distance in meters
  {Colors.GREEN}right{Colors.ENDC} <distance>         - Move right specified distance in meters
  {Colors.GREEN}up{Colors.ENDC} <distance>            - Move up specified distance in meters
  {Colors.GREEN}down{Colors.ENDC} <distance>          - Move down specified distance in meters
  {Colors.GREEN}yaw_left{Colors.ENDC} <degrees>       - Rotate left specified degrees
  {Colors.GREEN}yaw_right{Colors.ENDC} <degrees>      - Rotate right specified degrees

{Colors.BOLD}Advanced Commands:{Colors.ENDC}
  {Colors.GREEN}goto{Colors.ENDC} <lat> <lon> [alt]   - Go to specific GPS coordinates
  {Colors.GREEN}orbit{Colors.ENDC} [radius] [speed]   - Orbit around current position
  {Colors.GREEN}goto_person{Colors.ENDC}              - Detect and navigate to person
  {Colors.GREEN}goto_car{Colors.ENDC}                 - Detect and navigate to car
  {Colors.GREEN}goto_truck{Colors.ENDC}               - Detect and navigate to truck

{Colors.BOLD}{Colors.HEADER}================================={Colors.ENDC}
"""
    print(help_text)

async def main():
    """Main function"""
    controller = DroneController()
    
    print(f"{Colors.BOLD}{Colors.HEADER}==== MAVSDK-Python Drone Controller ===={Colors.ENDC}")
    print(f"{Colors.BOLD}Type 'help' to see available commands{Colors.ENDC}")
    
    # Connect to drone by default
    await controller.connect()
    
    while True:
        try:
            # Get user input
            command = input(f"\n{Colors.BOLD}drone> {Colors.ENDC}").strip()
            
            if not command:
                continue
                
            # Split command into parts
            parts = command.split()
            cmd = parts[0].lower()
            
            # Process commands
            if cmd == "help":
                await print_help()
                
            elif cmd == "connect":
                address = parts[1] if len(parts) > 1 else "udp://:14540"
                await controller.connect(address)
                
            elif cmd == "arm":
                await controller.arm()
                
            elif cmd == "disarm":
                await controller.disarm()
                
            elif cmd == "takeoff":
                altitude = float(parts[1]) if len(parts) > 1 else TAKEOFF_ALT
                await controller.takeoff(altitude)
                
            elif cmd == "land":
                await controller.land()
                
            elif cmd == "rtl" or cmd == "return":
                await controller.return_to_launch()
                
            elif cmd == "stop":
                await controller.stop_current_command()
                
            elif cmd == "status":
                await controller.print_status()
                
            elif cmd in ["forward", "backward", "left", "right", "up", "down", "yaw_left", "yaw_right"]:
                if len(parts) < 2:
                    print(f"{Colors.RED}❌ {cmd} command needs a value. Example: {cmd} 5{Colors.ENDC}")
                    continue
                    
                value = float(parts[1])
                await controller.move_direction(cmd, value)
                
            elif cmd == "goto":
                if len(parts) < 3:
                    print(f"{Colors.RED}❌ goto command needs at least latitude and longitude{Colors.ENDC}")
                    continue
                    
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3]) if len(parts) > 3 else None
                await controller.goto_position(lat, lon, alt)
                
            elif cmd == "orbit":
                radius = float(parts[1]) if len(parts) > 1 else DEFAULT_ORBIT_RADIUS
                speed = float(parts[2]) if len(parts) > 2 else 2.0
                clockwise = True if len(parts) <= 3 or parts[3].lower() != "ccw" else False
                await controller.orbit(radius, speed, clockwise)
                
            elif cmd == "goto_person":
                await controller.goto_object("person")
                
            elif cmd == "goto_car":
                await controller.goto_object("car")
                
            elif cmd == "goto_truck":
                await controller.goto_object("truck")
                
            elif cmd == "exit":
                print(f"{Colors.BLUE}👋 Exiting...{Colors.ENDC}")
                
                # Land if in air
                async for in_air in controller.drone.telemetry.in_air():
                    if in_air:
                        print(f"{Colors.YELLOW}⚠️ Drone is still in air. Landing...{Colors.ENDC}")
                        await controller.land()
                    break
                    
                # Disarm if armed
                if controller.armed:
                    await controller.disarm()
                    
                # Cancel telemetry task
                if controller.telemetry_task:
                    controller.telemetry_task.cancel()
                    
                break
                
            else:
                print(f"{Colors.RED}❌ Unknown command: {cmd}. Type 'help' for available commands.{Colors.ENDC}")
                
        except ValueError as e:
            print(f"{Colors.RED}❌ Invalid parameter: {e}{Colors.ENDC}")
        except Exception as e:
            print(f"{Colors.RED}❌ Error: {e}{Colors.ENDC}")
            
    print(f"{Colors.GREEN}✅ Exited safely{Colors.ENDC}")

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(main())
