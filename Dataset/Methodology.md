# UAV Skill Set Documentation

# Quick Example

  ```
import asyncio
from mavsdk import System

class MiniSpecExecutor:
    def __init__(self):
        self.drone = System()

    async def connect(self):
        """Connects to PX4 Gazebo only once."""
        await self.drone.connect(system_address="udp://:14540")
        print("✅ Connected to PX4 Gazebo")

    async def execute(self, command):
        """Parses and executes MiniSpec commands sequentially."""
        commands = command.split(";")
        for cmd in commands:
            cmd = cmd.strip()

            if cmd == "tk()":
                print("🚀 Executing: Takeoff")
                await self.drone.action.arm()
                await self.drone.action.takeoff()
                await asyncio.sleep(3)

            elif cmd == "ld()":
                print("🛬 Executing: Landing")
                await self.drone.action.land()
                await asyncio.sleep(5)
                print("🔻 Disarming motors")
                await self.drone.action.disarm()

async def main():
    executor = MiniSpecExecutor()
    await executor.connect()

    # Example MiniSpec Task: Takeoff → Hover → Land
    minispec_task = "tk(); ld();"
    await executor.execute(minispec_task)

asyncio.run(main())

  ```
## 🚀 High-Level Skills

### ✈️ Flight Operations


##### Take Off
  ```
- **abbr:** `tk`
  - **name:** `takeoff`
  - **definition:** `motors_arm();set_throttle(50);?baro_read()>=safe_altitude{->True};`
  - **args:** `[]`
  - **description:** Launch UAV into flight.
  ```
##### Take Off - MAVSDK
  ```
  await self.drone.action.arm()
  await self.drone.action.takeoff()
  await asyncio.sleep(3)

  ```
##### Land

  ```
- **abbr:** `ld`
  - **name:** `land`
  - **definition:** `set_throttle(0);set_attitude(0);motors_disarm();->True;`
  - **args:** `[]`
  - **description:** Descend and touch down safely.
  ```
##### Landing - MAVSDK
  ```
  await self.drone.action.land()
  await asyncio.sleep(5)
  await self.drone.action.disarm()
  ```

##### Hover
  ```
- **abbr:** `hv`
  - **name:** `hover`
  - **definition:** `12{?_1=gps_read()!=False&_2=baro_read()!=False{set_throttle(hold);d(100)}};`
  - **args:** `[]`
  - **description:** Maintain a fixed position in the air.
  ```
##### Hover - MAVSDK
  ```
await self.drone.action.hold()
await asyncio.sleep(10)  # Hover for 10 seconds
  ```

### 📍 Navigation & Path Planning

##### Navigate to Specific Waypoint
  ```
- **abbr:** `wp`
  - **name:** `navigate_to_waypoint`
  - **definition:** `12{?_1=gps_read($1)!=False{set_attitude(0);mf(10);d(500);}};`
  - **args:** `[coordinates:str]`
  - **description:** Navigate to a specified waypoint.
  ```
  ```
await self.drone.action.goto_location(latitude, longitude, altitude, yaw)
  ```


##### Follow to Specific Waypoint
  ```
- **abbr:** `wr`
  - **name:** `follow_waypoint_route`
  - **definition:** `_1=route[0];?_1!=False{wp(_1);wr(route[1:])};`
  - **args:** `[route:list]`
  - **description:** Follow a predefined waypoint sequence.
  ```
  ```
for waypoint in route:
    await self.drone.action.goto_location(waypoint[0], waypoint[1], altitude, yaw)
  ```
##### Return to Home Posision
  ```
- **abbr:** `rh`
  - **name:** `return_home`
  - **definition:** `wp(home_coords);ld();`
  - **args:** `[]`
  - **description:** Return to home position and land.
  ```
  ```
await self.drone.action.return_to_launch()
await asyncio.sleep(5)  # Wait for descent
await self.drone.action.disarm()
  ```


##### Path Planning
  ```
- **abbr:** `pp`
  - **name:** `path_plan`
  - **definition:** `_1=path_planner($1);?_1!=False{wp(_1)};`
  - **args:** `[destination:str]`
  - **description:** Compute an optimal flight path and navigate to the destination.
  ```
##### Path Re-Planning
  ```
- **abbr:** `pr`
  - **name:** `path_replan`
  - **definition:** `?od()==True{_1=path_planner(current_target);?_1!=False{wp(_1)}};`
  - **args:** `[]`
  - **description:** Replan the path in real-time if an obstacle is detected.
  ```

### 🛑 Obstacle Handling
##### Avoidance Objects
  ```
- **abbr:** `oa`
  - **name:** `obstacle_avoidance`
  - **definition:** `?od()==True{mf(-5);tc(45);mf(5);}`
  - **args:** `[]`
  - **description:** Avoid unexpected obstacles.
  ```
##### Detect Obstacle
  ```
- **abbr:** `od`
  - **name:** `obstacle_detect`
  - **definition:** `_1=sensor_check();?_1!=False{->True};->False;`
  - **args:** `[]`
  - **description:** Detect obstacles in the UAV's path.
  ```

---

## 🔧 Low-Level Skills

### 📏 Movement Controls
##### Moving Forward
  ```
- **abbr:** `mf`
  - **name:** `move_forward`
  - **args:** `[distance:int]`
  - **description:** Move forward by a distance.
  ```
  ```
await self.drone.manual_control.set_manual_control_input(1.0, 0.0, 0.0, 0.0)  # Move forward
await asyncio.sleep(time_based_on_distance)  # Adjust time based on distance
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop movement
  ```

##### Moving Backword
  ```
- **abbr:** `mb`
  - **name:** `move_backward`
  - **args:** `[distance:int]`
  - **description:** Move backward by a distance.
  ```
 ```
await self.drone.manual_control.set_manual_control_input(-1.0, 0.0, 0.0, 0.0)  # Move backward
await asyncio.sleep(time_based_on_distance)  # Adjust time based on distance
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop movement
 ```

##### Moving Left
  ```
- **abbr:** `ml`
  - **name:** `move_left`
  - **args:** `[distance:int]`
  - **description:** Move left by a distance.
  ```
  ```
await self.drone.manual_control.set_manual_control_input(0.0, -1.0, 0.0, 0.0)  # Move left
await asyncio.sleep(time_based_on_distance)  # Adjust time based on distance
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop movement
  ```


##### Moving Right
  ```
- **abbr:** `mr`
  - **name:** `move_right`
  - **args:** `[distance:int]`
  - **description:** Move right by a distance.
  ```
```
await self.drone.manual_control.set_manual_control_input(0.0, 1.0, 0.0, 0.0)  # Move right
await asyncio.sleep(time_based_on_distance)  # Adjust time based on distance
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop movement
```


##### Moving Up
  ```
- **abbr:** `mu`
  - **name:** `move_up`
  - **args:** `[distance:int]`
  - **description:** Move up by a distance.
  ```
  ```
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 1.0, 0.0)  # Move up
await asyncio.sleep(time_based_on_distance)  # Adjust time based on distance
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop movement
  ```

##### Moving Down
  ```
- **abbr:** `md`
  - **name:** `move_down`
  - **args:** `[distance:int]`
  - **description:** Move down by a distance.
  ```
  ```
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, -1.0, 0.0)  # Move down
await asyncio.sleep(time_based_on_distance)  # Adjust time based on distance
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop movement
  ```


### 🔄 Rotation & Positioning
##### Turn Clockwise
  ```
- **abbr:** `tc`
  - **name:** `turn_cw`
  - **args:** `[degrees:int]`
  - **description:** Rotate clockwise/right by certain degrees.
  ```


##### Turn Counter Clockwise
  ```
- **abbr:** `tu`
  - **name:** `turn_ccw`
  - **args:** `[degrees:int]`
  - **description:** Rotate counterclockwise/left by certain degrees.
  ```
  ```
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 1.0)  # Rotate clockwise
await asyncio.sleep(time_based_on_degrees)  # Adjust time based on degrees
await self.drone.manual_control.set_manual_control_input(0.0, 0.0, 0.0, 0.0)  # Stop rotation
  ```

##### Moving in Circle
  ```
- **abbr:** `mi`
  - **name:** `move_in_circle`
  - **args:** `[cw:bool]`
  - **description:** Move in a circular path (clockwise or counterclockwise).
  ```

### 🕹️ Sensors & Interaction
##### Check Visibility of Object
  ```
- **abbr:** `iv`
  - **name:** `is_visible`
  - **args:** `[object_name:str]`
  - **description:** Check the visibility of a target object.
  ```
##### Getting X-Coordinate of Object
  ```
- **abbr:** `ox`
  - **name:** `object_x`
  - **args:** `[object_name:str]`
  - **description:** Get object's X-coordinate in (0,1).
  ```
##### Getting Y-Coordinate of Object
  ```
- **abbr:** `oy`
  - **name:** `object_y`
  - **args:** `[object_name:str]`
  - **description:** Get object's Y-coordinate in (0,1).
  ```
##### Getting Width of Object
  ```
- **abbr:** `ow`
  - **name:** `object_width`
  - **args:** `[object_name:str]`
  - **description:** Get object's width in (0,1).
  ```
##### Getting Width of Height
  ```
- **abbr:** `oh`
  - **name:** `object_height`
  - **args:** `[object_name:str]`
  - **description:** Get object's height in (0,1).
  ```
##### Getting Distance
  ```
- **abbr:** `od`
  - **name:** `object_dis`
  - **args:** `[object_name:str]`
  - **description:** Get object's distance in cm.
  ```

### 📡 System & Control
##### Arm the drone
  ```
- **abbr:** `motors_arm`
  - **name:** `motors_arm`
  - **args:** `[]`
  - **description:** Arm the motors for takeoff.
  ```
##### Disarm the drone
  ```
- **abbr:** `motors_disarm`
  - **name:** `motors_disarm`
  - **args:** `[]`
  - **description:** Disarm the motors after landing.
  ```
##### Set throttle power percentage
  ```
- **abbr:** `set_throttle`
  - **name:** `set_throttle`
  - **args:** `[percent:int]`
  - **description:** Set the throttle power percentage.
  ```
##### Set the UAV attitud
  ```
- **abbr:** `set_attitude`
  - **name:** `set_attitude`
  - **args:** `[degrees:int]`
  - **description:** Set the UAV attitude (tilt angle).
  ```
##### Read the barometric altitude
  ```
- **abbr:** `baro_read`
  - **name:** `baro_read`
  - **args:** `[]`
  - **description:** Read the barometric altitude.
  ```
##### Read the GPS coordinates
  ```
- **abbr:** `gps_read`
  - **name:** `gps_read`
  - **args:** `[coordinates:str]`
  - **description:** Read the GPS coordinates.
  ```
---

## 📝 Summary
This document provides a structured list of high-level and low-level UAV skills, enabling efficient navigation, control, and decision-making for autonomous flight operations.
