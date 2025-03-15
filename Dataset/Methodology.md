# 🚀 **Drone Skill Sets**

## ✨ **High-Level Skills**

These skills enable the drone to perform complex and intelligent behaviors.

### 🛫 **Flight Operations**
- **Takeoff** (`tk`)
  - 📜 *Definition:* `motors_arm(); set_throttle(50); ?baro_read() >= safe_altitude {-> True};`
  - 📝 *Description:* Launch UAV into flight.
  
- **Land** (`ld`)
  - 📜 *Definition:* `set_throttle(0); set_attitude(0); motors_disarm(); -> True;`
  - 📝 *Description:* Descend and touch down safely.

- **Hover** (`hv`)
  - 📜 *Definition:* `12{?_1=gps_read()!=False&_2=baro_read()!=False{set_throttle(hold);d(100)}};`
  - 📝 *Description:* Maintain a fixed position in the air.

### 📍 **Navigation & Path Planning**
- **Navigate to Waypoint** (`wp`)
  - 📜 *Definition:* `12{?_1=gps_read($1)!=False{set_attitude(0); mf(10); d(500);}};`
  - 🎯 *Args:* `[coordinates:str]`
  - 📝 *Description:* Navigate to a specified waypoint.

- **Follow Waypoint Route** (`wr`)
  - 📜 *Definition:* `_1=route[0]; ?_1!=False{wp(_1); wr(route[1:])};`
  - 🎯 *Args:* `[route:list]`
  - 📝 *Description:* Follow a predefined waypoint sequence.

- **Return Home** (`rh`)
  - 📜 *Definition:* `wp(home_coords); ld();`
  - 📝 *Description:* Return to home position and land.

- **Path Planning** (`pp`)
  - 📜 *Definition:* `_1=path_planner($1); ?_1!=False{wp(_1)};`
  - 🎯 *Args:* `[destination:str]`
  - 📝 *Description:* Compute an optimal flight path and navigate to the destination.

- **Path Replanning** (`pr`)
  - 📜 *Definition:* `?od()==True{_1=path_planner(current_target); ?_1!=False{wp(_1)}};`
  - 📝 *Description:* Replan the path in real-time if an obstacle is detected.

### 🛑 **Obstacle Handling**
- **Obstacle Avoidance** (`oa`)
  - 📜 *Definition:* `?od()==True{mf(-5); tc(45); mf(5);}`
  - 📝 *Description:* Avoid unexpected obstacles.

- **Obstacle Detection** (`od`)
  - 📜 *Definition:* `_1=sensor_check(); ?_1!=False{-> True}; -> False;`
  - 📝 *Description:* Detect obstacles in the UAV's path.

---

## 🔧 **Low-Level Skills**

These fundamental actions allow precise control over the drone's movement and operations.

### 🚁 **Basic Movements**
- **Move Forward** (`mf`): Move forward by a distance.
- **Move Backward** (`mb`): Move backward by a distance.
- **Move Left** (`ml`): Move left by a distance.
- **Move Right** (`mr`): Move right by a distance.
- **Move Up** (`mu`): Move up by a distance.
- **Move Down** (`md`): Move down by a distance.
- **Move in Circle** (`mi`): Move in circle in CW/CCW.

### 🔄 **Rotations & Delays**
- **Turn Clockwise** (`tc`): Rotate clockwise/right by certain degrees.
- **Turn Counterclockwise** (`tu`): Rotate counterclockwise/left by certain degrees.
- **Delay** (`d`): Wait for a specified duration.

### 🎯 **Object Detection & AI Functions**
- **Is Visible** (`iv`): Check the visibility of a target object.
- **Object X** (`ox`): Get object's X-coordinate in (0,1).
- **Object Y** (`oy`): Get object's Y-coordinate in (0,1).
- **Object Width** (`ow`): Get object's width in (0,1).
- **Object Height** (`oh`): Get object's height in (0,1).
- **Object Distance** (`od`): Get object's distance in cm.
- **Probe AI Model** (`p`): Probe the LLM for reasoning.
- **Log Output** (`l`): Output text to console.

### 📸 **Camera & Sensors**
- **Take Picture** (`tp`): Capture an image.
- **Re-plan Path** (`rp`): Trigger replanning.
- **Arm Motors** (`motors_arm`): Enable the motors for flight.
- **Disarm Motors** (`motors_disarm`): Disable the motors.
- **Set Throttle** (`set_throttle`): Control motor power.
- **Set Attitude** (`set_attitude`): Adjust UAV tilt.
- **Read Barometric Altitude** (`baro_read`): Get altitude data.
- **Read GPS Coordinates** (`gps_read`): Fetch GPS location.
- **Path Planner** (`path_planner`): Compute an optimal path.
- **Sensor Check** (`sensor_check`): Detect obstacles using onboard sensors.

---

✨ *This markdown provides a structured overview of drone skills, categorized into high-level and low-level functionalities for better readability and usability.* 🚀
