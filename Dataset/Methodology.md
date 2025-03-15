# UAV Skill Set Documentation

## ✈️ High-Level Skills

These skills define complex UAV maneuvers and automated flight patterns.

### 🚀 **Flight Operations**
- **Takeoff** (abbr: `tk`)
  - **Definition**: `motors_arm(); set_throttle(50); ?baro_read()>=safe_altitude{->True};`
  - **Args**: `[]`
  - **Description**: Launch UAV into flight.

- **Land** (abbr: `ld`)
  - **Definition**: `set_throttle(0); set_attitude(0); motors_disarm(); ->True;`
  - **Args**: `[]`
  - **Description**: Descend and touch down safely.

- **Hover** (abbr: `hv`)
  - **Definition**: `12{?_1=gps_read()!=False&_2=baro_read()!=False{set_throttle(hold);d(100)}};`
  - **Args**: `[]`
  - **Description**: Maintain a fixed position in the air.

### 🗺️ **Navigation & Path Planning**
- **Navigate to Waypoint** (abbr: `wp`)
  - **Definition**: `12{?_1=gps_read($1)!=False{set_attitude(0);mf(10);d(500);}};`
  - **Args**: `[coordinates:str]`
  - **Description**: Navigate to a specified waypoint.

- **Follow Waypoint Route** (abbr: `wr`)
  - **Definition**: `_1=route[0];?_1!=False{wp(_1);wr(route[1:])};`
  - **Args**: `[route:list]`
  - **Description**: Follow a predefined waypoint sequence.

- **Return Home** (abbr: `rh`)
  - **Definition**: `wp(home_coords); ld();`
  - **Args**: `[]`
  - **Description**: Return to home position and land.

- **Path Planning** (abbr: `pp`)
  - **Definition**: `_1=path_planner($1);?_1!=False{wp(_1)};`
  - **Args**: `[destination:str]`
  - **Description**: Compute an optimal flight path and navigate to the destination.

- **Path Replanning** (abbr: `pr`)
  - **Definition**: `?od()==True{_1=path_planner(current_target);?_1!=False{wp(_1)}};`
  - **Args**: `[]`
  - **Description**: Replan the path in real-time if an obstacle is detected.

### 🚧 **Obstacle Avoidance**
- **Obstacle Avoidance** (abbr: `oa`)
  - **Definition**: `?od()==True{mf(-5);tc(45);mf(5);}`
  - **Args**: `[]`
  - **Description**: Avoid unexpected obstacles.

- **Obstacle Detection** (abbr: `od`)
  - **Definition**: `_1=sensor_check();?_1!=False{->True};->False;`
  - **Args**: `[]`
  - **Description**: Detect obstacles in the UAV's path.

---

## ⚙️ Low-Level Skills

These are fundamental movement and sensing commands for UAV operations.

### 🏃 **Basic Movement**
- **Move Forward** (abbr: `mf`): `[distance:int]` → Move forward by a distance.
- **Move Backward** (abbr: `mb`): `[distance:int]` → Move backward by a distance.
- **Move Left** (abbr: `ml`): `[distance:int]` → Move left by a distance.
- **Move Right** (abbr: `mr`): `[distance:int]` → Move right by a distance.
- **Move Up** (abbr: `mu`): `[distance:int]` → Move up by a distance.
- **Move Down** (abbr: `md`): `[distance:int]` → Move down by a distance.

### 🔄 **Rotational Movements**
- **Turn Clockwise** (abbr: `tc`): `[degrees:int]` → Rotate clockwise/right by certain degrees.
- **Turn Counterclockwise** (abbr: `tu`): `[degrees:int]` → Rotate counterclockwise/left by certain degrees.
- **Move in Circle** (abbr: `mi`): `[cw:bool]` → Move in a circular pattern.

### ⏳ **Timing & Control**
- **Delay** (abbr: `d`): `[milliseconds:int]` → Wait for specified milliseconds.
- **Take Picture** (abbr: `tp`): `[]` → Capture an image.
- **Replan Path** (abbr: `rp`): `[]` → Recalculate the flight path.

### 🏠 **Flight Controls**
- **Arm Motors** (abbr: `motors_arm`): `[]` → Arm the motors for takeoff.
- **Disarm Motors** (abbr: `motors_disarm`): `[]` → Disarm the motors after landing.
- **Set Throttle** (abbr: `set_throttle`): `[percent:int]` → Adjust throttle power.
- **Set Attitude** (abbr: `set_attitude`): `[degrees:int]` → Modify UAV tilt angle.

### 🔍 **Sensing & Object Detection**
- **Read Barometric Altitude** (abbr: `baro_read`): `[]` → Get altitude data.
- **Read GPS Coordinates** (abbr: `gps_read`): `[coordinates:str]` → Fetch GPS location.
- **Check Visibility** (abbr: `iv`): `[object_name:str]` → Detect if an object is visible.
- **Get Object X-Coordinate** (abbr: `ox`): `[object_name:str]` → Retrieve X-position.
- **Get Object Y-Coordinate** (abbr: `oy`): `[object_name:str]` → Retrieve Y-position.
- **Get Object Width** (abbr: `ow`): `[object_name:str]` → Measure object's width.
- **Get Object Height** (abbr: `oh`): `[object_name:str]` → Measure object's height.
- **Get Object Distance** (abbr: `od`): `[object_name:str]` → Compute object's distance in cm.

### 🤖 **AI & Logging**
- **Probe LLM** (abbr: `p`): `[question:str]` → Query the AI model for reasoning.
- **Log Information** (abbr: `l`): `[text:str]` → Output logs to the console.

---

### 🏆 **Summary**
This document categorizes and details UAV skills into **high-level flight operations** and **low-level fundamental commands**. These commands enable **autonomous UAV navigation, obstacle avoidance, and precise control**, supporting a wide range of aerial robotics applications.
