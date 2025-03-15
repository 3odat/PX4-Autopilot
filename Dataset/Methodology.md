# UAV MiniSpec Skill Set

## 🚀 Overview
This document provides a structured **MiniSpec Skill Set** categorized by UAV task templates, ensuring seamless execution in **PX4 SITL** with **MAVSDK**.

---

## 🛠 Pre-Flight & Setup Skills

```yaml
- abbr: motors_arm
  name: arm_motors
  definition: motors_arm();
  mavsdk: await drone.action.arm()
  description: Arm the motors before flight.

- abbr: motors_disarm
  name: disarm_motors
  definition: motors_disarm();
  mavsdk: await drone.action.disarm()
  description: Disarm the motors after landing.

- abbr: set_throttle
  name: set_throttle
  definition: set_throttle(percent);
  mavsdk: await drone.manual_control.set_throttle(percent/100)
  description: Set throttle power percentage.

- abbr: set_attitude
  name: set_attitude
  definition: set_attitude(degrees);
  mavsdk: await drone.manual_control.set_attitude(degrees)
  description: Set UAV tilt angle (attitude).
```

---

## 🛫 Basic Flight Skills

```yaml
- abbr: tk
  name: takeoff
  definition: motors_arm();set_throttle(50);?baro_read()>=safe_altitude{->True};
  mavsdk: await drone.action.arm(); await drone.action.takeoff()
  description: Launch UAV into flight.

- abbr: ld
  name: land
  definition: set_throttle(0);set_attitude(0);motors_disarm();->True;
  mavsdk: await drone.action.land()
  description: Descend and touch down safely.

- abbr: hv
  name: hover
  definition: 12{?_1=gps_read()!=False&_2=baro_read()!=False{set_throttle(hold);d(100)}};
  mavsdk: await asyncio.sleep(3)
  description: Maintain a fixed position in the air.
```

---

## 📍 Navigation & Waypoint-Based Skills

```yaml
- abbr: wp
  name: navigate_to_waypoint
  definition: 12{?_1=gps_read($1)!=False{set_attitude(0);mf(10);d(500);}};
  mavsdk: await drone.action.goto_location(x, y, z, 0)
  description: Navigate to a specified waypoint.

- abbr: wr
  name: follow_waypoint_route
  definition: _1=route[0];?_1!=False{wp(_1);wr(route[1:])};
  mavsdk: for wp in waypoints: await drone.action.goto_location(wp.x, wp.y, wp.z, 0)
  description: Follow a predefined waypoint sequence.

- abbr: rh
  name: return_home
  definition: wp(home_coords);ld();
  mavsdk: await drone.action.return_to_launch()
  description: Return to home position and land.
```

---

## 🛑 Obstacle Avoidance & Path Planning

```yaml
- abbr: pp
  name: path_plan
  definition: _1=path_planner($1);?_1!=False{wp(_1)};
  mavsdk: await plan_safe_route(destination)
  description: Compute an optimal flight path.

- abbr: pr
  name: path_replan
  definition: ?od()==True{_1=path_planner(current_target);?_1!=False{wp(_1)}};
  mavsdk: await replan_route(drone)
  description: Replan the path if an obstacle is detected.

- abbr: oa
  name: obstacle_avoidance
  definition: ?od()==True{mf(-5);tc(45);mf(5);}
  mavsdk: await avoid_obstacle(drone)
  description: Avoid unexpected obstacles.

- abbr: od
  name: obstacle_detect
  definition: _1=sensor_check();?_1!=False{->True};->False;
  mavsdk: await check_obstacles(drone);
  description: Detects obstacles in UAV’s path.

```

---

## 📡 5️⃣ Perception & Vision-Based

```yaml

- abbr: iv
  name: is_visible
  definition: iv(object_name);
  mavsdk: await scan_for_object(drone, object_name);
  description: Checks if an object is visible.

- abbr: ox
  name: object_x
  definition: object_x(object_name);
  mavsdk: await get_object_x(drone, object_name);
  description: Gets X coordinate of object in camera frame.

- abbr: oy
  name: object_y
  definition: object_y(object_name);
  mavsdk: await get_object_y(drone, object_name);
  description: Gets Y coordinate of object.

- abbr: od
  name: object_distance
  definition: object_dis(object_name);
  mavsdk: await get_object_distance(drone, object_name);
  description: Measures object distance.
```

## 📡 🌐 AI & Camera Skills
```yaml
- abbr: tp
  name: take_picture
  definition: tp();
  mavsdk: await drone.camera.take_photo()
  description: Capture an image.

- abbr: p
  name: probe
  definition: p(question);
  mavsdk: await query_ai_for_answer(drone, question)
  description: Ask AI for reasoning.
```




---

## 📷 Low-Level Control Skills

```yaml
- abbr: mf
  name: move_forward
  definition: mf(distance);
  mavsdk: await drone.action.set_manual_control_input(1.0, 0.0, 0.0, 0.0)
  description: Move forward by a distance.

- abbr: mb
  name: move_backward
  definition: mb(distance);
  mavsdk: await drone.action.set_manual_control_input(-1.0, 0.0, 0.0, 0.0)
  description: Move backward.

- abbr: ml
  name: move_left
  definition: ml(distance);
  mavsdk: await drone.action.set_manual_control_input(0.0, -1.0, 0.0, 0.0)
  description: Move left.

- abbr: mr
  name: move_right
  definition: mr(distance);
  mavsdk: await drone.action.set_manual_control_input(0.0, 1.0, 0.0, 0.0)
  description: Move right.
```
# 🚀 UAV MiniSpec Skills for PX4 & Tello (MAVSDK Execution)

## ✨ Why This Skill Set?
This MiniSpec skill set is **designed for UAV automation** using **MAVSDK for PX4 drones** and **vision-based perception for Tello drones**.  
It includes **mission-critical flight operations, object detection, navigation, and AI-assisted decision-making**.

## ✅ What This Covers
- **🛫 Navigation & Path Planning** (Waypoint flying, route following, dynamic replanning)
- **📡 Perception & Object Interaction** (Vision-based tracking, AI-assisted object detection)
- **🛑 Obstacle Avoidance & Safety** (Obstacle detection, emergency landing, geofencing)
- **📡 Communication & AI** (LLM-assisted decisions, real-time telemetry, logging)
- **⚙️ Full Control Over UAV Motion** (Precise movement using low-level controls)

---

## 🔥 MiniSpec Skill Categories & Mappings
### **🛠 Pre-Flight & Setup**
| **Skill** | **MiniSpec** | **MAVSDK Equivalent** | **Purpose** |
|-----------|------------|----------------------|------------|
| `motors_arm` | `motors_arm();` | `await drone.action.arm();` | Arms UAV motors before takeoff. |
| `baro_read` | `baro_read();` | `async for altitude in drone.telemetry.altitude(): return altitude.relative_altitude_m;` | Reads barometric altitude. |
| `gps_read` | `gps_read();` | `async for position in drone.telemetry.position(): return position.latitude, position.longitude;` | Reads GPS coordinates. |
| `sensor_check` | `sensor_check();` | `await validate_sensors(drone);` | Checks for hardware and sensor integrity. |

### **🛫 Flight Control**
| **Skill** | **MiniSpec** | **MAVSDK Equivalent** | **Purpose** |
|-----------|------------|----------------------|------------|
| `tk` | `takeoff();` | `await drone.action.arm(); await drone.action.takeoff();` | UAV launches into the air. |
| `ld` | `land();` | `await drone.action.land();` | Commands UAV to descend and land. |
| `hv` | `hover();` | `await asyncio.sleep(3);` | Keeps UAV hovering in a fixed position. |

### **📍 Navigation & Path Planning**
| **Skill** | **MiniSpec** | **MAVSDK Equivalent** | **Purpose** |
|-----------|------------|----------------------|------------|
| `wp(x, y, z)` | `goto_waypoint(x, y, z);` | `await drone.action.goto_location(x, y, z, 0);` | Fly to a specified GPS waypoint. |
| `wr(route)` | `follow_route(route);` | `for wp in waypoints: await drone.action.goto_location(wp.x, wp.y, wp.z, 0);` | Follow a set of waypoints. |
| `rh` | `return_home();` | `await drone.action.return_to_launch();` | Returns to takeoff location. |
| `pp(destination)` | `plan_path(destination);` | `await plan_safe_route(destination);` | Generates an optimized route to a destination. |
| `pr` | `path_replan();` | `await replan_route(drone);` | Replans route if obstacles are detected. |

### **🛑 Obstacle Avoidance & Safety**
| **Skill** | **MiniSpec** | **MAVSDK Equivalent** | **Purpose** |
|-----------|------------|----------------------|------------|
| `oa` | `obstacle_avoidance();` | `await avoid_obstacle(drone);` | Avoids obstacles dynamically. |
| `od` | `obstacle_detect();` | `await check_obstacles(drone);` | Detects obstacles in UAV’s path. |
| `el` | `emergency_land();` | `await drone.action.land();` | Triggers emergency landing. |

### **📡 Perception & Vision-Based Skills**
| **Skill** | **MiniSpec** | **MAVSDK Equivalent** | **Purpose** |
|-----------|------------|----------------------|------------|
| `iv(object_name)` | `is_visible(object_name);` | `await scan_for_object(drone, object_name);` | Checks if an object is visible. |
| `ox(object_name)` | `object_x(object_name);` | `await get_object_x(drone, object_name);` | Gets X coordinate of object in camera. |
| `oy(object_name)` | `object_y(object_name);` | `await get_object_y(drone, object_name);` | Gets Y coordinate of object. |
| `od(object_name)` | `object_dis(object_name);` | `await get_object_distance(drone, object_name);` | Measures object distance. |

### **📷 Camera & AI-Assisted Decision-Making**
| **Skill** | **MiniSpec** | **MAVSDK Equivalent** | **Purpose** |
|-----------|------------|----------------------|------------|
| `tp` | `take_picture();` | `await drone.camera.take_photo();` | Captures an image. |
| `p(question)` | `probe(question);` | `await query_ai_for_answer(drone, question);` | Uses LLM for AI-based reasoning. |

---

## 🚀 **Why Commit to This?**
✅ Covers **both GPS (PX4) and Vision-based (Tello) UAVs**  
✅ Includes **autonomous navigation, perception, AI-driven decision-making**  
✅ Supports **obstacle avoidance, emergency handling, and telemetry**  
✅ Future-proof **for SLAM, multi-UAV, and deep learning integration**  

---

## **Next Steps**
Would you like me to:  
1️⃣ **Automate dataset generation with these skills?**  
2️⃣ **Develop a Python framework to parse and execute MiniSpec?**  

Let me know how you'd like to proceed! 🚀📊

---

## 🎯 Summary
✅ **Expanded MiniSpec skills for PX4 SITL**  
✅ **Mapped each skill for real UAV execution**  
✅ **Introduced advanced AI-assisted perception & obstacle avoidance**  
✅ **Ensured compatibility with MAVSDK for automated execution**  

🚀 **Next Steps:** Would you like to integrate these into a Python-based execution framework?

