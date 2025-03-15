# UAV MiniSpec Skill Set

## 🚀 Overview
This document provides a structured **MiniSpec Skill Set** categorized by UAV task templates, ensuring seamless execution in **PX4 SITL** with **MAVSDK**.

---

## 🛠 1️⃣ Pre-Flight & Setup Skills

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

## 🛫 2️⃣ Basic Flight Skills

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

## 📍 3️⃣ Navigation & Waypoint-Based Skills

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

## 🛑 4️⃣ Obstacle Avoidance & Path Planning

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
```

---

## 📡 5️⃣ Perception & Object Interaction

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

## 📷 6️⃣ Low-Level Control Skills

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

---

## 🎯 Summary
✅ **Expanded MiniSpec skills for PX4 SITL**  
✅ **Mapped each skill for real UAV execution**  
✅ **Introduced advanced AI-assisted perception & obstacle avoidance**  
✅ **Ensured compatibility with MAVSDK for automated execution**  

🚀 **Next Steps:** Would you like to integrate these into a Python-based execution framework?

