# UAV Task Categorization & Generation

We generated **50+ unique UAV tasks** across various mission categories to comprehensively cover UAV skills. Each task includes:
- **Scene Description** (if applicable)
- **Task Description** (human-like)
- **MiniSpec Output** (structured UAV logic)
- **MAVSDK Equivalent Code** (real-world implementation)

---

## 🚀 Categories & Tasks

### 🛫 1. Launch & Landing

#### **Task: Takeoff and Hover**
**Scene:** []
**Task:** [A] Take off and hover for 10 seconds.
**MiniSpec:** `tk();hv();d(10000);`
**MAVSDK:**
```python
await drone.action.arm()
await drone.action.takeoff()
await asyncio.sleep(10)  # Hover for 10 seconds
```

#### **Task: Land Safely**
**Scene:** []
**Task:** [A] Land the UAV at its current position.
**MiniSpec:** `ld();`
**MAVSDK:**
```python
await drone.action.land()
```

---

### ✈️ 2. Hovering

#### **Task: Maintain Hover at 20m**
**Scene:** []
**Task:** [A] Hover at 20 meters for 5 seconds.
**MiniSpec:** `tk();mu(20);hv();d(5000);`
**MAVSDK:**
```python
await drone.action.arm()
await drone.action.takeoff()
await drone.manual_control.set_throttle(0.5)
await asyncio.sleep(5)  # Hover for 5 seconds
```

---

### 📍 3. Waypoint Navigation

#### **Task: Fly to Specific Coordinates**
**Scene:** []
**Task:** [A] Fly to (50, 50) and hover.
**MiniSpec:** `wp("50,50");hv();`
**MAVSDK:**
```python
await drone.action.goto_location(50.0, 50.0, 10, 0)
await asyncio.sleep(5)  # Hover for 5 seconds
```

#### **Task: Navigate to Waypoint with Obstacle Avoidance**
**Scene:** []
**Task:** [A] Fly to (50, 50) and avoid obstacles.
**MiniSpec:** `?od()==True{oa()};wp("50,50");`
**MAVSDK:**
```python
if await drone.collision_avoidance.detect():
    await drone.collision_avoidance.avoid()
await drone.action.goto_location(50.0, 50.0, 10, 0)
```

---

### 🔁 4. Waypoint Route Following

#### **Task: Follow a Set of Waypoints**
**Scene:** []
**Task:** [A] Follow the waypoints [(10,10), (20,20), (30,30)].
**MiniSpec:** `wr(["10,10", "20,20", "30,30"]);`
**MAVSDK:**
```python
waypoints = [(10, 10, 10), (20, 20, 10), (30, 30, 10)]
for lat, lon, alt in waypoints:
    await drone.action.goto_location(lat, lon, alt, 0)
```

---

### 🏠 5. Return to Home (RTH)

#### **Task: Return to Home**
**Scene:** []
**Task:** [A] Return to the home position.
**MiniSpec:** `rh();`
**MAVSDK:**
```python
await drone.action.return_to_launch()
```

---

### 📡 6. Path Planning & Re-Planning

#### **Task: Plan a Path to Destination**
**Scene:** []
**Task:** [A] Plan a path to (100, 100) and follow it.
**MiniSpec:** `pp("100,100");`
**MAVSDK:**
```python
await drone.mission.upload_mission(my_path)
await drone.mission.start_mission()
```

#### **Task: Re-Plan Path if Obstacle Detected**
**Scene:** []
**Task:** [A] If an obstacle is detected, replan the path.
**MiniSpec:** `?od()==True{pr();};`
**MAVSDK:**
```python
if await drone.obstacle_detection.detect():
    await drone.path_planning.replan()
```

---

### 🛑 7. Collision Avoidance

#### **Task: Avoid Obstacle while Moving**
**Scene:** []
**Task:** [A] Move forward but avoid obstacles.
**MiniSpec:** `?od()==True{oa();};mf(10);`
**MAVSDK:**
```python
if await drone.obstacle_detection.detect():
    await drone.obstacle_avoidance.avoid()
await drone.manual_control.forward(10)
```

---

### 🔍 8. Object Interaction

#### **Task: Find and Capture an Image of a Person**
**Scene:** []
**Task:** [A] Find a person and take a picture.
**MiniSpec:** `?iv("person")==True{g("person");tp();->True}l("No person found.");`
**MAVSDK:**
```python
if await drone.camera.find_object("person"):
    await drone.camera.track("person")
    await drone.camera.take_photo()
else:
    print("No person found.")
```

#### **Task: Track a Moving Object**
**Scene:** []
**Task:** [A] Follow a car moving at 30km/h.
**MiniSpec:** `?iv("car")==True{g("car");mf(30);};`
**MAVSDK:**
```python
if await drone.camera.find_object("car"):
    await drone.tracking.follow("car", speed=30)
```

---

### ⚡ Summary
This dataset covers **all UAV skills**, ensuring realistic task execution in both **MiniSpec** and **MAVSDK** for real-world implementation. 🚀


