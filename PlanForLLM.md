# Drone Mission DSL Documentation

## Overview
This document outlines a comprehensive grammar for a Drone Mission Domain-Specific Language (DSL). It provides structured commands, sensor checks, telemetry conditions, and clearly defined tasks for robust drone mission planning and execution.

---

## DSL Grammar (EBNF)

```ebnf
mission_script ::= "MISSION" mission_type "BEGIN" { task } "END"

mission_type ::= "WAYPOINT_NAV" | "INDOOR_NAV" | "OBSTACLE_AVOID"
              | "PRECISION_HOVER" | "SAFE_LANDING" | "TERRAIN_MAPPING"
              | "EMERGENCY_RTL" | "HEALTH_MONITOR" | "OBJECT_FOLLOW"
              | "ALTITUDE_HOLD"

task ::= command | sensor_check | condition_block | loop_block

command ::= "TAKEOFF" ["ALT" number] ["TIMEOUT" number]
          | "LAND" ["TIMEOUT" number]
          | "HOVER" ["DUR" number]
          | "GOTO" ("COORD" number number number | "OBJ" identifier) ["TIMEOUT" number]
          | "FOLLOW" "OBJ" identifier ["DIST" number] ["TIMEOUT" number]
          | "RETURN_HOME" ["TIMEOUT" number]
          | "WAIT" number
          | "ALIGN_TO_OBJ" identifier
          | "MAINTAIN_DISTANCE" "OBJ" identifier "DIST" number
          | "SCAN_AREA"
          | "ABORT_MISSION" ["MSG" string]

sensor_check ::= "CHECK_SENSOR" sensor_type ["TIMEOUT" number]

condition_block ::= "IF" condition "THEN" { task } ["ELSE" { task }] "ENDIF"

loop_block ::= "WHILE" condition "DO" { task } "ENDWHILE"

condition ::= telemetry_condition | sensor_condition | object_condition

telemetry_condition ::= "BATTERY" operator number
                      | "GPS_SIGNAL" "==" ("OK" | "FAIL")
                      | "ALTITUDE" operator number
                      | "VELOCITY" operator number
                      | "DISTANCE_TO_OBJ" identifier operator number
                      | "CONFIDENCE_OBJ" identifier operator number
                      | "QUALITY_ODOMETRY" operator number
                      | "ORIENTATION_ERROR" operator number

sensor_condition ::= "SENSOR_OK" sensor_type | "SENSOR_FAIL" sensor_type

object_condition ::= "FOUND_OBJ" identifier | "NOT_FOUND_OBJ" identifier

sensor_type ::= "GPS" | "IMU" | "ODOMETRY" | "DEPTH_CAM" | "LIDAR" 
              | "VISION" | "BAROMETER"

operator ::= ">" | "<" | ">=" | "<=" | "==" | "!="

number ::= numeric literal

identifier ::= alphanumeric object identifiers (e.g., car, person, landmark)

string ::= quoted string literal
```

---

## Mission Examples

### Waypoint Navigation
**Sensors:** GPS, IMU, Battery  
**Telemetry:** GPS signal, battery status
```dsl
MISSION WAYPOINT_NAV BEGIN
    CHECK_SENSOR GPS TIMEOUT 5
    CHECK_SENSOR IMU TIMEOUT 5
    IF SENSOR_OK GPS AND SENSOR_OK IMU AND GPS_SIGNAL == OK AND BATTERY >= 30 THEN
        TAKEOFF ALT 5
        GOTO COORD 10 20 5 TIMEOUT 20
        HOVER DUR 10
        RETURN_HOME TIMEOUT 30
        LAND TIMEOUT 10
    ELSE
        ABORT_MISSION MSG "Preflight check failed"
    ENDIF
END
```

### Indoor Navigation
**Sensors:** Odometry, Depth Camera, IMU  
**Telemetry:** Odometry quality
```dsl
MISSION INDOOR_NAV BEGIN
    CHECK_SENSOR ODOMETRY TIMEOUT 5
    CHECK_SENSOR DEPTH_CAM TIMEOUT 5
    CHECK_SENSOR IMU TIMEOUT 5
    IF SENSOR_OK ODOMETRY AND QUALITY_ODOMETRY >= 0.9 THEN
        TAKEOFF ALT 2
        SCAN_AREA
        GOTO COORD 5 5 2 TIMEOUT 15
        LAND TIMEOUT 10
    ELSE
        ABORT_MISSION MSG "Indoor sensor validation failed"
    ENDIF
END
```

### Object Following (Vision-based)
**Sensors:** Vision (YOLO), Odometry, Depth Camera  
**Telemetry:** Object detection confidence, battery
```dsl
MISSION OBJECT_FOLLOW BEGIN
    CHECK_SENSOR VISION TIMEOUT 5
    CHECK_SENSOR DEPTH_CAM TIMEOUT 5
    CHECK_SENSOR ODOMETRY TIMEOUT 5
    IF SENSOR_OK VISION AND SENSOR_OK DEPTH_CAM AND SENSOR_OK ODOMETRY THEN
        TAKEOFF ALT 3
        WHILE FOUND_OBJ car AND CONFIDENCE_OBJ car >= 0.6 AND BATTERY >= 30 DO
            ALIGN_TO_OBJ car
            MAINTAIN_DISTANCE OBJ car DIST 5
            WAIT 0.5
        ENDWHILE
        RETURN_HOME TIMEOUT 30
        LAND TIMEOUT 10
    ELSE
        ABORT_MISSION MSG "Vision or Odometry sensor unavailable"
    ENDIF
END
```

---

## Telemetry and Sensor Management
- **Pre-Mission:** Verify all sensors, telemetry data.
- **During Mission:** Continuously monitor sensor health, battery, position, and velocity.
- **Safety:** Abort mission if critical sensor data fails or telemetry values exceed predefined thresholds.

---

## Recommended Next Steps
- Implement a Python-based DSL parser (MissionRunner).
- Integrate the parser with MAVSDK and ROS2.
- Conduct validation tests for each mission type in simulated environments (PX4 SITL).
