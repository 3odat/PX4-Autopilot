#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# Movement parameters
VELOCITY = 1.0       # m/s translation
YAW_SPEED = 30.0     # deg/s rotation

async def wait_for_connection(drone):
    print("🔌 Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            return

async def wait_for_health(drone):
    print("📡 Waiting for GPS & home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ GPS & Home OK")
            return

async def ensure_armed(drone):
    for i in range(5):
        try:
            print(f"🛡️ Arming (attempt {i+1})...")
            await drone.action.arm()
            print("✅ Armed")
            return True
        except Exception as e:
            print(f"⚠️ Arm denied: {e}")
            await asyncio.sleep(1)
    print("❌ Failed to arm")
    return False

async def start_offboard(drone):
    print("🟢 Starting Offboard mode...")
    # send an initial zero setpoint
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    try:
        await drone.offboard.start()
        print("✅ Offboard mode ACTIVE")
        return True
    except OffboardError as e:
        print(f"❌ Offboard start failed: {e._result.result}")
        return False

async def stream_velocity(drone, vx, vy, vz, yaw_rate, duration):
    end = asyncio.get_event_loop().time() + duration
    print(f"🚁 Streaming vel vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} yaw={yaw_rate:.2f} for {duration:.1f}s")
    while asyncio.get_event_loop().time() < end:
        try:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
            )
        except OffboardError as e:
            print(f"❌ Velocity command denied: {e._result.result}")
            return
        await asyncio.sleep(0.1)
    # hover
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    print("🚁 Hovering")

async def move(drone, cmd, val, offboard_active):
    if not offboard_active:
        # automatically start offboard if not active
        ok = await start_offboard(drone)
        if not ok:
            return False
        offboard_active = True

    if cmd in ("forward","backward","left","right","up","down"):
        duration = val / VELOCITY
        mapping = {
            "forward": ( VELOCITY, 0, 0, 0),
            "backward":(-VELOCITY,0, 0, 0),
            "right":   (0, VELOCITY,0, 0),
            "left":    (0,-VELOCITY,0, 0),
            "up":      (0, 0, -VELOCITY,0),
            "down":    (0, 0, VELOCITY, 0),
        }
        vx, vy, vz, yaw_r = mapping[cmd]
        await stream_velocity(drone, vx, vy, vz, yaw_r, duration)

    elif cmd in ("yaw_left","yaw_right"):
        rate = -YAW_SPEED if cmd == "yaw_left" else YAW_SPEED
        duration = val / YAW_SPEED
        await stream_velocity(drone, 0, 0, 0, rate, duration)

    else:
        print("❌ Unknown move:", cmd)

    return offboard_active

async def print_status(drone):
    pos = await drone.telemetry.position().__anext__()
    batt = await drone.telemetry.battery().__anext__()
    print(f"📍 Lat={pos.latitude_deg:.6f} Lon={pos.longitude_deg:.6f} Alt={pos.relative_altitude_m:.2f}m")
    print(f"🔋 Battery={batt.remaining_percent*100:.0f}%")

async def print_mode(drone):
    mode = await drone.telemetry.flight_mode().__anext__()
    print(f"📡 Flight mode: {mode}")

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_for_connection(drone)
    await wait_for_health(drone)

    armed = False
    offboard_active = False

    print("""
🕹️ COMMANDS:
  arm                  — arm motors
  takeoff <m>          — climb to altitude
  land                 — land
  forward/backward/left/right/up/down <m>
  yaw_left/yaw_right <deg>
  stop                 — hover
  status               — show GPS & battery
  mode                 — show flight mode
  help                 — list commands
  exit                 — disarm & exit
""")

    while True:
        line = input("> ").strip().lower()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0]

        if cmd == "help":
            print("Commands: arm, takeoff, land, forward/backward/left/right/up/down <m>, yaw_left/yaw_right <deg>, stop, status, mode, exit")
            continue

        if cmd == "status":
            await print_status(drone)
            continue

        if cmd == "mode":
            await print_mode(drone)
            continue

        if cmd == "arm":
            armed = await ensure_armed(drone)
            continue

        if cmd == "takeoff":
            if not armed:
                print("⚠️ Please arm first")
                continue
            if len(parts) != 2:
                print("⚠️ Usage: takeoff <altitude_m>")
                continue
            alt = float(parts[1])
            await drone.action.set_takeoff_altitude(alt)
            await drone.action.takeoff()
            print(f"⇧ Taking off to {alt:.1f} m…")
            # wait for altitude
            await asyncio.sleep(max(alt / 1.5, 2.0))
            continue

        if cmd == "land":
            print("🛬 Landing…")
            await drone.action.land()
            # wait until landed
            async for in_air in drone.telemetry.in_air():
                if not in_air:
                    break
            break

        if cmd == "stop":
            if offboard_active:
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
                print("🚁 Hovering")
            continue

        if cmd in ("forward","backward","left","right","up","down","yaw_left","yaw_right"):
            if not armed:
                print("⚠️ Please arm first")
                continue
            if len(parts) != 2:
                print(f"⚠️ Usage: {cmd} <value>")
                continue
            val = float(parts[1])
            offboard_active = await move(drone, cmd, val, offboard_active)
            continue

        if cmd == "exit":
            break

        print("❌ Unknown command:", cmd)

    # shutdown
    if offboard_active:
        print("🛑 Stopping Offboard…")
        try:
            await drone.offboard.stop()
        except OffboardError as e:
            print(f"⚠️ Offboard stop failed: {e._result.result}")
    if armed:
        print("🛡️ Disarming…")
        await drone.action.disarm()
    print("✅ Disarmed. Goodbye!")

if __name__ == "__main__":
    asyncio.run(main())

