#!/usr/bin/env python3
"""
Pyton3D Terminal Demo:
Simulate a wooden box dropping onto a concrete floor with live ASCII telemetry.
"""
import pyton3d as p3d

def main():
    print("=" * 72)
    print("  Pyton3D Physics Simulation - Terminal Telemetry Demo")
    print("=" * 72)

    # 1. Create 3D physics world with Earth gravity (-9.81 m/s^2 along Y)
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

    # 2. Add static concrete floor
    ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True, name="Floor")
    ground.collider = p3d.BoxCollider(p3d.Vec3(5, 0.5, 5))
    ground.material = p3d.Materials.CONCRETE
    world.add_body(ground)

    # 3. Add dynamic wooden box dropped from 3.0 meters
    box = p3d.RigidBody(position=p3d.Vec3(0, 3.0, 0), mass=2.0, name="WoodBox")
    box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
    box.material = p3d.Materials.WOOD
    world.add_body(box)

    print(f"| Step | Time (s) | Box Y (m) | Vel Y (m/s) | Contacts | Kinetic Energy |")
    print("|" + "-"*6 + "|" + "-"*10 + "|" + "-"*11 + "|" + "-"*13 + "|" + "-"*10 + "|" + "-"*16 + "|")

    # 4. Step simulation for 60 frames (1 second at 60Hz)
    for step in range(61):
        world.step(1/60)
        if step % 5 == 0 or step == 60:
            contacts = sum(len(m) for m in world.manifolds.values())
            ke = box.kinetic_energy()
            print(f"| {step:04d} | {world.total_time:8.3f} | {box.position.y:9.3f} | {box.velocity.y:11.3f} | {contacts:8d} | {ke:12.3f} J |")

    print("-" * 72)
    print("Simulation complete! Notice the velocity reversal and rebound around step 45.")
    print("=" * 72)

if __name__ == "__main__":
    main()
