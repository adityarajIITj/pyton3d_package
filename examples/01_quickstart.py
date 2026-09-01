#!/usr/bin/env python3
"""Example 1: Quickstart Rigid Body Drop"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import pyton3d as p3d

def main():
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

    # Ground Plane
    ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True, name="Ground")
    ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
    ground.material = p3d.Materials.CONCRETE
    world.add_body(ground)

    # Dynamic Wood Box
    box = p3d.RigidBody(position=p3d.Vec3(0, 5.0, 0), mass=2.0, name="WoodBox")
    box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
    box.material = p3d.Materials.WOOD
    world.add_body(box)

    print("Simulating 60 steps (1 second at 60Hz)...")
    for step in range(60):
        world.step(1/60)
        if step % 10 == 0:
            print(f"Step {step:02d} | Elevation Y: {box.position.y:.3f} m | Vel Y: {box.velocity.y:.3f} m/s")

if __name__ == "__main__":
    main()
