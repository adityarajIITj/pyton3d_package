#!/usr/bin/env python3
"""Example 2: Damped Harmonic Spring Chain"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import pyton3d as p3d

def main():
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

    anchor = p3d.RigidBody(position=p3d.Vec3(0, 5.0, 0), is_static=True, name="Anchor")
    anchor.collider = p3d.SphereCollider(0.2)
    world.add_body(anchor)

    bob = p3d.RigidBody(position=p3d.Vec3(1.5, 5.0, 0), mass=1.0, name="Bob")
    bob.collider = p3d.SphereCollider(0.3)
    bob.material = p3d.Materials.STEEL
    world.add_body(bob)

    spring = p3d.SpringConstraint(anchor, bob, p3d.Vec3.zero(), p3d.Vec3.zero(), rest_length=1.0, stiffness=40.0, damping=0.8)
    world.add_constraint(spring)

    print("Running Damped Harmonic Spring Simulation...")
    for step in range(60):
        world.step(1/60)
        if step % 10 == 0:
            print(f"Step {step:02d} | Bob Pos: {bob.position} | Kinetic Energy: {bob.kinetic_energy():.3f} J")

if __name__ == "__main__":
    main()
