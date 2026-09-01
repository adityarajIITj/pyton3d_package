#!/usr/bin/env python3
"""Example 3: Integrator Accuracy & Energy Conservation Benchmark"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import pyton3d as p3d

def benchmark_integrator(method, name):
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))
    world.integration_method = method

    anchor = p3d.RigidBody(position=p3d.Vec3(0, 0, 0), is_static=True)
    anchor.collider = p3d.SphereCollider(0.1)
    world.add_body(anchor)

    bob = p3d.RigidBody(position=p3d.Vec3(2.0, 0, 0), mass=1.0)
    bob.collider = p3d.SphereCollider(0.2)
    world.add_body(bob)

    # Undamped spring
    spring = p3d.SpringConstraint(anchor, bob, p3d.Vec3.zero(), p3d.Vec3.zero(), rest_length=1.0, stiffness=50.0, damping=0.0)
    world.add_constraint(spring)

    energies = []
    for _ in range(300):
        world.step(1/60)
        energies.append(bob.kinetic_energy())

    avg_ke = sum(energies) / len(energies)
    max_ke = max(energies)
    print(f"[{name:<18}] Max KE: {max_ke:.4f} J | Avg KE: {avg_ke:.4f} J")

def main():
    print("Benchmarking Numerical Integrators in Closed Oscillator System:")
    print("-" * 60)
    benchmark_integrator(p3d.IntegrationMethod.EULER, "Explicit Euler")
    benchmark_integrator(p3d.IntegrationMethod.SYMPLECTIC_EULER, "Symplectic Euler")
    benchmark_integrator(p3d.IntegrationMethod.VERLET, "Velocity Verlet")
    benchmark_integrator(p3d.IntegrationMethod.RK4, "Runge-Kutta 4")
    print("-" * 60)

if __name__ == "__main__":
    main()
