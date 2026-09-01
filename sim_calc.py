import pyton3d as p3d

def run_simulation():
    print("=" * 70)
    print("      PYTON3D HEADLESS PHYSICS CALCULATION & SIMULATION")
    print("=" * 70)

    # 1. Initialize World with Earth Gravity
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

    # 2. Add Immovable Ground Plane (Concrete)
    ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True, name="Ground")
    ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
    ground.material = p3d.Materials.CONCRETE
    world.add_body(ground)

    # 3. Add Dynamic Ball (Steel) launched at 45 degrees
    ball = p3d.RigidBody(
        position=p3d.Vec3(0, 1.0, 0),
        velocity=p3d.Vec3(5.0, 10.0, 0.0), # Vx = 5 m/s, Vy = 10 m/s
        mass=2.0,
        name="SteelBall"
    )
    ball.collider = p3d.SphereCollider(0.4)
    ball.material = p3d.Materials.STEEL
    world.add_body(ball)

    # 4. Add Dynamic Box (Wood) at target location
    box = p3d.RigidBody(
        position=p3d.Vec3(10.0, 1.0, 0),
        mass=1.5,
        name="TargetBox"
    )
    box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
    box.material = p3d.Materials.WOOD
    world.add_body(box)

    print(f"Initial State:")
    print(f"  Ball Position : {ball.position} | Velocity: {ball.velocity}")
    print(f"  Ball Mass     : {ball.mass} kg | Kinetic Energy: {ball.kinetic_energy():.3f} J")
    print(f"  Box Position  : {box.position} | Mass: {box.mass} kg")
    print("-" * 70)
    print(f"{'Step':<6} | {'Time (s)':<9} | {'Ball Pos (X, Y)':<22} | {'Ball Vel (m/s)':<18} | {'Total KE (J)':<10}")
    print("-" * 70)

    # 5. Run 120 Timesteps (2.0 seconds at 60 Hz)
    dt = 1.0 / 60.0
    for step in range(121):
        world.step(dt)
        sim_time = step * dt

        if step % 10 == 0:
            pos_str = f"({ball.position.x:.2f}, {ball.position.y:.2f})"
            vel_str = f"({ball.velocity.x:.2f}, {ball.velocity.y:.2f})"
            total_ke = world.total_kinetic_energy()
            print(f"{step:<6} | {sim_time:<9.2f} | {pos_str:<22} | {vel_str:<18} | {total_ke:<10.3f}")

    print("-" * 70)
    print(f"Final Calculation Results after 2.0s:")
    print(f"  Final Ball Position : {ball.position}")
    print(f"  Final Ball Velocity : {ball.velocity}")
    print(f"  Final Box Position  : {box.position}")
    print(f"  Total System Energy : {world.total_kinetic_energy():.4f} Joules")
    print("=" * 70)

if __name__ == "__main__":
    run_simulation()
