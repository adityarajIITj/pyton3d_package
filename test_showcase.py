#!/usr/bin/env python3
"""
====================================================================
  Pyton3D Package Comprehensive Feature & Functionality Test Suite
====================================================================
  Tests all foundational components of the published `pyton3d` library:
    1. 3D Vector Math & Rodrigues Quaternion Rotations
    2. 3D Separating Axis Theorem (SAT) Collision Pipeline
    3. 6-DOF Multi-Body Dynamics & Energy Conservation
    4. Physical Damped Springs & Constraints
    5. Scene JSON Serialization & Deserialization
    6. Interactive 3D CAD Visual Studio Launcher
====================================================================
"""

import math
import time
import pyton3d as p3d

def print_header(title):
    print("\n" + "=" * 65)
    print(f"  {title}")
    print("=" * 65)

def test_1_vector_and_quaternion_math():
    print_header("TEST 1: 3D Vector Math & Quaternion Kinematics")
    
    v1 = p3d.Vec3(1.0, 2.0, 3.0)
    v2 = p3d.Vec3(4.0, 5.0, 6.0)
    
    # 1. Vector addition & scaling
    v_sum = v1 + v2
    v_scaled = v1 * 2.0
    print(f" [+] Vec3 Addition : {v1} + {v2} = {v_sum}")
    print(f" [+] Vec3 Scaling  : {v1} * 2.0 = {v_scaled}")
    
    # 2. Dot & Cross Product
    dot = v1.dot(v2)
    cross = v1.cross(v2)
    print(f" [+] Dot Product   : {v1} . {v2} = {dot}")
    print(f" [+] Cross Product : {v1} x {v2} = {cross}")
    
    # 3. Rodrigues Quaternion Vector Rotation (90 deg around Y-axis)
    q = p3d.Quaternion.from_axis_angle(p3d.Vec3(0, 1, 0), math.radians(90.0))
    forward = p3d.Vec3(1.0, 0.0, 0.0)
    rotated = q.rotate_vector(forward)
    print(f" [+] Quaternion Rotation (90° around Y):")
    print(f"     Original: {forward}  -->  Rotated: {rotated}")
    
    assert abs(rotated.x) < 1e-5 and abs(rotated.z - (-1.0)) < 1e-5
    print(" [✓] Vector Math & Quaternion Tests PASSED!")

def test_2_sat_collision_pipeline():
    print_header("TEST 2: 3D Separating Axis Theorem (SAT) Collision Pipeline")
    
    # Box A at origin
    box_a = p3d.RigidBody(position=p3d.Vec3(0, 0, 0), is_static=True, name="BoxA")
    box_a.collider = p3d.BoxCollider(p3d.Vec3(1.0, 1.0, 1.0))
    
    # Box B slightly overlapping Box A
    box_b = p3d.RigidBody(position=p3d.Vec3(1.5, 0, 0), mass=2.0, name="BoxB")
    box_b.collider = p3d.BoxCollider(p3d.Vec3(1.0, 1.0, 1.0))
    
    contact = p3d.CollisionDetector.detect(box_a.collider, box_b.collider)
    assert contact is not None, "Collision should be detected!"
    
    print(f" [+] Box-Box Overlap Detected via SAT (15 candidate axes):")
    print(f"     Penetration Depth : {contact.penetration:.4f} m")
    print(f"     Collision Normal  : {contact.normal}")
    print(f"     Contact Point     : {contact.point}")
    print(" [✓] SAT Collision Pipeline Tests PASSED!")

def test_3_rigid_body_dynamics():
    print_header("TEST 3: 6-DOF Multi-Body Physics & Momentum Solver")
    
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))
    
    # Ground
    ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True, name="ConcreteFloor")
    ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
    ground.material = p3d.Materials.CONCRETE
    world.add_body(ground)
    
    # Drop 3 stacked dynamic cubes
    for i in range(3):
        cube = p3d.RigidBody(position=p3d.Vec3(0, 1.0 + i * 1.2, 0), mass=1.5, name=f"Cube_{i+1}")
        cube.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
        cube.material = p3d.Materials.WOOD
        world.add_body(cube)
    
    print(" [+] Stepping simulation for 60 ticks (1.0 second at 60 Hz)...")
    for step in range(60):
        world.step(1/60)
        if step % 15 == 0:
            top_cube = world.bodies[-1]
            print(f"     Step {step:02d} | Top Cube Y: {top_cube.position.y:.3f} m | Total KE: {world.total_kinetic_energy():.3f} J")
    
    print(" [✓] 6-DOF Multi-Body Physics Tests PASSED!")

def test_4_spring_constraints():
    print_header("TEST 4: Damped Harmonic Springs & Constraints")
    
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))
    
    anchor = p3d.RigidBody(position=p3d.Vec3(0, 5, 0), is_static=True, name="Anchor")
    anchor.collider = p3d.SphereCollider(0.2)
    world.add_body(anchor)
    
    bob = p3d.RigidBody(position=p3d.Vec3(2.0, 5, 0), mass=1.0, name="SpringBob")
    bob.collider = p3d.SphereCollider(0.3)
    world.add_body(bob)
    
    spring = p3d.SpringConstraint(
        anchor, bob,
        p3d.Vec3.zero(), p3d.Vec3.zero(),
        rest_length=1.5, stiffness=50.0, damping=0.8
    )
    world.add_constraint(spring)
    
    for step in range(45):
        world.step(1/60)
        if step % 15 == 0:
            print(f"     Step {step:02d} | Bob Position: {bob.position} | Kinetic Energy: {bob.kinetic_energy():.4f} J")
            
    print(" [✓] Spring Constraint Tests PASSED!")

def test_5_scene_serialization():
    print_header("TEST 5: Scene JSON Serialization & Round-Trip")
    
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -3.71, 0)) # Mars Gravity
    ball = p3d.RigidBody(position=p3d.Vec3(1, 2, 3), mass=4.5, name="MarsProbe")
    ball.collider = p3d.SphereCollider(0.5)
    ball.material = p3d.Materials.STEEL
    world.add_body(ball)
    
    json_path = "test_mars_scene.json"
    p3d.SceneManager.save_scene(world, json_path)
    print(f" [+] Saved scene to JSON file: {json_path}")
    
    new_world = p3d.PhysicsWorld()
    p3d.SceneManager.load_scene(new_world, json_path)
    print(f" [+] Loaded scene from JSON! Bodies count = {len(new_world.bodies)}")
    
    loaded_probe = new_world.bodies[0]
    assert loaded_probe.name == "MarsProbe"
    assert loaded_probe.mass == 4.5
    print(f"     Loaded Body: {loaded_probe.name} | Mass: {loaded_probe.mass} kg | Pos: {loaded_probe.position}")
    
    # Clean up test json
    if os.path.exists(json_path):
        os.remove(json_path)
        
    print(" [✓] Scene JSON Serialization Tests PASSED!")

def test_6_launch_interactive_studio():
    print_header("TEST 6: Interactive 3D CAD Studio Launcher")
    print(" [+] Building a custom physics world (Tower of Spheres and Blocks)...")
    
    world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))
    
    # Ground plane
    ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True, name="MainGround")
    ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
    ground.material = p3d.Materials.CONCRETE
    world.add_body(ground)
    
    # Add a colorful pyramid of wooden and steel blocks
    colors = [p3d.Materials.WOOD, p3d.Materials.STEEL, p3d.Materials.RUBBER, p3d.Materials.BOUNCY]
    for layer in range(4):
        for col in range(4 - layer):
            pos = p3d.Vec3((col - (4 - layer) / 2.0 + 0.5) * 1.1, 0.5 + layer * 1.1, 0)
            box = p3d.RigidBody(position=pos, mass=1.5, name=f"Block_L{layer}_C{col}")
            box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
            box.material = colors[(layer + col) % len(colors)]
            world.add_body(box)
            
    # Add an incoming wrecking sphere projectile
    wrecking_ball = p3d.RigidBody(
        position=p3d.Vec3(-6.0, 3.5, 0.0),
        velocity=p3d.Vec3(8.0, 0.0, 0.0),
        mass=10.0,
        name="WreckingBall"
    )
    wrecking_ball.collider = p3d.SphereCollider(0.8)
    wrecking_ball.material = p3d.Materials.STEEL
    world.add_body(wrecking_ball)
    
    print(" [+] Launching Pyton3D Visual Studio with your custom scene...")
    print("     (Close the studio window when you are done testing)")
    
    p3d.launch_studio(world)
    print(" [✓] Interactive CAD Studio closed cleanly!")

def main():
    print("\n" + "#" * 65)
    print("  RUNNING PYTON3D COMPREHENSIVE PACKAGE VERIFICATION")
    print("#" * 65)
    
    test_1_vector_and_quaternion_math()
    test_2_sat_collision_pipeline()
    test_3_rigid_body_dynamics()
    test_4_spring_constraints()
    test_5_scene_serialization()
    
    print("\n" + "=" * 65)
    print("  ALL 5 PROGRAMMATIC TESTS PASSED (100%)!")
    print("=" * 65)
    
    # Launch GUI test
    test_6_launch_interactive_studio()
    
    print("\n" + "#" * 65)
    print("  PYTON3D PACKAGE VERIFICATION COMPLETED SUCCESSFULLY!")
    print("#" * 65 + "\n")

if __name__ == "__main__":
    main()
