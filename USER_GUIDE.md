# Pyton3D Developer and User Guide

This guide provides a comprehensive technical overview and programmatic manual for **Pyton3D**—a 6-Degrees-of-Freedom (6-DOF) 3D rigid body physics simulation engine and interactive CAD workbench built from first principles in pure Python.

Every component—linear algebra, quaternion kinematics, 15-axis Separating Axis Theorem (SAT) collision detection, numerical integrators, iterative impulse manifolds, and Coulomb friction cones—is accessible both programmatically via the Python API and interactively via the desktop CAD Studio.

---

## Table of Contents
1. [Installation and Quick Start](#1-installation-and-quick-start)
2. [3D Vector Math and Spatial Kinematics](#2-3d-vector-math-and-spatial-kinematics)
3. [Rigid Bodies and Physical Materials](#3-rigid-bodies-and-physical-materials)
4. [Collision Detection and Geometric Queries](#4-collision-detection-and-geometric-queries)
5. [Constraints, Springs, and Inelastic Joints](#5-constraints-springs-and-inelastic-joints)
6. [Environmental Forces and Drag Models](#6-environmental-forces-and-drag-models)
7. [Numerical Integrators and Stability](#7-numerical-integrators-and-stability)
8. [Headless Simulation and Terminal Telemetry](#8-headless-simulation-and-terminal-telemetry)
9. [Pre-Configured Classical Mechanics Labs](#9-pre-configured-classical-mechanics-labs)
10. [Interactive CAD Desktop Studio](#10-interactive-cad-desktop-studio)
11. [Scene JSON Serialization](#11-scene-json-serialization)
12. [Developer Recipe Book](#12-developer-recipe-book)

---

## 1. Installation and Quick Start

### Installation
Install Pyton3D into your Python environment:
```bash
pip install pyton3d
```

### Basic Free-Fall Simulation Script
```python
import pyton3d as p3d

# 1. Initialize physics world with Earth gravity
world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

# 2. Add static ground plane
ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True, name="Ground")
ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
ground.material = p3d.Materials.CONCRETE
world.add_body(ground)

# 3. Add dynamic wooden box dropped from 5.0 meters
box = p3d.RigidBody(position=p3d.Vec3(0, 5.0, 0), mass=2.5, name="WoodBox")
box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
box.material = p3d.Materials.WOOD
world.add_body(box)

# 4. Advance physics by 60 frames (1 second at 60 Hz)
for step in range(60):
    world.step(dt=1/60)
    if step % 15 == 0:
        print(f"Step {step:02d} | Elevation: {box.position.y:.3f} m | Velocity Y: {box.velocity.y:.3f} m/s")
```

---

## 2. 3D Vector Math and Spatial Kinematics

Pyton3D contains an optimized pure-Python vector and matrix algebra engine.

### 2.1 Vector3 (`Vec3`)
Represents points, linear velocities, accelerations, and forces in 3D Euclidean space:

```python
import pyton3d as p3d

# Instantiation
v1 = p3d.Vec3(1.0, 2.0, 3.0)
v2 = p3d.Vec3(4.0, 5.0, 6.0)

# Directional Constants
zero = p3d.Vec3.zero()       # Vec3(0, 0, 0)
up = p3d.Vec3.up()           # Vec3(0, 1, 0)
right = p3d.Vec3.right()     # Vec3(1, 0, 0)
forward = p3d.Vec3.forward() # Vec3(0, 0, 1)

# Arithmetic Operator Overloads
v_sum = v1 + v2              # Vec3(5.0, 7.0, 9.0)
v_diff = v2 - v1             # Vec3(3.0, 3.0, 3.0)
v_scaled = v1 * 2.5          # Vec3(2.5, 5.0, 7.5)
v_neg = -v1                  # Vec3(-1.0, -2.0, -3.0)

# Geometric Operations
dot_val = v1.dot(v2)         # Scalar product: 32.0
cross_val = v1.cross(v2)     # Orthogonal cross product: Vec3(-3.0, 6.0, -3.0)
length = v1.length()         # Euclidean norm
unit = v1.normalize()        # Unit vector (norm = 1.0)
dist = v1.distance_to(v2)    # Distance between points
perp = v1.perpendicular()    # Deterministic orthogonal vector
```

### 2.2 Quaternions (`Quaternion`) and Spatial Rotations
Quaternions parameterize 3D orientations without gimbal lock and rotate arbitrary vectors using the Rodrigues formulation:

```python
import math
import pyton3d as p3d

# Construct rotation: 90 degrees around Y axis
axis = p3d.Vec3(0, 1, 0)
angle = math.radians(90.0)
q = p3d.Quaternion.from_axis_angle(axis, angle)

# Rotate vector via 15 FLOPs Rodrigues formula:
v = p3d.Vec3(1.0, 0.0, 0.0)
v_rotated = q.rotate_vector(v)
print(f"Rotated Vector: {v_rotated}")  # Vec3(0.000, 0.000, -1.000)

# Quaternion Multiplication (combining successive rotations)
q2 = p3d.Quaternion.from_axis_angle(p3d.Vec3(1, 0, 0), math.radians(45.0))
q_combined = q * q2

# Operator Overloads
q_sum = q + q2
q_diff = q - q2
```

### 2.3 3x3 Matrices (`Mat3`)
Used for spatial inertia tensors and coordinate frames:

```python
import pyton3d as p3d

# Identity matrix
I = p3d.Mat3()

# Matrix from components
M = p3d.Mat3([[2, 0, 0], [0, 3, 0], [0, 0, 4]])

# Operations
det = M.determinant()       # Determinant: 24.0
inv = M.inverse()           # Analytical inverse
trans = M.transpose()       # Transposed matrix

# Matrix-Vector and Matrix-Matrix multiplication
v_trans = M * p3d.Vec3(1, 1, 1)  # Vec3(2.0, 3.0, 4.0)
```

---

## 3. Rigid Bodies and Physical Materials

### 3.1 Creating Rigid Bodies (`RigidBody`)
A `RigidBody` tracks position, orientation, velocity, angular velocity, and world-space inertia:

```python
import pyton3d as p3d

# Dynamic rigid body
body = p3d.RigidBody(
    position=p3d.Vec3(0.0, 5.0, 0.0),
    mass=2.0,
    name="DynamicBox"
)
body.velocity = p3d.Vec3(1.0, 0.0, 0.0)
body.angular_velocity = p3d.Vec3(0.0, 2.0, 0.0)

# Static immovable body (infinite mass, zero inverse mass)
ground = p3d.RigidBody(
    position=p3d.Vec3(0.0, -0.5, 0.0),
    is_static=True,
    name="GroundPlane"
)
```

### 3.2 Material Presets and Custom Materials
Materials govern restitution (bounciness) and Coulomb friction:

```python
import pyton3d as p3d

# Built-in presets
body.material = p3d.Materials.WOOD
body.material = p3d.Materials.STEEL
body.material = p3d.Materials.RUBBER
body.material = p3d.Materials.ICE
body.material = p3d.Materials.CONCRETE
body.material = p3d.Materials.BOUNCY

# Custom material
custom_mat = p3d.Material(
    name="SuperElasticPolymer",
    density=1.1,
    restitution=0.92,
    static_friction=0.45,
    dynamic_friction=0.25,
    color=(0.2, 0.8, 0.3)
)
body.material = custom_mat
```

---

## 4. Collision Detection and Geometric Queries

Pyton3D uses the 15-axis Separating Axis Theorem (SAT) for 3D Oriented Bounding Boxes (OBB).

### 4.1 Attaching Colliders
```python
import pyton3d as p3d

# 1. Box Collider (half-extents along X, Y, Z)
body.collider = p3d.BoxCollider(half_extents=p3d.Vec3(0.5, 1.0, 0.5))

# 2. Sphere Collider (radius)
sphere_body = p3d.RigidBody(position=p3d.Vec3(0, 3, 0), mass=1.5)
sphere_body.collider = p3d.SphereCollider(radius=0.6)

# 3. Plane Collider (infinite half-space with normal and distance offset)
plane_body = p3d.RigidBody(position=p3d.Vec3(0, 0, 0), is_static=True)
plane_body.collider = p3d.PlaneCollider(normal=p3d.Vec3.up(), offset=0.0)
```

### 4.2 Narrow-Phase Collision Inspection
You can execute direct collision tests between colliders without running the full simulation loop:

```python
import pyton3d as p3d

c1 = p3d.SphereCollider(radius=1.0)
b1 = p3d.RigidBody(position=p3d.Vec3(0, 0, 0), collider=c1)

c2 = p3d.SphereCollider(radius=1.0)
b2 = p3d.RigidBody(position=p3d.Vec3(1.5, 0, 0), collider=c2)

contact = p3d.CollisionDetector.detect(b1.collider, b2.collider)
if contact:
    print(f"Collision Detected!")
    print(f"Penetration Depth: {contact.penetration:.4f} m")
    print(f"Contact Normal:    {contact.normal}")
    print(f"Contact Point:     {contact.point}")
```

---

## 5. Constraints, Springs, and Inelastic Joints

### 5.1 Damped Harmonic Springs (`SpringConstraint`)
Implements Hooke\'s law with velocity damping along the separation axis:

```python
import pyton3d as p3d

world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

anchor = p3d.RigidBody(position=p3d.Vec3(0, 5, 0), is_static=True)
anchor.collider = p3d.SphereCollider(0.2)
world.add_body(anchor)

weight = p3d.RigidBody(position=p3d.Vec3(1.5, 5, 0), mass=1.0)
weight.collider = p3d.SphereCollider(0.3)
world.add_body(weight)

# Spring constraint between anchor and weight
spring = p3d.SpringConstraint(
    body_a=anchor,
    body_b=weight,
    local_a=p3d.Vec3.zero(),
    local_b=p3d.Vec3.zero(),
    rest_length=1.0,    # Target equilibrium separation (m)
    stiffness=50.0,     # Spring stiffness k (N/m)
    damping=0.8         # Damping coefficient c
)
world.add_constraint(spring)
```

### 5.2 Inelastic Distance Joints (`DistanceConstraint`)
Enforces rigid distance separation with compliance:

```python
rod = p3d.DistanceConstraint(
    body_a=anchor,
    body_b=weight,
    local_a=p3d.Vec3.zero(),
    local_b=p3d.Vec3.zero(),
    distance=2.0,       # Inelastic length
    compliance=0.0001   # Near-zero compliance for rigid behavior
)
world.add_constraint(rod)
```

---

## 6. Environmental Forces and Drag Models

### 6.1 Planetary Gravity Presets
```python
import pyton3d as p3d

world = p3d.PhysicsWorld()

# Presets
world.gravity = p3d.Vec3(0, -9.81, 0)   # Earth
world.gravity = p3d.Vec3(0, -1.62, 0)   # Moon
world.gravity = p3d.Vec3(0, -3.71, 0)   # Mars
world.gravity = p3d.Vec3.zero()         # Zero Gravity
```

### 6.2 Aerodynamic Resistance and Fluid Buoyancy
```python
# Configure linear and quadratic aerodynamic drag
world.drag_force.k1 = 0.05  # Linear velocity resistance
world.drag_force.k2 = 0.005 # Quadratic velocity resistance

# Enable Archimedes buoyancy
world.buoyancy_force.enabled = True
world.buoyancy_force.liquid_level = 0.0    # Elevation of fluid surface
world.buoyancy_force.liquid_density = 1.0  # Relative fluid density
world.buoyancy_force.max_depth = 1.5       # Submersion depth scaling
```

---

## 7. Numerical Integrators and Stability

Pyton3D supports 4 numerical integrators selectable per world:

```python
import pyton3d as p3d

world = p3d.PhysicsWorld()

# 1. Symplectic Euler (Default - 1st order semi-implicit, excellent energy conservation)
world.integration_method = p3d.IntegrationMethod.SYMPLECTIC_EULER

# 2. Velocity Verlet (2nd order symplectic - Ideal for conservative orbital oscillators)
world.integration_method = p3d.IntegrationMethod.VERLET

# 3. Runge-Kutta 4th Order (RK4 - 4-stage derivative evaluation, high accuracy)
world.integration_method = p3d.IntegrationMethod.RK4

# 4. Explicit Euler (Standard 1st order reference)
world.integration_method = p3d.IntegrationMethod.EULER
```

---

## 8. Headless Simulation and Terminal Telemetry

Pyton3D provides detailed tabular telemetry logging in headless terminal sessions.

### Tabular Telemetry Script
```python
import pyton3d as p3d

world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

# Add floor
ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True)
ground.collider = p3d.BoxCollider(p3d.Vec3(5, 0.5, 5))
ground.material = p3d.Materials.CONCRETE
world.add_body(ground)

# Add dropping box
box = p3d.RigidBody(position=p3d.Vec3(0, 3.0, 0), mass=2.0, name="WoodBox")
box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
box.material = p3d.Materials.WOOD
world.add_body(box)

print(f"| Step | Time (s) | Box Y (m) | Vel Y (m/s) | Contacts | Kinetic Energy |")
print("|" + "-"*6 + "|" + "-"*10 + "|" + "-"*11 + "|" + "-"*13 + "|" + "-"*10 + "|" + "-"*16 + "|")

for step in range(61):
    world.step(1/60)
    if step % 5 == 0 or step == 60:
        contacts = sum(len(m) for m in world.manifolds.values())
        ke = box.kinetic_energy()
        print(f"| {step:04d} | {world.total_time:8.3f} | {box.position.y:9.3f} | {box.velocity.y:11.3f} | {contacts:8d} | {ke:12.3f} J |")
```

---

## 9. Pre-Configured Classical Mechanics Labs

Load any of the 7 pre-configured physics labs with one line of code:

```python
import pyton3d as p3d

world = p3d.PhysicsWorld()

# Available presets: "stack", "spheres", "mixed", "springs", "jenga", "buoyancy", "cradle"
p3d.SceneManager.load_preset(world, "jenga")

print(f"Loaded Jenga Tower with {len(world.bodies)} active rigid bodies.")

# Step simulation in terminal
for step in range(120):
    world.step(1/60)
    if step % 30 == 0:
        print(f"Step {step:03d} | Total Kinetic Energy: {world.get_total_kinetic_energy():.3f} J")
```

---

## 10. Interactive CAD Desktop Studio

Launch the full Tkinter + Matplotlib 3D Studio directly from code:

```python
import pyton3d as p3d

# Option A: Clean studio with ground plane
p3d.launch_studio()

# Option B: Launch studio with custom pre-configured world
world = p3d.PhysicsWorld()
p3d.SceneManager.load_preset(world, "cradle")
p3d.launch_studio(world=world)
```

### Keyboard Shortcuts
- `SPACE`: Toggle continuous simulation execution
- `S`: Advance simulation by 1 frame
- `Ctrl + N`: New blank scene
- `Ctrl + O`: Open scene from JSON
- `Ctrl + S`: Save scene to JSON
- `A`: Toggle AABB bounding box wireframes
- `C`: Toggle contact normal vectors and collision points
- `V`: Toggle linear velocity vectors
- `G`: Toggle coordinate plane floor grid

---

## 11. Scene JSON Serialization

Scenes can be completely serialized to and loaded from JSON:

```python
import pyton3d as p3d

world = p3d.PhysicsWorld()
p3d.SceneManager.load_preset(world, "springs")

# Save to disk
p3d.SceneManager.save_scene(world, "spring_scene.json")

# Restore in a fresh world
restored_world = p3d.PhysicsWorld()
p3d.SceneManager.load_scene(restored_world, "spring_scene.json")
print(f"Restored {len(restored_world.bodies)} bodies successfully.")
```

---

## 12. Developer Recipe Book

### Recipe 1: Projectile Ballistics Simulation
```python
import pyton3d as p3d

world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

shell = p3d.RigidBody(
    position=p3d.Vec3(0, 1.0, 0),
    velocity=p3d.Vec3(30.0, 20.0, 0.0),
    mass=10.0
)
shell.collider = p3d.SphereCollider(0.25)
world.add_body(shell)

dt = 1 / 120.0
while shell.position.y > 0.0:
    world.step(dt)

print(f"Impact Range: {shell.position.x:.2f} m | Total Flight Time: {world.total_time:.2f} s")
```

### Recipe 2: Momentum Transfer Verification (Newton's Cradle)
```python
import pyton3d as p3d

world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))
p3d.SceneManager.load_preset(world, "cradle")

initial_ke = world.get_total_kinetic_energy()
for step in range(240):
    world.step(1/60)

final_ke = world.get_total_kinetic_energy()
print(f"Initial KE: {initial_ke:.3f} J | Final KE: {final_ke:.3f} J")
```

---

## License

Pyton3D is open-source software distributed under the **MIT License**.  
GitHub Repository: **https://github.com/adityarajIITj/pyton3d**
