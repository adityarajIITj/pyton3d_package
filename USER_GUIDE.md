# Pyton3D Developer & User Guide

Welcome to the comprehensive developer guide for **Pyton3D**—a lightweight, pure-Python 3D rigid body physics engine and interactive CAD workbench built from first principles.

This guide walks you step-by-step through every feature, class, mathematical vector operation, collision query, and visual command available in the library.

---

## Table of Contents
1. [Installation & Quick Start](#1-installation--quick-start)
2. [3D Vector Math & Spatial Transformations](#2-3d-vector-math--spatial-transformations)
3. [Rigid Bodies & Material Properties](#3-rigid-bodies--material-properties)
4. [Collision Detection & Geometric Queries](#4-collision-detection--geometric-queries)
5. [Constraints, Springs & Joints](#5-constraints-springs--joints)
6. [Environmental Forces & Custom Force Fields](#6-environmental-forces--custom-force-fields)
7. [Numerical Integrators & Physics World](#7-numerical-integrators--physics-world)
8. [Visual Player & Interactive CAD Studio](#8-visual-player--interactive-cad-studio)
9. [Scene JSON Serialization](#9-scene-json-serialization)
10. [Developer Recipe Book](#10-developer-recipe-book)

---

## 1. Installation & Quick Start

### Installation
Install Pyton3D into your Python environment:
```bash
pip install pyton3d
```

### 10-Second Quickstart
```python
import pyton3d as p3d

# 1. Create a physics world with Earth gravity
world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

# 2. Add an immovable ground block
ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True)
ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
ground.material = p3d.Materials.CONCRETE
world.add_body(ground)

# 3. Add a dynamic wooden cube
box = p3d.RigidBody(position=p3d.Vec3(0, 5.0, 0), mass=2.5)
box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
box.material = p3d.Materials.WOOD
world.add_body(box)

# 4. Advance physics by 1 second at 60 Hz
for step in range(60):
    world.step(dt=1/60)

print(f"Final box position: {box.position}")
```

---

## 2. 3D Vector Math & Spatial Transformations

Pyton3D provides an optimized, pure-Python linear algebra core.

### 2.1 Vector3 (`Vec3`)
Represents 3D coordinates, velocities, accelerations, and forces:

```python
import pyton3d as p3d

# Instantiation
v1 = p3d.Vec3(1.0, 2.0, 3.0)
v2 = p3d.Vec3(4.0, 5.0, 6.0)

# Built-in Direction Constants
up = p3d.Vec3.up()         # Vec3(0, 1, 0)
down = p3d.Vec3.down()     # Vec3(0, -1, 0)
zero = p3d.Vec3.zero()     # Vec3(0, 0, 0)

# Arithmetic Overloads
v3 = v1 + v2               # Vec3(5.0, 7.0, 9.0)
v4 = v2 - v1               # Vec3(3.0, 3.0, 3.0)
scaled = v1 * 2.5          # Vec3(2.5, 5.0, 7.5)

# Vector Algebra
dot_product = v1.dot(v2)   # Scalar dot product: 32.0
cross_prod = v1.cross(v2)  # Perpendicular vector: Vec3(-3.0, 6.0, -3.0)
magnitude = v1.length()    # Euclidean norm: 3.7416...
unit_vector = v1.normalized()
```

### 2.2 Quaternions (`Quaternion`) & Spatial Rotations
Quaternions prevent gimbal lock and represent orientation in 3D:

```python
import math
import pyton3d as p3d

# Create identity orientation
q_identity = p3d.Quaternion.identity()

# Create a rotation from Axis and Angle (e.g. Rotate 90 degrees around Y-axis)
axis = p3d.Vec3(0, 1, 0)
angle = math.radians(90.0)
q_rot = p3d.Quaternion.from_axis_angle(axis, angle)

# Rotate a vector using Rodrigues formula (15 FLOPs)
point = p3d.Vec3(1.0, 0.0, 0.0)
rotated_point = q_rot.rotate_vector(point)
print(rotated_point)  # Vec3(0.000, 0.000, -1.000)

# Quaternion Multiplication (combining successive rotations)
q_combined = q_rot * p3d.Quaternion.from_axis_angle(p3d.Vec3(1, 0, 0), math.radians(45.0))
```

### 2.3 3x3 Matrices (`Mat3`)
Used for spatial inertia tensors and coordinate basis frames:

```python
import pyton3d as p3d

# Identity and custom matrices
I = p3d.Mat3.identity()
M = p3d.Mat3([[2, 0, 0], [0, 3, 0], [0, 0, 4]])

# Matrix operations
inv_M = M.inverse()        # Inverse matrix
det = M.determinant()      # Determinant: 24.0
transposed = M.transpose() # Transpose matrix
```

---

## 3. Rigid Bodies & Material Properties

### 3.1 Creating Rigid Bodies (`RigidBody`)
A `RigidBody` represents an object with mass, orientation, velocities, and colliders.

```python
import pyton3d as p3d

# Dynamic Moving Body
body = p3d.RigidBody(
    position=p3d.Vec3(0.0, 10.0, 0.0),
    velocity=p3d.Vec3(2.0, 0.0, 0.0),       # Initial linear velocity (m/s)
    angular_velocity=p3d.Vec3(0.0, 1.0, 0.0), # Spin velocity (rad/s)
    mass=3.0,                               # Mass in kilograms
    is_static=False,                        # Dynamic body
    name="Satellite"
)

# Immovable Static Anchor (e.g. Floor, Wall, Static Obstacle)
wall = p3d.RigidBody(
    position=p3d.Vec3(10.0, 0.0, 0.0),
    is_static=True,
    name="ImmovableWall"
)
```

### 3.2 Material Presets & Custom Materials
Materials define physical friction and restitution (bounciness):

```python
import pyton3d as p3d

# Using built-in material presets
body.material = p3d.Materials.WOOD
body.material = p3d.Materials.STEEL
body.material = p3d.Materials.RUBBER
body.material = p3d.Materials.ICE
body.material = p3d.Materials.CONCRETE
body.material = p3d.Materials.BOUNCY

# Creating a custom physical material
custom_mat = p3d.Material(
    name="SuperRubber",
    density=1.2,           # Density (g/cm³)
    restitution=0.95,      # High bounciness (0.0 = clay, 1.0 = superball)
    static_friction=0.6,   # Resistance to start sliding
    dynamic_friction=0.4,  # Resistance while sliding
    color=(0.1, 0.8, 0.4)  # RGB visualization color
)
body.material = custom_mat
```

---

## 4. Collision Detection & Geometric Queries

Pyton3D implements the **3D Separating Axis Theorem (SAT)** to detect intersections across arbitrary 3D orientations.

### 4.1 Attaching Colliders
```python
import pyton3d as p3d

# 1. Box Collider (half-extents: half_width, half_height, half_depth)
box_collider = p3d.BoxCollider(p3d.Vec3(0.5, 1.0, 0.5))  # Full size: 1.0 x 2.0 x 1.0 m

# 2. Sphere Collider (radius)
sphere_collider = p3d.SphereCollider(radius=0.75)

# 3. Half-Space Plane Collider (normal, distance)
floor_collider = p3d.PlaneCollider(normal=p3d.Vec3.up(), distance=0.0)

# Attach to body
body.collider = box_collider
```

### 4.2 Querying Collisions Programmatically
You can query collisions between any two shapes directly without stepping the entire world:

```python
import pyton3d as p3d

body_a = p3d.RigidBody(position=p3d.Vec3(0, 0, 0))
body_a.collider = p3d.BoxCollider(p3d.Vec3(1, 1, 1))

body_b = p3d.RigidBody(position=p3d.Vec3(1.5, 0, 0))
body_b.collider = p3d.SphereCollider(radius=1.0)

# Perform SAT collision test
contact = p3d.CollisionDetector.detect(body_a.collider, body_b.collider)

if contact:
    print(f"Collision Detected!")
    print(f"Penetration Depth : {contact.penetration:.4f} m")
    print(f"Collision Normal  : {contact.normal}")
    print(f"Contact Point     : {contact.point}")
```

---

## 5. Constraints, Springs & Joints

Connect multiple bodies together using physical constraints:

### 5.1 Damped Harmonic Springs (`SpringConstraint`)
```python
import pyton3d as p3d

anchor = p3d.RigidBody(position=p3d.Vec3(0, 5, 0), is_static=True)
weight = p3d.RigidBody(position=p3d.Vec3(0, 2, 0), mass=2.0)

# Connect with spring (Hooke's Law with velocity damping)
spring = p3d.SpringConstraint(
    body_a=anchor,
    body_b=weight,
    local_anchor_a=p3d.Vec3.zero(),
    local_anchor_b=p3d.Vec3.zero(),
    rest_length=2.0,      # Equilibrium length (m)
    stiffness=100.0,      # Spring constant k (N/m)
    damping=1.5           # Viscous damping factor
)

world.add_constraint(spring)
```

### 5.2 Rigid Inelastic Distance Joints (`DistanceConstraint`)
```python
# Fixed distance rod (e.g. Pendulums or Newton's Cradle)
rod = p3d.DistanceConstraint(
    body_a=anchor,
    body_b=weight,
    distance=3.0          # Rigid separation maintained each frame
)
world.add_constraint(rod)
```

---

## 6. Environmental Forces & Custom Force Fields

### 6.1 Planetary Gravity Presets
```python
import pyton3d as p3d

# Standard Earth Gravity (-9.81 m/s²)
world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

# Moon Gravity (-1.62 m/s²)
world.gravity = p3d.Vec3(0, -1.62, 0)

# Mars Gravity (-3.71 m/s²)
world.gravity = p3d.Vec3(0, -3.71, 0)

# Zero Gravity
world.gravity = p3d.Vec3.zero()
```

### 6.2 Aerodynamic Drag & Fluid Buoyancy
```python
# Enable aerodynamic air resistance
world.air_drag_enabled = True
world.air_density = 1.225  # kg/m³

# Enable fluid Archimedes buoyancy
world.fluid_buoyancy_enabled = True
world.water_level = 0.0    # Elevation of water surface
world.fluid_density = 1000.0 # Water density (kg/m³)
```

### 6.3 Writing Custom Force Fields
Subclass `ForceGenerator` to plug in custom equations (e.g. Coulomb electrostatic, Lorentz magnetic, vortex forces):

```python
import pyton3d as p3d

class CentralGravityForce(p3d.ForceGenerator):
    """Applies Newtonian inverse-square gravity towards origin (0,0,0)"""
    def __init__(self, GM=500.0):
        self.GM = GM

    def apply(self, body: p3d.RigidBody, dt: float):
        if body.is_static: return
        r = body.position
        dist_sq = r.length_squared()
        if dist_sq > 0.1:
            force_mag = (self.GM * body.mass) / dist_sq
            force_vec = r.normalized() * (-force_mag)
            body.apply_force(force_vec)

world.add_force_generator(CentralGravityForce())
```

---

## 7. Numerical Integrators & Physics World

Pyton3D supports 4 integration schemes for different simulation requirements:

```python
import pyton3d as p3d

world = p3d.PhysicsWorld()

# 1. Symplectic Euler (Default - Recommended for game dev and general mechanics)
world.integration_method = p3d.IntegrationMethod.SYMPLECTIC_EULER

# 2. Velocity Verlet (Time-reversible - Ideal for orbital mechanics and molecular dynamics)
world.integration_method = p3d.IntegrationMethod.VERLET

# 3. Runge-Kutta 4th Order (RK4 - Maximum numerical precision)
world.integration_method = p3d.IntegrationMethod.RK4

# 4. Explicit Euler (Baseline comparison)
world.integration_method = p3d.IntegrationMethod.EULER

# Tune Impulse Solver Iterations (1-30)
world.solver_iterations = 12
```

---

## 8. Visual Player & Interactive CAD Studio

Launch the 3D GUI simulation player directly from Python or the command line:

### Launching via Python
```python
import pyton3d as p3d

# Option A: Open clean CAD studio
p3d.launch_studio()

# Option B: Open CAD studio loaded with your custom world
world = p3d.PhysicsWorld()
# ... populate world ...
p3d.launch_studio(world)
```

### Launching via Command Line
```bash
pyton3d
```

### Interactive Viewport Controls
- **Left Mouse Drag**: Orbit 3D camera.
- **Right Mouse Drag**: Zoom in / out.
- **Home Button**: Reset camera to isometric CAD perspective.
- **SPACE**: Pause / Resume physics simulation.
- **S**: Step forward by exactly one frame.
- **A**: Toggle AABB bounding box wireframes.
- **C**: Toggle contact points and collision normal vectors.
- **V**: Toggle linear velocity vectors.
- **G**: Toggle floor coordinate grid.

---

## 9. Scene JSON Serialization

Save and load entire multi-body physics setups to structured JSON files:

```python
import pyton3d as p3d

# Save current scene
p3d.SceneManager.save_scene(world, "my_scene.json")

# Load saved scene into world
p3d.SceneManager.load_scene(world, "my_scene.json")
```

---

## 10. Developer Recipe Book

### Recipe 1: Projectile Trajectory Calculation
```python
import pyton3d as p3d

world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

cannonball = p3d.RigidBody(
    position=p3d.Vec3(0, 1.5, 0),
    velocity=p3d.Vec3(25.0, 15.0, 0.0), # 45-degree launch
    mass=5.0
)
cannonball.collider = p3d.SphereCollider(0.2)
world.add_body(cannonball)

time_elapsed = 0.0
dt = 1/120.0
while cannonball.position.y > 0.0:
    world.step(dt)
    time_elapsed += dt

print(f"Range: {cannonball.position.x:.2f} m | Flight Time: {time_elapsed:.2f} s")
```

---

## License & Support
Pyton3D is open-source software licensed under the **MIT License**.  
For issues, contributions, and documentation, visit:  
**https://github.com/adityarajIITj/pyton3d**
