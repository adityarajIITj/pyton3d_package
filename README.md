# Pyton3D Package

[![PyPI Version](https://img.shields.io/badge/PyPI-v0.1.0-blue.svg)](https://pypi.org/project/pyton3d/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Python: 3.9+](https://img.shields.io/badge/Python-3.9+-brightgreen.svg)](https://www.python.org/downloads/)
[![Physics Engine: Built From Scratch](https://img.shields.io/badge/Physics_Engine-Built_From_Scratch-success.svg)]()
[![Tests: Pytest Passing](https://img.shields.io/badge/Tests-6%2F6%20Passing-brightgreen.svg)]()

> **Pyton3D** is an open-source, pure-Python 3D rigid body dynamics library and interactive CAD simulation workbench, **built completely from scratch** with zero proprietary physics dependencies (no PyBullet, Box2D, ODE, or PhysX).

It is designed for researchers, educators, roboticists, and developers who need transparent, hackable 6-DOF physics modeling, vector calculus, 3D SAT collision detection, and instant visual simulation.

---

## Key Features

- **Built 100% From Scratch**: Every vector, quaternion, collision manifold, and integrator is written in foundational Python and NumPy.
- **6-DOF Rigid Body Dynamics**: Quaternion rotations with Rodrigues vector rotation optimization (15 FLOPs), world-space inertia tensor updates, and linear/angular momentum conservation.
- **3D Collision Pipeline (SAT)**: Separating Axis Theorem evaluating all 15 potential separating axes for Oriented Bounding Boxes (OBB), spheres, and half-space planes.
- **4 Numerical Integrators**: Symplectic Euler, Velocity Verlet, Runge-Kutta 4th Order (RK4), and Explicit Euler.
- **Constraints & Forces**: Damped harmonic springs, rigid distance linkages, planetary gravity fields, aerodynamic air drag, and Archimedes fluid buoyancy.
- **Interactive Visual CAD Player**: Launch the desktop 3D CAD simulation studio directly with one function call (`p3d.launch_studio()`) or via the command line (`pyton3d`).

---

## Installation

Install the package via `pip`:

```bash
pip install pyton3d
```

For development and running tests locally:
```bash
git clone https://github.com/adityarajIITj/pyton3d_package.git
cd pyton3d_package
pip install -e .[dev]
```

---

## Quick Start Guide

### 1. Headless Python Simulation (10 Lines)
```python
import pyton3d as p3d

# Create world with Earth gravity
world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))

# Add static concrete ground
ground = p3d.RigidBody(position=p3d.Vec3(0, -0.5, 0), is_static=True)
ground.collider = p3d.BoxCollider(p3d.Vec3(10, 0.5, 10))
ground.material = p3d.Materials.CONCRETE
world.add_body(ground)

# Add dynamic wooden box
box = p3d.RigidBody(position=p3d.Vec3(0, 5.0, 0), mass=2.0)
box.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
box.material = p3d.Materials.WOOD
world.add_body(box)

# Step physics loop (60 Hz)
for step in range(60):
    world.step(dt=1/60)
    if step % 10 == 0:
        print(f"Step {step:02d} | Elevation: {box.position.y:.3f} m | Velocity: {box.velocity.y:.3f} m/s")
```

---

### 2. Launching the Interactive 3D Visual Player

You can launch the full desktop CAD workbench from Python:

```python
import pyton3d as p3d

# Launch a clean interactive workbench
p3d.launch_studio()

# Or pass your custom-built world directly to the visual player:
# p3d.launch_studio(world)
```

Or directly from your terminal:
```bash
pyton3d
```

---

## 3D Vector & Kinematics API

Pyton3D provides an optimized, standalone spatial math core:

```python
import math
import pyton3d as p3d

# 3D Vector Calculus
v1 = p3d.Vec3(1.0, 2.0, 3.0)
v2 = p3d.Vec3(4.0, 5.0, 6.0)
cross = v1.cross(v2)       # Vec3(-3.0, 6.0, -3.0)
dot = v1.dot(v2)           # 32.0

# Rodrigues Quaternion Vector Rotation
q = p3d.Quaternion.from_axis_angle(p3d.Vec3(0, 1, 0), math.radians(90))
rotated_v = q.rotate_vector(p3d.Vec3(1, 0, 0)) # Rotates in 15 FLOPs
```

---

## 3D Collision Detection API (SAT)

Perform geometric intersection queries programmatically:

```python
import pyton3d as p3d

box_a = p3d.RigidBody(position=p3d.Vec3(0, 0, 0))
box_a.collider = p3d.BoxCollider(p3d.Vec3(1, 1, 1))

box_b = p3d.RigidBody(position=p3d.Vec3(1.5, 0, 0))
box_b.collider = p3d.SphereCollider(radius=1.0)

# Check for collision across all 15 3D separating axes
contact = p3d.CollisionDetector.detect(box_a.collider, box_b.collider)
if contact:
    print(f"Collision Point: {contact.point}")
    print(f"Normal: {contact.normal}")
    print(f"Penetration Depth: {contact.penetration:.4f} m")
```

---

## Running Examples & Unit Tests

### Runnable Developer Examples
```bash
# Example 1: Basic Drop
python examples/01_quickstart.py

# Example 2: Damped Harmonic Spring Chain
python examples/02_spring_pendulum.py

# Example 3: Integrator Accuracy & Energy Conservation Benchmark
python examples/03_integrator_benchmark.py
```

### Running Automated Test Suite
```bash
pytest tests/ -v
```

---

## Documentation
- **Developer & User Manual**: [USER_GUIDE.md](USER_GUIDE.md) — Comprehensive guide covering every vector method, collision query, constraint, force field, and visual option.
- **Mathematical & Physics Derivations**: [DOCUMENTATION.md](DOCUMENTATION.md) — First-principles formulation of 6-DOF dynamics, quaternion Rodrigues rotation, SAT manifold projections, Coulomb friction clamping, and Baumgarte stabilization.

---

## Project Structure

```
pyton3d_package/
├── pyproject.toml              # Build backend and PyPI metadata
├── requirements.txt            # Minimal runtime requirements (numpy, matplotlib)
├── README.md                   # Package overview and getting started
├── USER_GUIDE.md               # Detailed developer manual and code recipes
├── DOCUMENTATION.md            # Mathematical derivations from first principles
├── LICENSE                     # MIT License
│
├── dist/                       # Built distribution packages (.whl and .tar.gz)
│   ├── pyton3d-0.1.0-py3-none-any.whl
│   └── pyton3d-0.1.0.tar.gz
│
├── pyton3d/                    # Core package source
│   ├── __init__.py             # Public API exports and physics engine core
│   └── __main__.py             # CLI runner (`python -m pyton3d`)
│
├── examples/                   # Runnable code samples for developers
│   ├── 01_quickstart.py
│   ├── 02_spring_pendulum.py
│   └── 03_integrator_benchmark.py
│
└── tests/                      # Automated unit test suite
    ├── test_math.py
    ├── test_collision.py
    └── test_dynamics.py
```

---

## License

Distributed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

## Author

**Aditya Raj**  
Indian Institute of Technology Jodhpur (IIT Jodhpur)  
GitHub: [@adityarajIITj](https://github.com/adityarajIITj)
