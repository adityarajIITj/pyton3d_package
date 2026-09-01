# Implementation Plan: Pyton3D Open-Source Python Package Architecture

Modularize and package **Pyton3D** from a single-file script into a production-grade, open-source Python library and CLI tool ready for distribution on PyPI (Python Package Index) and GitHub.

---

## User Review Required

> [!IMPORTANT]
> **Destination Directory**: The package will be created in a new dedicated directory specified by the user (or default `C:\Users\adity\OneDrive\文档\pyton3d_package`) to keep the existing single-file repository safe and untouched.

> [!NOTE]
> **Zero External Physics Dependencies**: The package retains 100% pure Python implementation from scratch with only standard scientific runtime dependencies (`numpy` and `matplotlib`).

---

## Proposed Package Architecture

### Directory Layout

```
pyton3d_package/
├── pyproject.toml                     # PEP 621 / PEP 518 standard build configuration
├── requirements.txt                   # Minimal runtime dependencies
├── README.md                          # Package documentation for PyPI and GitHub
├── DOCUMENTATION.md                   # First-principles physics and mathematics guide
├── LICENSE                            # MIT License
├── .gitignore                         # Python, IDE, and build artifact exclusions
│
├── .github/
│   └── workflows/
│       └── ci.yml                     # GitHub Actions CI matrix (Python 3.9 - 3.14 on Linux/macOS/Windows)
│
├── src/
│   └── pyton3d/
│       ├── __init__.py                # Clean top-level public API exports
│       ├── __main__.py                # Enables `python -m pyton3d` execution
│       ├── cli.py                     # Command-line interface (`pyton3d [demo] [--headless]`)
│       │
│       ├── core/                      # Foundational Mathematics
│       │   ├── __init__.py
│       │   ├── vec3.py                # 3D Vector operations (dot, cross, norm, lerp)
│       │   ├── mat3.py                # 3x3 Matrix, inverse, determinants, rotations
│       │   ├── mat4.py                # 4x4 Matrix transformations
│       │   ├── quaternion.py          # Quaternions with Rodrigues vector rotation
│       │   └── geometry.py            # AABB, OBB, Ray, Plane primitives
│       │
│       ├── collision/                 # Collision Detection & Contact Manifolds
│       │   ├── __init__.py
│       │   ├── collider.py            # Base Collider, BoxCollider, SphereCollider, PlaneCollider
│       │   ├── detector.py            # Separating Axis Theorem (SAT) and Contact extraction
│       │   └── material.py            # Physical materials (density, restitution, friction)
│       │
│       ├── dynamics/                  # 6-DOF Dynamics & Numerical Solvers
│       │   ├── __init__.py
│       │   ├── rigidbody.py           # 6-DOF RigidBody state, sleep/wake management
│       │   ├── world.py               # PhysicsWorld simulation coordinator
│       │   └── integrators.py         # Symplectic Euler, Velocity Verlet, RK4, Euler
│       │
│       ├── constraints/               # Constraint Mechanics
│       │   ├── __init__.py
│       │   ├── base.py                # Base Constraint interface
│       │   ├── spring.py              # Damped Hooke's Law SpringConstraint
│       │   ├── distance.py            # Inelastic DistanceConstraint
│       │   └── hinge.py               # Revolute HingeConstraint
│       │
│       ├── forces/                    # Environmental & Field Forces
│       │   ├── __init__.py
│       │   ├── base.py                # Base ForceGenerator interface
│       │   ├── gravity.py             # Planetary Gravity fields
│       │   ├── drag.py                # Linear & Quadratic aerodynamic drag
│       │   └── buoyancy.py            # Archimedes fluid buoyancy
│       │
│       ├── studio/                    # Interactive CAD Desktop Application
│       │   ├── __init__.py
│       │   ├── app.py                 # Tkinter PhysicsStudioApp main window
│       │   ├── renderer.py            # Embedded Matplotlib 3D Viewport with Toolbar
│       │   ├── dialogs.py             # Object Spawner, Gravity, and Collision Windows
│       │   └── scene.py               # JSON Scene Serialization & Deserialization
│       │
│       └── demos/                     # Pre-built Classical Physics Labs
│           ├── __init__.py
│           └── labs.py                # 7 Classical Labs (Stack, Jenga, Cradle, etc.)
│
├── examples/                          # Standalone developer code samples
│   ├── 01_quickstart.py               # 10-line basic physics drop
│   ├── 02_spring_pendulum.py          # Multi-link spring chain
│   ├── 03_integrator_benchmark.py     # Accuracy comparison between Euler vs Verlet vs RK4
│   └── 04_headless_data_logger.py     # Headless data export to CSV/JSON
│
└── tests/                             # Automated unit testing suite
    ├── __init__.py
    ├── test_math.py                   # Vector, Matrix, and Quaternion algebraic correctness
    ├── test_collision.py              # SAT box-box, sphere-box, plane-sphere intersection tests
    ├── test_dynamics.py               # Momentum and kinetic energy conservation tests
    └── test_serialization.py          # JSON save and load round-trip validation
```

---

## Key Technical Specifications

### 1. Top-Level Public API (`pyton3d/__init__.py`)
Allows clean and standard Python imports:
```python
import pyton3d as p3d

# Core objects directly accessible
world = p3d.PhysicsWorld(gravity=p3d.Vec3(0, -9.81, 0))
body = p3d.RigidBody(position=p3d.Vec3(0, 5, 0), mass=2.0)
body.collider = p3d.BoxCollider(p3d.Vec3(0.5, 0.5, 0.5))
body.material = p3d.Materials.WOOD
world.add_body(body)

# Run simulation
world.step(dt=1/60)

# Or launch studio
p3d.launch_studio()
```

### 2. Command-Line Interface (`pyton3d/cli.py`)
Configures a console entry point in `pyproject.toml` so users can execute from terminal:
- `pyton3d` -> Launches interactive CAD Studio.
- `pyton3d stack` / `pyton3d jenga` -> Directly opens specific lab presets.
- `pyton3d --test` -> Runs engine self-diagnostic suite.

### 3. Build & Packaging Configuration (`pyproject.toml`)
- Uses `setuptools` build backend.
- Declares metadata: name, version (`0.1.0`), description, author, license (`MIT`), keywords, classifiers.
- Specifies runtime dependencies (`numpy>=1.20.0`, `matplotlib>=3.5.0`) and test dependencies (`pytest>=7.0.0`).

---

## Verification Plan

### Automated Verification
1. **Local Editable Installation**:
   ```powershell
   pip install -e .
   ```
2. **Execute Full Test Suite**:
   ```powershell
   pytest tests/ -v
   ```
   - Vector operations (addition, cross, dot, norm, lerp).
   - Quaternion rotation accuracy against Rodrigues formula.
   - SAT collision penetration depths and normal vectors.
   - Energy conservation in closed harmonic systems.
   - Scene JSON serialization round-trip.
3. **Validate Standalone Examples**:
   ```powershell
   python examples/01_quickstart.py
   python examples/03_integrator_benchmark.py
   ```
4. **CLI Launch Verification**:
   ```powershell
   pyton3d --help
   ```

### Manual Verification
- Launch the interactive CAD studio via `python -m pyton3d` and verify 3D viewport, toolbar navigation, and dedicated dialogs (spawner, gravity, collision matrix).
