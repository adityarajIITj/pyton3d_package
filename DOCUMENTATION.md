# Pyton3D Technical Documentation

This document provides a comprehensive technical overview of the mathematics, algorithms, and architecture implemented in **Pyton3D**, developed from first principles in pure Python.

---

## Table of Contents
1. [Core Mathematics and Kinematics](#1-core-mathematics-and-kinematics)
2. [Rigid Body Dynamics (6-DOF)](#2-rigid-body-dynamics-6-dof)
3. [Collision Detection Architecture](#3-collision-detection-architecture)
4. [Impulse Resolution and Friction Formulation](#4-impulse-resolution-and-friction-formulation)
5. [Numerical Integration Methods](#5-numerical-integration-methods)
6. [Constraints and Force Generators](#6-constraints-and-force-generators)
7. [Scene Serialization Specification](#7-scene-serialization-specification)

---

## 1. Core Mathematics and Kinematics

### 1.1 Vector3 (`Vec3`)
Points, linear velocities, accelerations, and forces in 3D Euclidean space are represented as:
$$\vec{v} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}$$

Standard vector operations implemented:
- **Dot Product**: $\vec{a} \cdot \vec{b} = a_x b_x + a_y b_y + a_z b_z$
- **Cross Product**: $\vec{a} \times \vec{b} = \begin{bmatrix} a_y b_z - a_z b_y \\ a_z b_x - a_x b_z \\ a_x b_y - a_y b_x \end{bmatrix}$
- **Euclidean Norm**: $\|\vec{v}\| = \sqrt{x^2 + y^2 + z^2}$
- **Unit Normalization**: $\hat{v} = \frac{\vec{v}}{\|\vec{v}\|}$

### 1.2 Quaternions (`Quaternion`) and Rodrigues Rotation
Orientations are parameterized using unit quaternions:
$$\mathbf{q} = (x, y, z, w) = (\vec{q}_v, q_w), \quad \|\mathbf{q}\| = 1$$

#### Optimized Rodrigues Vector Rotation
To rotate an arbitrary vector $\vec{v}$ by quaternion $\mathbf{q}$ without constructing a full rotation matrix:
$$\vec{v}\' = \vec{v} + 2 q_w (\vec{q}_v \times \vec{v}) + 2 (\vec{q}_v \times (\vec{q}_v \times \vec{v}))$$

This formulation executes in **15 floating-point operations**, compared to 32 FLOPs required for full quaternion sandwich multiplication $\mathbf{q} \mathbf{v} \mathbf{q}^*$.

### 1.3 Inertia Tensors (`Mat3`)
Rigid body mass distributions are represented by symmetric $3 \times 3$ inertia tensors $\mathbf{I}$.

For an Oriented Bounding Box with half-extents $(h_x, h_y, h_z)$ and mass $m$:
$$\mathbf{I}_{box} = \begin{bmatrix} \frac{1}{3} m (h_y^2 + h_z^2) & 0 & 0 \\ 0 & \frac{1}{3} m (h_x^2 + h_z^2) & 0 \\ 0 & 0 & \frac{1}{3} m (h_x^2 + h_y^2) \end{bmatrix}$$

For a uniform solid sphere of radius $r$ and mass $m$:
$$\mathbf{I}_{sphere} = \begin{bmatrix} \frac{2}{5} m r^2 & 0 & 0 \\ 0 & \frac{2}{5} m r^2 & 0 \\ 0 & 0 & \frac{2}{5} m r^2 \end{bmatrix}$$

The world-space inverse inertia tensor is transformed each frame using the current rotation matrix $\mathbf{R}$:
$$\mathbf{I}_{world}^{-1} = \mathbf{R} \mathbf{I}_{local}^{-1} \mathbf{R}^T$$

---

## 2. Rigid Body Dynamics (6-DOF)

Each rigid body tracks linear and angular states:
$$\mathbf{S} = \{ \vec{x}, \vec{v}, \vec{a}, \mathbf{q}, \vec{\omega}, \vec{\tau}, m, \mathbf{I}^{-1} \}$$

The velocity of an arbitrary world point $\vec{p}$ attached to the body is given by:
$$\vec{v}_p = \vec{v} + \vec{\omega} \times (\vec{p} - \vec{x})$$

When an impulse $\vec{J}$ is applied at point $\vec{p}$:
$$\Delta \vec{v} = \frac{\vec{J}}{m}$$
$$\Delta \vec{\omega} = \mathbf{I}_{world}^{-1} ((\vec{p} - \vec{x}) \times \vec{J})$$

---

## 3. Collision Detection Architecture

### 3.1 Broad-Phase: AABB Pruning
Axis-Aligned Bounding Box (AABB) intersection tests reject non-overlapping pairs in $O(1)$ time per pair:
$$\text{Overlap} = (\min_A \le \max_B) \land (\max_A \ge \min_B) \quad \forall \, x,y,z$$

### 3.2 Narrow-Phase: Separating Axis Theorem (SAT)
For two Oriented Bounding Boxes $A$ and $B$, the existence of a separating plane is evaluated across **15 candidate axes**:
1. 3 face normal axes of Box A: $\vec{u}_{A0}, \vec{u}_{A1}, \vec{u}_{A2}$
2. 3 face normal axes of Box B: $\vec{u}_{B0}, \vec{u}_{B1}, \vec{u}_{B2}$
3. 9 cross-product edge axes: $\vec{u}_{Ai} \times \vec{u}_{Bj} \quad (i, j \in \{0, 1, 2\})$

For each candidate unit axis $\hat{L}$:
$$r_A = \sum_{i=0}^2 h_{A,i} |\vec{u}_{Ai} \cdot \hat{L}|, \quad r_B = \sum_{j=0}^2 h_{B,j} |\vec{u}_{Bj} \cdot \hat{L}|$$
$$d = |(\vec{x}_B - \vec{x}_A) \cdot \hat{L}|$$

If $d > r_A + r_B$, a separating axis is found and no collision occurs. Otherwise, penetration depth is $p = (r_A + r_B) - d$. The axis yielding the minimum penetration depth is selected as the collision normal.

---

## 4. Impulse Resolution and Friction Formulation

### 4.1 Relative Contact Velocity
At contact point $\vec{p}$ between bodies $A$ and $B$:
$$\vec{v}_{rel} = \vec{v}_B(\vec{p}) - \vec{v}_A(\vec{p})$$
$$v_n = \vec{v}_{rel} \cdot \hat{n}$$

### 4.2 Effective Contact Mass
$$K_n = \frac{1}{m_A} + \frac{1}{m_B} + \left[ (\mathbf{I}_A^{-1} (\vec{r}_A \times \hat{n})) \times \vec{r}_A + (\mathbf{I}_B^{-1} (\vec{r}_B \times \hat{n})) \times \vec{r}_B \right] \cdot \hat{n}$$
$$m_{eff} = \frac{1}{K_n}$$

### 4.3 Normal Impulse with Restitution
$$\Delta j_n = -(1 + e) v_n m_{eff}$$
$$j_{n, new} = \max(j_{n, old} + \Delta j_n, 0)$$

### 4.4 Dual-Axis Coulomb Friction Formulation
Two orthogonal tangent vectors $\hat{t}_1, \hat{t}_2$ are constructed perpendicular to contact normal $\hat{n}$. The frictional impulse along each tangent direction $k$ is clamped to the Coulomb friction limit:
$$|j_{t,k}| \le \mu j_n$$

### 4.5 Baumgarte Stabilization
To eliminate penetration drift without introducing artificial kinetic energy:
$$\vec{C}_{pos} = \hat{n} \cdot \frac{\max(p - \text{slop}, 0)}{m_A^{-1} + m_B^{-1}} \cdot \beta$$
where $\beta = 0.4$ and $\text{slop} = 0.01\text{ m}$.

---

## 5. Numerical Integration Methods

| Method | Order | Symplectic | Formulation |
| :--- | :---: | :---: | :--- |
| **Explicit Euler** | 1st | No | $\vec{x}_{t+1} = \vec{x}_t + \vec{v}_t \Delta t$, $\vec{v}_{t+1} = \vec{v}_t + \vec{a}_t \Delta t$ |
| **Symplectic Euler** | 1st | Yes | $\vec{v}_{t+1} = \vec{v}_t + \vec{a}_t \Delta t$, $\vec{x}_{t+1} = \vec{x}_t + \vec{v}_{t+1} \Delta t$ |
| **Velocity Verlet** | 2nd | Yes | $\vec{x}_{t+1} = 2\vec{x}_t - \vec{x}_{t-1} + \vec{a}_t \Delta t^2$ |
| **Runge-Kutta 4** | 4th | No | 4-stage derivative evaluation |

---

## 6. Constraints and Force Generators

### 6.1 Damped Hooke\'s Law Spring
$$F_{spring} = -k (\|\vec{x}_B - \vec{x}_A\| - L_0) \hat{n} - c (\vec{v}_{rel} \cdot \hat{n}) \hat{n}$$

### 6.2 Archimedes Fluid Buoyancy
For a body submerged to depth $d$ below liquid level $y_{water}$:
$$F_{buoyant} = \rho_{fluid} \cdot V_{submerged} \cdot g$$

---

## 7. Scene Serialization Specification

Scenes in Pyton3D are serialized to standard JSON:

```json
{
  "gravity": [0.0, -9.81, 0.0],
  "iterations": 8,
  "time_scale": 1.0,
  "bodies": [
    {
      "name": "Wood_Box_1",
      "is_static": false,
      "mass": 2.0,
      "position": [0.0, 3.5, 0.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "velocity": [0.0, 0.0, 0.0],
      "material": {
        "name": "Wood",
        "density": 0.7,
        "restitution": 0.3,
        "static_friction": 0.5,
        "dynamic_friction": 0.3,
        "color": [0.78, 0.55, 0.35]
      },
      "collider": {
        "type": "box",
        "half_extents": [0.5, 0.5, 0.5]
      }
    }
  ]
}
```
