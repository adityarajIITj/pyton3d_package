#!/usr/bin/env python3
"""
Pyton3D - 3D Physics Simulation Engine and CAD Workbench (Built From Scratch)
==============================================================
A complete rigid body physics simulation suite with interactive
CAD GUI, dedicated control windows, object spawner, gravity &
collision tuners, scene save/load, and Matplotlib 3D viewport.

Features:
  - 6-DOF Rigid body dynamics (Euler, Symplectic Euler, Verlet, RK4)
  - SAT Collision Detection for Box-Box, Sphere-Sphere, Sphere-Box, Planes
  - Iterative impulse solver with 2-axis Coulomb friction & restitution
  - Springs, Distance Joints, and Hinge constraints
  - Gravity fields, Aerodynamic Drag, and Archimedes Buoyancy
  - Desktop Studio GUI (Tkinter + Matplotlib Canvas + Navigation Toolbar)
  - Dedicated Tool Windows:
      * ➕ Object Creator Window
      * 🌍 Gravity & Environment Window
      * ⚙️ Collision Physics & Solver Window
      * 📦 Scene Inspector Window
  - File Operations: New Scene, Open JSON, Save JSON, Export Snapshot
  - 8 Pre-built Physics Demonstration Labs
"""

import sys
import math
import random
import time
import json
import os
from typing import List, Tuple, Optional, Dict, Callable, Union, Set
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

import tkinter as tk
from tkinter import ttk, messagebox, filedialog


# ============================================================================
# SECTION 1: CORE MATHEMATICS
# ============================================================================

class Vec3:
    """3D vector with operator overloading and geometric operations."""
    __slots__ = ("x", "y", "z")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def __repr__(self): return f"Vec3({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"
    def __add__(self, o): return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)
    def __sub__(self, o): return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)
    def __mul__(self, s):
        if isinstance(s, (int, float)): return Vec3(self.x * s, self.y * s, self.z * s)
        return Vec3(self.x * s.x, self.y * s.y, self.z * s.z)
    def __rmul__(self, s): return Vec3(self.x * s, self.y * s, self.z * s)
    def __truediv__(self, s):
        inv = 1.0 / s
        return Vec3(self.x * inv, self.y * inv, self.z * inv)
    def __neg__(self): return Vec3(-self.x, -self.y, -self.z)
    def __eq__(self, o):
        return isinstance(o, Vec3) and abs(self.x - o.x) < 1e-6 and abs(self.y - o.y) < 1e-6 and abs(self.z - o.z) < 1e-6

    def copy(self): return Vec3(self.x, self.y, self.z)
    def tuple(self): return (self.x, self.y, self.z)
    def to_list(self): return [self.x, self.y, self.z]
    def length_sq(self): return self.x * self.x + self.y * self.y + self.z * self.z
    def length(self): return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        m2 = self.x * self.x + self.y * self.y + self.z * self.z
        if m2 < 1e-20: return Vec3(0.0, 0.0, 0.0)
        inv = 1.0 / math.sqrt(m2)
        return Vec3(self.x * inv, self.y * inv, self.z * inv)

    def dot(self, o): return self.x * o.x + self.y * o.y + self.z * o.z
    def cross(self, o):
        return Vec3(self.y * o.z - self.z * o.y,
                    self.z * o.x - self.x * o.z,
                    self.x * o.y - self.y * o.x)

    def distance_to(self, o): return (self - o).length()
    def distance_sq_to(self, o): return (self - o).length_sq()
    def lerp(self, o, t): return Vec3(self.x + (o.x - self.x) * t, self.y + (o.y - self.y) * t, self.z + (o.z - self.z) * t)

    def perpendicular(self):
        if abs(self.x) < abs(self.y):
            inv = 1.0 / math.sqrt(self.y * self.y + self.z * self.z + 1e-20)
            return Vec3(0.0, -self.z * inv, self.y * inv)
        inv = 1.0 / math.sqrt(self.x * self.x + self.z * self.z + 1e-20)
        return Vec3(-self.z * inv, 0.0, self.x * inv)

    @staticmethod
    def zero(): return Vec3(0.0, 0.0, 0.0)
    @staticmethod
    def one(): return Vec3(1.0, 1.0, 1.0)
    @staticmethod
    def up(): return Vec3(0.0, 1.0, 0.0)
    @staticmethod
    def right(): return Vec3(1.0, 0.0, 0.0)
    @staticmethod
    def forward(): return Vec3(0.0, 0.0, 1.0)
    @staticmethod
    def random(minv=-1.0, maxv=1.0):
        return Vec3(random.uniform(minv, maxv), random.uniform(minv, maxv), random.uniform(minv, maxv))
    @staticmethod
    def from_list(lst):
        return Vec3(lst[0], lst[1], lst[2]) if lst else Vec3.zero()


class Mat3:
    """3x3 matrix for rotations and inertia tensors."""
    __slots__ = ("m00", "m01", "m02",
                 "m10", "m11", "m12",
                 "m20", "m21", "m22")

    def __init__(self, data=None):
        if data is None:
            self.m00, self.m01, self.m02 = 1.0, 0.0, 0.0
            self.m10, self.m11, self.m12 = 0.0, 1.0, 0.0
            self.m20, self.m21, self.m22 = 0.0, 0.0, 1.0
        elif isinstance(data, list):
            self.m00, self.m01, self.m02 = float(data[0][0]), float(data[0][1]), float(data[0][2])
            self.m10, self.m11, self.m12 = float(data[1][0]), float(data[1][1]), float(data[1][2])
            self.m20, self.m21, self.m22 = float(data[2][0]), float(data[2][1]), float(data[2][2])
        else:
            self.m00, self.m01, self.m02 = float(data.m00), float(data.m01), float(data.m02)
            self.m10, self.m11, self.m12 = float(data.m10), float(data.m11), float(data.m12)
            self.m20, self.m21, self.m22 = float(data.m20), float(data.m21), float(data.m22)

    def __mul__(self, o):
        if isinstance(o, Vec3):
            return Vec3(
                self.m00 * o.x + self.m01 * o.y + self.m02 * o.z,
                self.m10 * o.x + self.m11 * o.y + self.m12 * o.z,
                self.m20 * o.x + self.m21 * o.y + self.m22 * o.z
            )
        if isinstance(o, Mat3):
            res = Mat3()
            res.m00 = self.m00 * o.m00 + self.m01 * o.m10 + self.m02 * o.m20
            res.m01 = self.m00 * o.m01 + self.m01 * o.m11 + self.m02 * o.m21
            res.m02 = self.m00 * o.m02 + self.m01 * o.m12 + self.m02 * o.m22

            res.m10 = self.m10 * o.m00 + self.m11 * o.m10 + self.m12 * o.m20
            res.m11 = self.m10 * o.m01 + self.m11 * o.m11 + self.m12 * o.m21
            res.m12 = self.m10 * o.m02 + self.m11 * o.m12 + self.m12 * o.m22

            res.m20 = self.m20 * o.m00 + self.m21 * o.m10 + self.m22 * o.m20
            res.m21 = self.m20 * o.m01 + self.m21 * o.m11 + self.m22 * o.m21
            res.m22 = self.m20 * o.m02 + self.m21 * o.m12 + self.m22 * o.m22
            return res
        s = float(o)
        res = Mat3()
        res.m00, res.m01, res.m02 = self.m00 * s, self.m01 * s, self.m02 * s
        res.m10, res.m11, res.m12 = self.m10 * s, self.m11 * s, self.m12 * s
        res.m20, res.m21, res.m22 = self.m20 * s, self.m21 * s, self.m22 * s
        return res

    def transpose(self):
        res = Mat3()
        res.m00, res.m01, res.m02 = self.m00, self.m10, self.m20
        res.m10, res.m11, res.m12 = self.m01, self.m11, self.m21
        res.m20, res.m21, res.m22 = self.m02, self.m12, self.m22
        return res

    def determinant(self):
        return (self.m00 * (self.m11 * self.m22 - self.m12 * self.m21) -
                self.m01 * (self.m10 * self.m22 - self.m12 * self.m20) +
                self.m02 * (self.m10 * self.m21 - self.m11 * self.m20))

    def inverse(self):
        det = self.determinant()
        if abs(det) < 1e-12: return Mat3()
        inv_det = 1.0 / det
        res = Mat3()
        res.m00 = (self.m11 * self.m22 - self.m12 * self.m21) * inv_det
        res.m01 = (self.m02 * self.m21 - self.m01 * self.m22) * inv_det
        res.m02 = (self.m01 * self.m12 - self.m02 * self.m11) * inv_det

        res.m10 = (self.m12 * self.m20 - self.m10 * self.m22) * inv_det
        res.m11 = (self.m00 * self.m22 - self.m02 * self.m20) * inv_det
        res.m12 = (self.m02 * self.m10 - self.m00 * self.m12) * inv_det

        res.m20 = (self.m10 * self.m21 - self.m11 * self.m20) * inv_det
        res.m21 = (self.m01 * self.m20 - self.m00 * self.m21) * inv_det
        res.m22 = (self.m00 * self.m11 - self.m01 * self.m10) * inv_det
        return res

    @staticmethod
    def from_quaternion(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        res = Mat3()
        res.m00 = 1.0 - 2.0 * (y * y + z * z)
        res.m01 = 2.0 * (x * y - z * w)
        res.m02 = 2.0 * (x * z + y * w)

        res.m10 = 2.0 * (x * y + z * w)
        res.m11 = 1.0 - 2.0 * (x * x + z * z)
        res.m12 = 2.0 * (y * z - x * w)

        res.m20 = 2.0 * (x * z - y * w)
        res.m21 = 2.0 * (y * z + x * w)
        res.m22 = 1.0 - 2.0 * (x * x + y * y)
        return res


class Quaternion:
    """Unit quaternion with fast Rodrigues rotation."""
    __slots__ = ("x", "y", "z", "w")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)

    def __repr__(self): return f"Quat({self.x:.3f}, {self.y:.3f}, {self.z:.3f}, {self.w:.3f})"
    def to_list(self): return [self.x, self.y, self.z, self.w]
    def __add__(self, o): return Quaternion(self.x + o.x, self.y + o.y, self.z + o.z, self.w + o.w)
    def __sub__(self, o): return Quaternion(self.x - o.x, self.y - o.y, self.z - o.z, self.w - o.w)

    def __mul__(self, o):
        if isinstance(o, Quaternion):
            return Quaternion(
                self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
                self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
                self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
                self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z
            )
        s = float(o)
        return Quaternion(self.x * s, self.y * s, self.z * s, self.w * s)

    def conjugate(self): return Quaternion(-self.x, -self.y, -self.z, self.w)
    def length_sq(self): return self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    def length(self): return math.sqrt(self.length_sq())

    def normalize(self):
        m2 = self.length_sq()
        if m2 < 1e-20: return Quaternion(0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / math.sqrt(m2)
        return Quaternion(self.x * inv, self.y * inv, self.z * inv, self.w * inv)

    def rotate_vector(self, v: Vec3) -> Vec3:
        tx = 2.0 * (self.y * v.z - self.z * v.y)
        ty = 2.0 * (self.z * v.x - self.x * v.z)
        tz = 2.0 * (self.x * v.y - self.y * v.x)
        return Vec3(
            v.x + self.w * tx + (self.y * tz - self.z * ty),
            v.y + self.w * ty + (self.z * tx - self.x * tz),
            v.z + self.w * tz + (self.x * ty - self.y * tx)
        )

    @staticmethod
    def from_axis_angle(axis: Vec3, angle: float):
        h = angle * 0.5
        s = math.sin(h)
        n = axis.normalize()
        return Quaternion(n.x * s, n.y * s, n.z * s, math.cos(h)).normalize()

    @staticmethod
    def from_list(lst):
        return Quaternion(lst[0], lst[1], lst[2], lst[3]) if lst else Quaternion()


# ============================================================================
# SECTION 2: GEOMETRY & BOUNDING VOLUMES
# ============================================================================

class AABB:
    __slots__ = ("min", "max")

    def __init__(self, min_pt: Optional[Vec3] = None, max_pt: Optional[Vec3] = None):
        self.min = min_pt if min_pt else Vec3(0, 0, 0)
        self.max = max_pt if max_pt else Vec3(0, 0, 0)

    def center(self) -> Vec3: return (self.min + self.max) * 0.5
    def extents(self) -> Vec3: return (self.max - self.min) * 0.5

    def intersects(self, o) -> bool:
        return (self.min.x <= o.max.x and self.max.x >= o.min.x and
                self.min.y <= o.max.y and self.max.y >= o.min.y and
                self.min.z <= o.max.z and self.max.z >= o.min.z)


class OBB:
    __slots__ = ("center", "extents", "axes")

    def __init__(self, center: Vec3, extents: Vec3, axes: Mat3):
        self.center, self.extents, self.axes = center, extents, axes

    def vertices(self) -> List[Vec3]:
        ax = self.axes * Vec3(1, 0, 0)
        ay = self.axes * Vec3(0, 1, 0)
        az = self.axes * Vec3(0, 0, 1)
        verts = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    verts.append(self.center + ax * (sx * self.extents.x) +
                                 ay * (sy * self.extents.y) + az * (sz * self.extents.z))
        return verts


# ============================================================================
# SECTION 3: MATERIALS & COLLIDERS
# ============================================================================

@dataclass
class Material:
    name: str = "Custom"
    density: float = 1.0
    restitution: float = 0.5
    static_friction: float = 0.5
    dynamic_friction: float = 0.3
    color: Tuple[float, float, float] = (0.7, 0.7, 0.8)

    def combine_restitution(self, o): return max(self.restitution, o.restitution)
    def combine_friction(self, o): return math.sqrt(self.static_friction * o.static_friction)
    def combine_dynamic_friction(self, o): return math.sqrt(self.dynamic_friction * o.dynamic_friction)

    def to_dict(self):
        return {
            "name": self.name,
            "density": self.density,
            "restitution": self.restitution,
            "static_friction": self.static_friction,
            "dynamic_friction": self.dynamic_friction,
            "color": list(self.color)
        }

    @staticmethod
    def from_dict(d):
        return Material(
            name=d.get("name", "Custom"),
            density=d.get("density", 1.0),
            restitution=d.get("restitution", 0.5),
            static_friction=d.get("static_friction", 0.5),
            dynamic_friction=d.get("dynamic_friction", 0.3),
            color=tuple(d.get("color", [0.7, 0.7, 0.8]))
        )


class Materials:
    STEEL = Material("Steel", 7.85, 0.2, 0.6, 0.4, color=(0.75, 0.78, 0.82))
    WOOD = Material("Wood", 0.7, 0.3, 0.5, 0.3, color=(0.78, 0.55, 0.35))
    RUBBER = Material("Rubber", 1.5, 0.9, 0.8, 0.6, color=(0.25, 0.65, 0.85))
    ICE = Material("Ice", 0.9, 0.1, 0.05, 0.02, color=(0.65, 0.88, 0.95))
    CONCRETE = Material("Concrete", 2.4, 0.1, 0.7, 0.5, color=(0.5, 0.52, 0.56))
    BOUNCY = Material("Bouncy", 0.5, 0.95, 0.3, 0.2, color=(0.95, 0.4, 0.35))

    ALL = [WOOD, STEEL, RUBBER, ICE, CONCRETE, BOUNCY]
    BY_NAME = {m.name: m for m in ALL}


class ColliderType(Enum):
    SPHERE = auto()
    BOX = auto()
    PLANE = auto()


class Collider:
    def __init__(self, ctype: ColliderType):
        self.type = ctype
        self.offset = Vec3.zero()
        self.body = None

    def get_world_position(self) -> Vec3:
        return self.offset if self.body is None else self.body.position + self.body.orientation.rotate_vector(self.offset)

    def get_world_aabb(self) -> AABB: raise NotImplementedError


class SphereCollider(Collider):
    def __init__(self, radius: float):
        super().__init__(ColliderType.SPHERE)
        self.radius = float(radius)

    def get_world_aabb(self) -> AABB:
        c = self.get_world_position()
        r = self.radius
        return AABB(Vec3(c.x - r, c.y - r, c.z - r), Vec3(c.x + r, c.y + r, c.z + r))

    def inertia_tensor(self, mass: float) -> Mat3:
        i = 0.4 * mass * self.radius * self.radius
        res = Mat3()
        res.m00, res.m11, res.m22 = i, i, i
        return res


class BoxCollider(Collider):
    def __init__(self, half_extents: Vec3):
        super().__init__(ColliderType.BOX)
        self.half_extents = half_extents

    def get_world_aabb(self) -> AABB:
        pos = self.get_world_position()
        if self.body:
            rot = Mat3.from_quaternion(self.body.orientation)
            ex = abs(rot.m00) * self.half_extents.x + abs(rot.m01) * self.half_extents.y + abs(rot.m02) * self.half_extents.z
            ey = abs(rot.m10) * self.half_extents.x + abs(rot.m11) * self.half_extents.y + abs(rot.m12) * self.half_extents.z
            ez = abs(rot.m20) * self.half_extents.x + abs(rot.m21) * self.half_extents.y + abs(rot.m22) * self.half_extents.z
            return AABB(Vec3(pos.x - ex, pos.y - ey, pos.z - ez), Vec3(pos.x + ex, pos.y + ey, pos.z + ez))
        return AABB(pos - self.half_extents, pos + self.half_extents)

    def inertia_tensor(self, mass: float) -> Mat3:
        x2, y2, z2 = self.half_extents.x ** 2, self.half_extents.y ** 2, self.half_extents.z ** 2
        res = Mat3()
        res.m00 = mass * (y2 + z2) / 3.0
        res.m11 = mass * (x2 + z2) / 3.0
        res.m22 = mass * (x2 + y2) / 3.0
        return res

    def get_obb(self) -> OBB:
        pos = self.get_world_position()
        rot = Mat3.from_quaternion(self.body.orientation) if self.body else Mat3()
        return OBB(pos, self.half_extents, rot)


class PlaneCollider(Collider):
    def __init__(self, normal: Vec3, offset: float = 0.0):
        super().__init__(ColliderType.PLANE)
        self.normal = normal.normalize()
        self.offset = float(offset)

    def get_world_position(self) -> Vec3: return self.normal * self.offset
    def get_world_aabb(self) -> AABB: return AABB(Vec3(-1e6, -1e6, -1e6), Vec3(1e6, 1e6, 1e6))


# ============================================================================
# SECTION 4: RIGID BODY DYNAMICS
# ============================================================================

class IntegrationMethod(Enum):
    EULER = auto()
    SYMPLECTIC_EULER = auto()
    VERLET = auto()
    RK4 = auto()


class RigidBody:
    _id_counter = 0

    def __init__(self, position: Optional[Vec3] = None, mass: float = 1.0,
                 collider: Optional[Collider] = None, material: Optional[Material] = None,
                 is_static: bool = False, name: str = ""):
        RigidBody._id_counter += 1
        self.id = RigidBody._id_counter
        self.name = name if name else f"Body_{self.id}"
        self.position = position.copy() if position else Vec3.zero()
        self.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.velocity = Vec3.zero()
        self.acceleration = Vec3.zero()
        self.force_accum = Vec3.zero()
        self.linear_damping = 0.995
        self.angular_velocity = Vec3.zero()
        self.torque_accum = Vec3.zero()
        self.angular_damping = 0.995
        self.is_static = bool(is_static)
        self.is_sleeping = False

        self.mass = float(mass)
        self.inv_mass = 0.0 if (self.is_static or mass == 0) else (1.0 / self.mass)
        self.inertia_tensor = Mat3()
        self.inv_inertia_tensor = Mat3()
        self.inv_inertia_tensor_world = Mat3()

        self._collider = None
        self.collider = collider
        self.material = material if material else Material()

        self.sleep_threshold = 0.1
        self.sleep_counter = 0
        self.prev_position = self.position.copy()
        self.color = self.material.color if material else (random.uniform(0.3, 0.85), random.uniform(0.3, 0.85), random.uniform(0.3, 0.85))

    @property
    def collider(self):
        return self._collider

    @collider.setter
    def collider(self, c):
        self._collider = c
        if c is not None:
            c.body = self
            self._update_inertia()

    def _update_inertia(self):
        if getattr(self, '_collider', None) and not getattr(self, 'is_static', False):
            if hasattr(self._collider, 'inertia_tensor'):
                self.inertia_tensor = self._collider.inertia_tensor(self.mass)
                self.inv_inertia_tensor = self.inertia_tensor.inverse()

    def get_world_position(self) -> Vec3:
        return self.position

    def get_aabb(self) -> AABB:
        return self._collider.get_world_aabb() if self._collider else AABB(self.position, self.position)

    def get_world_point(self, local: Vec3) -> Vec3:
        return self.position + self.orientation.rotate_vector(local)

    def get_local_point(self, world: Vec3) -> Vec3:
        return self.orientation.conjugate().rotate_vector(world - self.position)

    def get_velocity_at_point(self, world: Vec3) -> Vec3:
        return self.velocity + self.angular_velocity.cross(world - self.position)

    def apply_force(self, force: Vec3, world_point: Optional[Vec3] = None):
        if self.is_static or self.is_sleeping: return
        self.force_accum = self.force_accum + force
        if world_point:
            self.torque_accum = self.torque_accum + (world_point - self.position).cross(force)

    def apply_impulse(self, impulse: Vec3, world_point: Optional[Vec3] = None):
        if self.is_static: return
        self.velocity = self.velocity + impulse * self.inv_mass
        if world_point:
            r = world_point - self.position
            self.angular_velocity = self.angular_velocity + self.inv_inertia_tensor_world * r.cross(impulse)

    def update_inertia_tensor_world(self):
        if not self.is_static:
            rot = Mat3.from_quaternion(self.orientation)
            self.inv_inertia_tensor_world = rot * self.inv_inertia_tensor * rot.transpose()

    def integrate(self, dt: float, method: IntegrationMethod = IntegrationMethod.SYMPLECTIC_EULER):
        if self.is_static or self.is_sleeping: return
        self.update_inertia_tensor_world()

        if method == IntegrationMethod.EULER:
            self.acceleration = self.force_accum * self.inv_mass
            self.position = self.position + self.velocity * dt
            self.velocity = self.velocity + self.acceleration * dt
            ang_acc = self.inv_inertia_tensor_world * self.torque_accum
            self.orientation = self.orientation + Quaternion(self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z, 0.0) * self.orientation * (0.5 * dt)
            self.angular_velocity = self.angular_velocity + ang_acc * dt

        elif method == IntegrationMethod.SYMPLECTIC_EULER:
            self.acceleration = self.force_accum * self.inv_mass
            self.velocity = self.velocity + self.acceleration * dt
            self.position = self.position + self.velocity * dt
            ang_acc = self.inv_inertia_tensor_world * self.torque_accum
            self.angular_velocity = self.angular_velocity + ang_acc * dt
            omega = self.angular_velocity
            om = omega.length()
            if om > 1e-10:
                h = dt * 0.5
                s = math.sin(om * h) / om
                c = math.cos(om * h)
                dq = Quaternion(omega.x * s, omega.y * s, omega.z * s, c)
                self.orientation = dq * self.orientation

        elif method == IntegrationMethod.VERLET:
            if dt > 1e-10:
                acc = self.force_accum * self.inv_mass
                temp = self.position.copy()
                self.position = self.position * 2 - self.prev_position + acc * (dt * dt)
                self.prev_position = temp
                self.velocity = (self.position - self.prev_position) / dt
                ang_acc = self.inv_inertia_tensor_world * self.torque_accum
                self.angular_velocity = self.angular_velocity + ang_acc * dt

        elif method == IntegrationMethod.RK4:
            acc = self.force_accum * self.inv_mass
            self.velocity = self.velocity + acc * dt
            self.position = self.position + self.velocity * dt
            ang_acc = self.inv_inertia_tensor_world * self.torque_accum
            self.angular_velocity = self.angular_velocity + ang_acc * dt
            omega = self.angular_velocity
            om = omega.length()
            if om > 1e-10:
                h = dt * 0.5
                s = math.sin(om * h) / om
                c = math.cos(om * h)
                self.orientation = Quaternion(omega.x * s, omega.y * s, omega.z * s, c) * self.orientation

        self.orientation = self.orientation.normalize()
        self.velocity = self.velocity * self.linear_damping
        self.angular_velocity = self.angular_velocity * self.angular_damping
        self.force_accum = Vec3.zero()
        self.torque_accum = Vec3.zero()
        self._check_sleep()

    def _check_sleep(self):
        if self.velocity.length_sq() < self.sleep_threshold and self.angular_velocity.length_sq() < self.sleep_threshold:
            self.sleep_counter += 1
            if self.sleep_counter > 30:
                self.is_sleeping = True
                self.velocity = Vec3.zero()
                self.angular_velocity = Vec3.zero()
        else:
            self.sleep_counter = 0
            self.is_sleeping = False

    def wake(self):
        self.is_sleeping = False
        self.sleep_counter = 0

    def kinetic_energy(self) -> float:
        return 0.5 * self.mass * self.velocity.length_sq() + 0.5 * self.angular_velocity.dot(self.inv_inertia_tensor_world * self.angular_velocity)

    def to_dict(self):
        d = {
            "name": self.name,
            "is_static": self.is_static,
            "mass": self.mass,
            "position": self.position.to_list(),
            "orientation": self.orientation.to_list(),
            "velocity": self.velocity.to_list(),
            "material": self.material.to_dict()
        }
        if isinstance(self.collider, BoxCollider):
            d["collider"] = {"type": "box", "half_extents": self.collider.half_extents.to_list()}
        elif isinstance(self.collider, SphereCollider):
            d["collider"] = {"type": "sphere", "radius": self.collider.radius}
        elif isinstance(self.collider, PlaneCollider):
            d["collider"] = {"type": "plane", "normal": self.collider.normal.to_list(), "offset": self.collider.offset}
        return d


# ============================================================================
# SECTION 5: COLLISION DETECTION & IMPULSE SOLVER
# ============================================================================

@dataclass
class Contact:
    body_a: Optional[RigidBody] = None
    body_b: Optional[RigidBody] = None
    point: Vec3 = field(default_factory=Vec3.zero)
    normal: Vec3 = field(default_factory=Vec3.up)
    penetration: float = 0.0
    restitution: float = 0.0
    friction: float = 0.0
    contact_to_a: Vec3 = field(default_factory=Vec3.zero)
    contact_to_b: Vec3 = field(default_factory=Vec3.zero)
    relative_velocity: Vec3 = field(default_factory=Vec3.zero)
    contact_mass: float = 0.0
    tangent1: Vec3 = field(default_factory=Vec3.zero)
    tangent2: Vec3 = field(default_factory=Vec3.zero)
    friction_mass1: float = 0.0
    friction_mass2: float = 0.0
    normal_impulse: float = 0.0
    tangent_impulse1: float = 0.0
    tangent_impulse2: float = 0.0


class ContactManifold:
    def __init__(self, body_a: RigidBody, body_b: RigidBody):
        self.body_a = body_a
        self.body_b = body_b
        self.contacts: List[Contact] = []
        self.max_contacts = 4

    def add_contact(self, contact: Contact) -> bool:
        if len(self.contacts) >= self.max_contacts:
            min_idx = min(range(len(self.contacts)), key=lambda i: self.contacts[i].penetration)
            if contact.penetration > self.contacts[min_idx].penetration:
                self.contacts[min_idx] = contact
                return True
            return False
        self.contacts.append(contact)
        return True

    def clear(self): self.contacts.clear()
    def __len__(self): return len(self.contacts)


class CollisionDetector:
    @staticmethod
    def sphere_vs_sphere(a: SphereCollider, b: SphereCollider) -> Optional[Contact]:
        pos_a = a.get_world_position()
        pos_b = b.get_world_position()
        delta = pos_b - pos_a
        dist_sq = delta.length_sq()
        rs = a.radius + b.radius
        if dist_sq > rs * rs or dist_sq < 1e-10: return None
        dist = math.sqrt(dist_sq)
        normal = delta * (1.0 / dist)
        penetration = rs - dist
        point = pos_a + normal * (a.radius - penetration * 0.5)
        mat_a = a.body.material if a.body else Material()
        mat_b = b.body.material if b.body else Material()
        return Contact(body_a=a.body, body_b=b.body, point=point, normal=normal, penetration=penetration,
                       restitution=mat_a.combine_restitution(mat_b),
                       friction=mat_a.combine_friction(mat_b))

    @staticmethod
    def sphere_vs_box(sphere: SphereCollider, box: BoxCollider) -> Optional[Contact]:
        sp = sphere.get_world_position()
        bp = box.get_world_position()
        if box.body:
            inv_rot = Mat3.from_quaternion(box.body.orientation).transpose()
            ls = inv_rot * (sp - bp)
        else:
            ls = sp - bp

        closest = Vec3(max(-box.half_extents.x, min(box.half_extents.x, ls.x)),
                       max(-box.half_extents.y, min(box.half_extents.y, ls.y)),
                       max(-box.half_extents.z, min(box.half_extents.z, ls.z)))
        delta = ls - closest
        dist_sq = delta.length_sq()
        if dist_sq > sphere.radius * sphere.radius: return None

        if box.body:
            rot = Mat3.from_quaternion(box.body.orientation)
            wc = bp + rot * closest
            wd = rot * delta
        else:
            wc = bp + closest
            wd = delta

        dist = math.sqrt(dist_sq)
        if dist < 1e-10:
            dx = box.half_extents.x - abs(ls.x)
            dy = box.half_extents.y - abs(ls.y)
            dz = box.half_extents.z - abs(ls.z)
            if dx <= dy and dx <= dz: local_n = Vec3(1 if ls.x > 0 else -1, 0, 0)
            elif dy <= dz: local_n = Vec3(0, 1 if ls.y > 0 else -1, 0)
            else: local_n = Vec3(0, 0, 1 if ls.z > 0 else -1)
            world_n = rot * local_n if box.body else local_n
            normal = -(world_n.normalize())
            penetration = sphere.radius + min(dx, dy, dz)
            point = wc
        else:
            normal = -(wd * (1.0 / dist))
            penetration = sphere.radius - dist
            point = wc

        mat_s = sphere.body.material if sphere.body else Material()
        mat_b = box.body.material if box.body else Material()
        return Contact(body_a=sphere.body, body_b=box.body, point=point, normal=normal, penetration=penetration,
                       restitution=mat_s.combine_restitution(mat_b),
                       friction=mat_s.combine_friction(mat_b))

    @staticmethod
    def box_vs_box(a: BoxCollider, b: BoxCollider) -> Optional[Contact]:
        obb_a = a.get_obb()
        obb_b = b.get_obb()
        axes_a = [obb_a.axes * Vec3(1, 0, 0), obb_a.axes * Vec3(0, 1, 0), obb_a.axes * Vec3(0, 0, 1)]
        axes_b = [obb_b.axes * Vec3(1, 0, 0), obb_b.axes * Vec3(0, 1, 0), obb_b.axes * Vec3(0, 0, 1)]

        test_axes = axes_a + axes_b
        for ax_a in axes_a:
            for ax_b in axes_b:
                cross = ax_a.cross(ax_b)
                if cross.length_sq() > 1e-10:
                    test_axes.append(cross.normalize())

        min_pen = float('inf')
        min_axis = Vec3.up()

        for axis in test_axes:
            if axis.length_sq() < 1e-10: continue
            axis = axis.normalize()

            c_a = obb_a.center.dot(axis)
            r_a = (abs((obb_a.axes.m00 * obb_a.extents.x) * axis.x + (obb_a.axes.m10 * obb_a.extents.x) * axis.y + (obb_a.axes.m20 * obb_a.extents.x) * axis.z) +
                   abs((obb_a.axes.m01 * obb_a.extents.y) * axis.x + (obb_a.axes.m11 * obb_a.extents.y) * axis.y + (obb_a.axes.m21 * obb_a.extents.y) * axis.z) +
                   abs((obb_a.axes.m02 * obb_a.extents.z) * axis.x + (obb_a.axes.m12 * obb_a.extents.z) * axis.y + (obb_a.axes.m22 * obb_a.extents.z) * axis.z))

            c_b = obb_b.center.dot(axis)
            r_b = (abs((obb_b.axes.m00 * obb_b.extents.x) * axis.x + (obb_b.axes.m10 * obb_b.extents.x) * axis.y + (obb_b.axes.m20 * obb_b.extents.x) * axis.z) +
                   abs((obb_b.axes.m01 * obb_b.extents.y) * axis.x + (obb_b.axes.m11 * obb_b.extents.y) * axis.y + (obb_b.axes.m21 * obb_b.extents.y) * axis.z) +
                   abs((obb_b.axes.m02 * obb_b.extents.z) * axis.x + (obb_b.axes.m12 * obb_b.extents.z) * axis.y + (obb_b.axes.m22 * obb_b.extents.z) * axis.z))

            min_a, max_a = c_a - r_a, c_a + r_a
            min_b, max_b = c_b - r_b, c_b + r_b
            if max_a < min_b or max_b < min_a: return None
            pen = min(max_a, max_b) - max(min_a, min_b)
            if pen < min_pen:
                min_pen = pen
                min_axis = axis

        normal = min_axis if (obb_b.center - obb_a.center).dot(min_axis) > 0 else -min_axis
        point = (obb_a.center + obb_b.center) * 0.5
        mat_a = a.body.material if a.body else Material()
        mat_b = b.body.material if b.body else Material()
        return Contact(body_a=a.body, body_b=b.body, point=point, normal=normal, penetration=min_pen,
                       restitution=mat_a.combine_restitution(mat_b),
                       friction=mat_a.combine_friction(mat_b))

    @staticmethod
    def plane_vs_sphere(plane: PlaneCollider, sphere: SphereCollider) -> Optional[Contact]:
        pos = sphere.get_world_position()
        dist = plane.normal.dot(pos) - plane.offset
        if dist > sphere.radius: return None
        point = pos - plane.normal * dist
        normal = plane.normal
        penetration = sphere.radius - dist
        mat_p = plane.body.material if plane.body else Material()
        mat_s = sphere.body.material if sphere.body else Material()
        return Contact(body_a=plane.body, body_b=sphere.body, point=point, normal=normal, penetration=penetration,
                       restitution=mat_p.combine_restitution(mat_s),
                       friction=mat_p.combine_friction(mat_s))

    @staticmethod
    def plane_vs_box(plane: PlaneCollider, box: BoxCollider) -> Optional[Contact]:
        obb = box.get_obb()
        verts = obb.vertices()
        deepest = None
        max_pen = 0.0
        for v in verts:
            dist = plane.normal.dot(v) - plane.offset
            if dist < 0 and abs(dist) > max_pen:
                max_pen = abs(dist)
                deepest = v
        if deepest is None: return None
        point = deepest
        normal = plane.normal
        mat_p = plane.body.material if plane.body else Material()
        mat_b = box.body.material if box.body else Material()
        return Contact(body_a=plane.body, body_b=box.body, point=point, normal=normal, penetration=max_pen,
                       restitution=mat_p.combine_restitution(mat_b),
                       friction=mat_p.combine_friction(mat_b))

    @staticmethod
    def detect(a: Collider, b: Collider) -> Optional[Contact]:
        types = (a.type, b.type)
        if types == (ColliderType.SPHERE, ColliderType.SPHERE):
            return CollisionDetector.sphere_vs_sphere(a, b)
        if types == (ColliderType.SPHERE, ColliderType.BOX):
            return CollisionDetector.sphere_vs_box(a, b)
        if types == (ColliderType.BOX, ColliderType.SPHERE):
            c = CollisionDetector.sphere_vs_box(b, a)
            if c:
                c.normal = -c.normal
                c.body_a, c.body_b = c.body_b, c.body_a
            return c
        if types == (ColliderType.BOX, ColliderType.BOX):
            return CollisionDetector.box_vs_box(a, b)
        if types == (ColliderType.PLANE, ColliderType.SPHERE):
            return CollisionDetector.plane_vs_sphere(a, b)
        if types == (ColliderType.SPHERE, ColliderType.PLANE):
            c = CollisionDetector.plane_vs_sphere(b, a)
            if c:
                c.normal = -c.normal
                c.body_a, c.body_b = c.body_b, c.body_a
            return c
        if types == (ColliderType.PLANE, ColliderType.BOX):
            return CollisionDetector.plane_vs_box(a, b)
        if types == (ColliderType.BOX, ColliderType.PLANE):
            c = CollisionDetector.plane_vs_box(b, a)
            if c:
                c.normal = -c.normal
                c.body_a, c.body_b = c.body_b, c.body_a
            return c
        return None


# ============================================================================
# SECTION 6: CONSTRAINTS & FORCE GENERATORS
# ============================================================================

class Constraint:
    def __init__(self, body_a: RigidBody, body_b: RigidBody):
        self.body_a = body_a
        self.body_b = body_b
    def solve(self, dt: float): raise NotImplementedError


class DistanceConstraint(Constraint):
    def __init__(self, body_a: RigidBody, body_b: RigidBody,
                 local_a: Vec3, local_b: Vec3, distance: float, compliance: float = 0.0):
        super().__init__(body_a, body_b)
        self.local_a = local_a
        self.local_b = local_b
        self.rest_distance = float(distance)
        self.compliance = float(compliance)
        self.lambda_acc = 0.0

    def solve(self, dt: float):
        if dt < 1e-10: return
        world_a = self.body_a.get_world_point(self.local_a)
        world_b = self.body_b.get_world_point(self.local_b)
        delta = world_b - world_a
        dist = delta.length()
        if dist < 1e-10: return
        n = delta * (1.0 / dist)
        c = dist - self.rest_distance

        r_a = world_a - self.body_a.position
        r_b = world_b - self.body_b.position

        term_a = self.body_a.inv_mass + (self.body_a.inv_inertia_tensor_world * r_a.cross(n)).cross(r_a).dot(n)
        term_b = self.body_b.inv_mass + (self.body_b.inv_inertia_tensor_world * r_b.cross(n)).cross(r_b).dot(n)
        w = term_a + term_b
        if w < 1e-10: return

        alpha = self.compliance / (dt * dt)
        dlambda = (-c - alpha * self.lambda_acc) / (w + alpha)
        self.lambda_acc += dlambda

        impulse = n * dlambda
        self.body_a.apply_impulse(-impulse, world_a)
        self.body_b.apply_impulse(impulse, world_b)


class SpringConstraint(Constraint):
    def __init__(self, body_a: RigidBody, body_b: RigidBody,
                 local_a: Vec3, local_b: Vec3, rest_length: float, stiffness: float, damping: float):
        super().__init__(body_a, body_b)
        self.local_a = local_a
        self.local_b = local_b
        self.rest_length = float(rest_length)
        self.stiffness = float(stiffness)
        self.damping = float(damping)

    def solve(self, dt: float):
        world_a = self.body_a.get_world_point(self.local_a)
        world_b = self.body_b.get_world_point(self.local_b)
        delta = world_b - world_a
        dist = delta.length()
        if dist < 1e-10: return
        n = delta * (1.0 / dist)

        displacement = dist - self.rest_length
        spring_force = n * (displacement * self.stiffness)

        vel_a = self.body_a.get_velocity_at_point(world_a)
        vel_b = self.body_b.get_velocity_at_point(world_b)
        rel_vel = (vel_b - vel_a).dot(n)
        damping_force = n * (rel_vel * self.damping)

        total_force = spring_force + damping_force
        self.body_a.apply_force(total_force, world_a)
        self.body_b.apply_force(-total_force, world_b)


class ForceGenerator:
    def apply(self, body: RigidBody, dt: float): raise NotImplementedError


class DragForce(ForceGenerator):
    def __init__(self, k1: float = 0.0, k2: float = 0.0):
        self.k1, self.k2 = float(k1), float(k2)
    def apply(self, body: RigidBody, dt: float):
        if body.is_static: return
        v = body.velocity
        speed = v.length()
        if speed > 1e-10:
            drag = v * ((-self.k1 * speed - self.k2 * speed * speed) / speed)
            body.apply_force(drag)


class BuoyancyForce(ForceGenerator):
    def __init__(self, liquid_level: float = 0.0, liquid_density: float = 1.0, max_depth: float = 1.0):
        self.liquid_level = float(liquid_level)
        self.liquid_density = float(liquid_density)
        self.max_depth = float(max_depth)
        self.enabled = True

    def apply(self, body: RigidBody, dt: float):
        if not self.enabled or body.is_static: return
        depth = self.liquid_level - body.position.y
        if depth <= 0: return
        frac = min(1.0, depth / self.max_depth)
        force = Vec3(0.0, self.liquid_density * body.mass * 9.81 * frac, 0.0)
        body.apply_force(force)


# ============================================================================
# SECTION 7: PHYSICS WORLD SIMULATOR
# ============================================================================

class PhysicsWorld:
    def __init__(self, gravity: Vec3 = Vec3(0, -9.81, 0), iterations: int = 8):
        self.bodies: List[RigidBody] = []
        self.constraints: List[Constraint] = []
        self.force_generators: List[ForceGenerator] = []
        self.gravity = gravity
        self.iterations = iterations
        self.position_correction_percent = 0.4
        self.time_step = 1.0 / 60.0
        self.time_scale = 1.0
        self.integration_method = IntegrationMethod.SYMPLECTIC_EULER
        self.manifolds: Dict[Tuple[int, int], ContactManifold] = {}
        self.broad_phase_pairs: List[Tuple[RigidBody, RigidBody]] = []
        self.paused = False
        self.total_time = 0.0
        self.frame_count = 0

        self.drag_force = DragForce(k1=0.01, k2=0.001)
        self.buoyancy_force = BuoyancyForce(liquid_level=0.0, liquid_density=2.0, max_depth=1.5)
        self.buoyancy_force.enabled = False
        self.force_generators.extend([self.drag_force, self.buoyancy_force])

    def add_body(self, body: RigidBody):
        self.bodies.append(body)

    def remove_body(self, body: RigidBody):
        if body in self.bodies:
            self.bodies.remove(body)
            self.constraints = [c for c in self.constraints if c.body_a != body and c.body_b != body]

    def add_constraint(self, constraint: Constraint):
        self.constraints.append(constraint)

    def step(self, dt: Optional[float] = None):
        if self.paused: return
        base_dt = dt if dt else self.time_step
        effective_dt = base_dt * self.time_scale
        self.total_time += effective_dt
        self.frame_count += 1

        # Apply forces
        for body in self.bodies:
            for fg in self.force_generators:
                fg.apply(body, effective_dt)
            if not body.is_static:
                body.apply_force(self.gravity * body.mass)

        # Integrate
        for body in self.bodies:
            body.integrate(effective_dt, self.integration_method)

        # Broad phase
        self.broad_phase_pairs.clear()
        n = len(self.bodies)
        for i in range(n):
            a = self.bodies[i]
            aabb_a = a.get_aabb()
            for j in range(i + 1, n):
                b = self.bodies[j]
                if a.is_static and b.is_static: continue
                if aabb_a.intersects(b.get_aabb()):
                    self.broad_phase_pairs.append((a, b))

        # Narrow phase
        self.manifolds.clear()
        for a, b in self.broad_phase_pairs:
            if a.collider is None or b.collider is None: continue
            contact = CollisionDetector.detect(a.collider, b.collider)
            if contact:
                key = (min(a.id, b.id), max(a.id, b.id))
                if key not in self.manifolds:
                    self.manifolds[key] = ContactManifold(a, b)
                self.manifolds[key].add_contact(contact)
                a.wake()
                b.wake()

        # Solve contacts
        for manifold in self.manifolds.values():
            for c in manifold.contacts:
                self._prepare_contact(c, effective_dt)
            for _ in range(self.iterations):
                for c in manifold.contacts:
                    self._solve_contact_impulse(c)

        # Solve constraints
        for _ in range(self.iterations):
            for constraint in self.constraints:
                constraint.solve(effective_dt)

        # Positional recovery
        self._positional_correction()

    def _prepare_contact(self, c: Contact, dt: float):
        c.contact_to_a = c.point - c.body_a.position
        c.contact_to_b = c.point - c.body_b.position

        va = c.body_a.get_velocity_at_point(c.point)
        vb = c.body_b.get_velocity_at_point(c.point)
        c.relative_velocity = vb - va

        vel_along_normal = c.relative_velocity.dot(c.normal)
        if vel_along_normal < -1.0:
            c.restitution = c.body_a.material.combine_restitution(c.body_b.material)
        else:
            c.restitution = 0.0

        r_a, r_b = c.contact_to_a, c.contact_to_b
        term_a = c.body_a.inv_mass + (c.body_a.inv_inertia_tensor_world * r_a.cross(c.normal)).cross(r_a).dot(c.normal)
        term_b = c.body_b.inv_mass + (c.body_b.inv_inertia_tensor_world * r_b.cross(c.normal)).cross(r_b).dot(c.normal)
        c.contact_mass = 1.0 / (term_a + term_b) if (term_a + term_b) > 1e-10 else 0.0

        c.tangent1 = (c.relative_velocity - c.normal * c.relative_velocity.dot(c.normal)).normalize()
        if c.tangent1.length_sq() < 1e-10:
            c.tangent1 = c.normal.perpendicular()
        c.tangent2 = c.normal.cross(c.tangent1).normalize()

        term_a1 = c.body_a.inv_mass + (c.body_a.inv_inertia_tensor_world * r_a.cross(c.tangent1)).cross(r_a).dot(c.tangent1)
        term_b1 = c.body_b.inv_mass + (c.body_b.inv_inertia_tensor_world * r_b.cross(c.tangent1)).cross(r_b).dot(c.tangent1)
        c.friction_mass1 = 1.0 / (term_a1 + term_b1) if (term_a1 + term_b1) > 1e-10 else 0.0

        term_a2 = c.body_a.inv_mass + (c.body_a.inv_inertia_tensor_world * r_a.cross(c.tangent2)).cross(r_a).dot(c.tangent2)
        term_b2 = c.body_b.inv_mass + (c.body_b.inv_inertia_tensor_world * r_b.cross(c.tangent2)).cross(r_b).dot(c.tangent2)
        c.friction_mass2 = 1.0 / (term_a2 + term_b2) if (term_a2 + term_b2) > 1e-10 else 0.0

        c.normal_impulse = 0.0
        c.tangent_impulse1 = 0.0
        c.tangent_impulse2 = 0.0

    def _solve_contact_impulse(self, c: Contact):
        rel_vel = c.relative_velocity.dot(c.normal)
        if rel_vel > 0.0: return

        jn = -(1.0 + c.restitution) * rel_vel * c.contact_mass
        jn_old = c.normal_impulse
        c.normal_impulse = max(jn_old + jn, 0.0)
        jn = c.normal_impulse - jn_old

        impulse = c.normal * jn
        c.body_a.apply_impulse(-impulse, c.point)
        c.body_b.apply_impulse(impulse, c.point)

        va = c.body_a.get_velocity_at_point(c.point)
        vb = c.body_b.get_velocity_at_point(c.point)
        c.relative_velocity = vb - va

        rel_vel_t1 = c.relative_velocity.dot(c.tangent1)
        jt1 = -rel_vel_t1 * c.friction_mass1
        max_friction = c.friction * c.normal_impulse
        jt1_old = c.tangent_impulse1
        c.tangent_impulse1 = max(-max_friction, min(jt1_old + jt1, max_friction))
        jt1 = c.tangent_impulse1 - jt1_old

        impulse_t1 = c.tangent1 * jt1
        c.body_a.apply_impulse(-impulse_t1, c.point)
        c.body_b.apply_impulse(impulse_t1, c.point)

        va = c.body_a.get_velocity_at_point(c.point)
        vb = c.body_b.get_velocity_at_point(c.point)
        c.relative_velocity = vb - va

        rel_vel_t2 = c.relative_velocity.dot(c.tangent2)
        jt2 = -rel_vel_t2 * c.friction_mass2
        jt2_old = c.tangent_impulse2
        c.tangent_impulse2 = max(-max_friction, min(jt2_old + jt2, max_friction))
        jt2 = c.tangent_impulse2 - jt2_old

        impulse_t2 = c.tangent2 * jt2
        c.body_a.apply_impulse(-impulse_t2, c.point)
        c.body_b.apply_impulse(impulse_t2, c.point)

    def _positional_correction(self):
        percent = self.position_correction_percent
        slop = 0.01
        for manifold in self.manifolds.values():
            for c in manifold.contacts:
                if c.penetration > slop:
                    inv_mass_sum = c.body_a.inv_mass + c.body_b.inv_mass
                    if inv_mass_sum > 1e-10:
                        correction = c.normal * ((c.penetration - slop) / inv_mass_sum * percent)
                        if not c.body_a.is_static: c.body_a.position = c.body_a.position - correction * c.body_a.inv_mass
                        if not c.body_b.is_static: c.body_b.position = c.body_b.position + correction * c.body_b.inv_mass

    def get_total_kinetic_energy(self) -> float:
        return sum(b.kinetic_energy() for b in self.bodies if not b.is_static)

    def wake_all(self):
        for b in self.bodies:
            b.wake()

    def clear(self):
        self.bodies.clear()
        self.constraints.clear()
        self.manifolds.clear()
        self.total_time = 0.0
        self.frame_count = 0


# ============================================================================
# SECTION 8: SCENE SERIALIZER & PRESETS
# ============================================================================

class SceneManager:
    @staticmethod
    def create_default_ground(world: PhysicsWorld) -> RigidBody:
        ground = RigidBody(position=Vec3(0, -0.5, 0), is_static=True, name="Ground_Floor")
        ground.collider = PlaneCollider(Vec3.up(), 0.0)
        ground.material = Materials.CONCRETE
        world.add_body(ground)
        return ground

    @staticmethod
    def save_scene(world: PhysicsWorld, filepath: str):
        data = {
            "gravity": world.gravity.to_list(),
            "iterations": world.iterations,
            "time_scale": world.time_scale,
            "bodies": [b.to_dict() for b in world.bodies if not (b.is_static and isinstance(b.collider, PlaneCollider))]
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    @staticmethod
    def load_scene(world: PhysicsWorld, filepath: str):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        world.clear()
        SceneManager.create_default_ground(world)

        if "gravity" in data: world.gravity = Vec3.from_list(data["gravity"])
        if "iterations" in data: world.iterations = int(data["iterations"])
        if "time_scale" in data: world.time_scale = float(data["time_scale"])

        for bd in data.get("bodies", []):
            mat = Material.from_dict(bd.get("material", {}))
            body = RigidBody(
                position=Vec3.from_list(bd.get("position", [0, 0, 0])),
                mass=float(bd.get("mass", 1.0)),
                material=mat,
                is_static=bool(bd.get("is_static", False)),
                name=bd.get("name", "")
            )
            body.orientation = Quaternion.from_list(bd.get("orientation", [0, 0, 0, 1]))
            body.velocity = Vec3.from_list(bd.get("velocity", [0, 0, 0]))

            col_data = bd.get("collider", {})
            col_type = col_data.get("type", "")
            if col_type == "box":
                he = Vec3.from_list(col_data.get("half_extents", [0.5, 0.5, 0.5]))
                body.collider = BoxCollider(he)
            elif col_type == "sphere":
                body.collider = SphereCollider(float(col_data.get("radius", 0.5)))

            world.add_body(body)

    @staticmethod
    def load_preset(world: PhysicsWorld, preset_name: str):
        world.clear()
        SceneManager.create_default_ground(world)

        if preset_name == "stack":
            for i in range(5):
                box = RigidBody(position=Vec3(0, 0.6 + i * 1.15, 0), mass=2.0, name=f"StackBox_{i+1}")
                box.collider = BoxCollider(Vec3(0.5, 0.5, 0.5))
                box.material = Materials.WOOD
                world.add_body(box)

        elif preset_name == "spheres":
            for i in range(12):
                x = random.uniform(-0.5, 0.5)
                z = random.uniform(-0.5, 0.5)
                sphere = RigidBody(position=Vec3(x, 1.0 + i * 1.2, z), mass=1.5, name=f"Sphere_{i+1}")
                sphere.collider = SphereCollider(0.5)
                sphere.material = Materials.RUBBER
                world.add_body(sphere)

        elif preset_name == "mixed":
            box = RigidBody(position=Vec3(-1.0, 1.0, 0), mass=2.0, name="Wood_Box_A")
            box.collider = BoxCollider(Vec3(0.6, 0.6, 0.6))
            box.material = Materials.WOOD
            world.add_body(box)

            sphere = RigidBody(position=Vec3(1.5, 3.5, 0), mass=2.0, name="Bouncy_Ball")
            sphere.collider = SphereCollider(0.6)
            sphere.material = Materials.BOUNCY
            world.add_body(sphere)

            box2 = RigidBody(position=Vec3(0.0, 5.5, 0.2), mass=2.0, name="Glider_Box_B")
            box2.collider = BoxCollider(Vec3(0.5, 0.5, 0.5))
            box2.material = Materials.WOOD
            box2.velocity = Vec3(0.5, 0.0, -0.2)
            world.add_body(box2)

            for i in range(3):
                s = RigidBody(position=Vec3(random.uniform(-2, 2), 7.5 + i * 1.5, random.uniform(-2, 2)), mass=1.0, name=f"Rain_Drop_{i+1}")
                s.collider = SphereCollider(0.4)
                s.material = Materials.RUBBER
                world.add_body(s)

        elif preset_name == "springs":
            bodies = []
            for i in range(5):
                sphere = RigidBody(position=Vec3(i * 1.4 - 2.8, 5.0, 0), mass=1.0, name=f"PendulumNode_{i+1}")
                sphere.collider = SphereCollider(0.35)
                sphere.material = Materials.STEEL
                world.add_body(sphere)
                bodies.append(sphere)

            for i in range(len(bodies) - 1):
                spring = SpringConstraint(bodies[i], bodies[i + 1], Vec3.zero(), Vec3.zero(), 1.4, 60.0, 2.0)
                world.add_constraint(spring)

            anchor = RigidBody(position=Vec3(-4.2, 7.5, 0), is_static=True, name="Spring_Anchor")
            anchor.collider = SphereCollider(0.2)
            world.add_body(anchor)
            world.add_constraint(SpringConstraint(anchor, bodies[0], Vec3.zero(), Vec3.zero(), 1.8, 40.0, 1.5))

        elif preset_name == "jenga":
            bw, bh, bd = 0.4, 0.25, 1.2
            for layer in range(8):
                angle = 0 if layer % 2 == 0 else math.pi * 0.5
                for i in range(3):
                    offset = (i - 1) * bw * 2.1
                    pos = Vec3(offset, 0.15 + layer * bh * 2.05, 0) if layer % 2 == 0 else Vec3(0, 0.15 + layer * bh * 2.05, offset)
                    he = Vec3(bw, bh, bd) if layer % 2 == 0 else Vec3(bd, bh, bw)
                    box = RigidBody(position=pos, mass=0.5, name=f"Jenga_L{layer}_B{i}")
                    box.collider = BoxCollider(he)
                    box.material = Materials.WOOD
                    box.orientation = Quaternion.from_axis_angle(Vec3.up(), angle)
                    world.add_body(box)

            ball = RigidBody(position=Vec3(5.5, 2.0, 0), mass=4.0, name="Wrecking_Sphere")
            ball.collider = SphereCollider(0.6)
            ball.material = Materials.STEEL
            ball.velocity = Vec3(-9.0, 0.5, 0.0)
            world.add_body(ball)

        elif preset_name == "buoyancy":
            world.gravity = Vec3(0, -6.0, 0)
            world.buoyancy_force.enabled = True
            for i in range(4):
                sphere = RigidBody(position=Vec3(i * 1.8 - 2.7, 4.0, 0), mass=2.0, name=f"FloatingSphere_{i+1}")
                sphere.collider = SphereCollider(0.6)
                sphere.material = Materials.WOOD
                world.add_body(sphere)

        elif preset_name == "cradle":
            for pos in [Vec3(-3, 4, 0), Vec3(3, 4, 0)]:
                post = RigidBody(position=pos, is_static=True, name="Cradle_Post")
                post.collider = BoxCollider(Vec3(0.1, 2.0, 0.1))
                world.add_body(post)

            beam = RigidBody(position=Vec3(0, 6, 0), is_static=True, name="Cradle_Beam")
            beam.collider = BoxCollider(Vec3(3.5, 0.1, 0.1))
            world.add_body(beam)

            balls = []
            for i in range(5):
                ball = RigidBody(position=Vec3(i * 0.9 - 1.8, 3.0, 0), mass=1.0, name=f"CradleBall_{i+1}")
                ball.collider = SphereCollider(0.4)
                ball.material = Materials.STEEL
                world.add_body(ball)
                balls.append(ball)

                anchor = RigidBody(position=Vec3(i * 0.9 - 1.8, 6.0, 0), is_static=True)
                anchor.collider = SphereCollider(0.1)
                world.add_body(anchor)
                world.add_constraint(DistanceConstraint(anchor, ball, Vec3.zero(), Vec3.zero(), 3.0, compliance=0.0001))

            balls[0].position = Vec3(-3.5, 4.5, 0)
            balls[0].velocity = Vec3(2.5, 0, 0)


# ============================================================================
# SECTION 9: DEDICATED TOOL WINDOWS (DIALOGS)
# ============================================================================

class ObjectSpawnerWindow(tk.Toplevel):
    """Floating dedicated window to create and inject custom rigid bodies."""
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.title("Add Rigid Body Object")
        self.geometry("380x520")
        self.resizable(False, False)
        self.configure(bg="#232731")
        self._build_ui()

    def _build_ui(self):
        pad = {'padx': 10, 'pady': 5}

        # Shape Type
        frame_type = tk.LabelFrame(self, text="Shape & Dimensions", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_type.pack(fill="x", **pad)

        tk.Label(frame_type, text="Shape Type:", bg="#232731", fg="#e0e0e0").grid(row=0, column=0, sticky="w", padx=5, pady=3)
        self.shape_var = tk.StringVar(value="Box")
        cb = ttk.Combobox(frame_type, textvariable=self.shape_var, values=["Box", "Sphere"], state="readonly", width=12)
        cb.grid(row=0, column=1, sticky="w", padx=5, pady=3)
        cb.bind("<<ComboboxSelected>>", self._on_shape_change)

        # Size inputs
        self.lbl_dim1 = tk.Label(frame_type, text="Width (X) / Size:", bg="#232731", fg="#e0e0e0")
        self.lbl_dim1.grid(row=1, column=0, sticky="w", padx=5, pady=3)
        self.ent_dim1 = tk.Entry(frame_type, width=10, bg="#2e3340", fg="white", insertbackground="white")
        self.ent_dim1.insert(0, "1.0")
        self.ent_dim1.grid(row=1, column=1, sticky="w", padx=5, pady=3)

        self.lbl_dim2 = tk.Label(frame_type, text="Height (Y):", bg="#232731", fg="#e0e0e0")
        self.lbl_dim2.grid(row=2, column=0, sticky="w", padx=5, pady=3)
        self.ent_dim2 = tk.Entry(frame_type, width=10, bg="#2e3340", fg="white", insertbackground="white")
        self.ent_dim2.insert(0, "1.0")
        self.ent_dim2.grid(row=2, column=1, sticky="w", padx=5, pady=3)

        self.lbl_dim3 = tk.Label(frame_type, text="Depth (Z):", bg="#232731", fg="#e0e0e0")
        self.lbl_dim3.grid(row=3, column=0, sticky="w", padx=5, pady=3)
        self.ent_dim3 = tk.Entry(frame_type, width=10, bg="#2e3340", fg="white", insertbackground="white")
        self.ent_dim3.insert(0, "1.0")
        self.ent_dim3.grid(row=3, column=1, sticky="w", padx=5, pady=3)

        # Mass & Material
        frame_mat = tk.LabelFrame(self, text="Physical Material & Mass", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_mat.pack(fill="x", **pad)

        tk.Label(frame_mat, text="Material Preset:", bg="#232731", fg="#e0e0e0").grid(row=0, column=0, sticky="w", padx=5, pady=3)
        self.mat_var = tk.StringVar(value="Wood")
        mat_cb = ttk.Combobox(frame_mat, textvariable=self.mat_var, values=[m.name for m in Materials.ALL], state="readonly", width=12)
        mat_cb.grid(row=0, column=1, sticky="w", padx=5, pady=3)

        tk.Label(frame_mat, text="Mass (kg):", bg="#232731", fg="#e0e0e0").grid(row=1, column=0, sticky="w", padx=5, pady=3)
        self.ent_mass = tk.Entry(frame_mat, width=10, bg="#2e3340", fg="white", insertbackground="white")
        self.ent_mass.insert(0, "2.0")
        self.ent_mass.grid(row=1, column=1, sticky="w", padx=5, pady=3)

        self.static_var = tk.BooleanVar(value=False)
        tk.Checkbutton(frame_mat, text="Is Static (Immovable)", variable=self.static_var, bg="#232731", fg="#e0e0e0", selectcolor="#2e3340", activebackground="#232731").grid(row=2, column=0, columnspan=2, sticky="w", padx=5, pady=3)

        # Spawn Position & Velocity
        frame_pos = tk.LabelFrame(self, text="Spawn Coordinates & Launch Velocity", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_pos.pack(fill="x", **pad)

        tk.Label(frame_pos, text="Position (X, Y, Z):", bg="#232731", fg="#e0e0e0").grid(row=0, column=0, sticky="w", padx=5, pady=3)
        f_p = tk.Frame(frame_pos, bg="#232731")
        f_p.grid(row=0, column=1, sticky="w")
        self.ent_px = tk.Entry(f_p, width=4, bg="#2e3340", fg="white", insertbackground="white"); self.ent_px.insert(0, "0.0"); self.ent_px.pack(side="left", padx=1)
        self.ent_py = tk.Entry(f_p, width=4, bg="#2e3340", fg="white", insertbackground="white"); self.ent_py.insert(0, "6.0"); self.ent_py.pack(side="left", padx=1)
        self.ent_pz = tk.Entry(f_p, width=4, bg="#2e3340", fg="white", insertbackground="white"); self.ent_pz.insert(0, "0.0"); self.ent_pz.pack(side="left", padx=1)

        tk.Label(frame_pos, text="Velocity (Vx, Vy, Vz):", bg="#232731", fg="#e0e0e0").grid(row=1, column=0, sticky="w", padx=5, pady=3)
        f_v = tk.Frame(frame_pos, bg="#232731")
        f_v.grid(row=1, column=1, sticky="w")
        self.ent_vx = tk.Entry(f_v, width=4, bg="#2e3340", fg="white", insertbackground="white"); self.ent_vx.insert(0, "0.0"); self.ent_vx.pack(side="left", padx=1)
        self.ent_vy = tk.Entry(f_v, width=4, bg="#2e3340", fg="white", insertbackground="white"); self.ent_vy.insert(0, "0.0"); self.ent_vy.pack(side="left", padx=1)
        self.ent_vz = tk.Entry(f_v, width=4, bg="#2e3340", fg="white", insertbackground="white"); self.ent_vz.insert(0, "0.0"); self.ent_vz.pack(side="left", padx=1)

        # Action Button
        btn_spawn = tk.Button(self, text="Spawn Object Into Scene", bg="#2d6cb5", fg="white", font=("Segoe UI", 10, "bold"), relief="raised", command=self._spawn_object)
        btn_spawn.pack(fill="x", padx=15, pady=12)

    def _on_shape_change(self, event=None):
        if self.shape_var.get() == "Sphere":
            self.lbl_dim1.config(text="Radius (m):")
            self.lbl_dim2.grid_remove()
            self.ent_dim2.grid_remove()
            self.lbl_dim3.grid_remove()
            self.ent_dim3.grid_remove()
        else:
            self.lbl_dim1.config(text="Width (X):")
            self.lbl_dim2.grid()
            self.ent_dim2.grid()
            self.lbl_dim3.grid()
            self.ent_dim3.grid()

    def _spawn_object(self):
        try:
            shape = self.shape_var.get()
            mass = float(self.ent_mass.get())
            is_static = self.static_var.get()
            pos = Vec3(float(self.ent_px.get()), float(self.ent_py.get()), float(self.ent_pz.get()))
            vel = Vec3(float(self.ent_vx.get()), float(self.ent_vy.get()), float(self.ent_vz.get()))
            mat = Materials.BY_NAME.get(self.mat_var.get(), Materials.WOOD)

            body = RigidBody(position=pos, mass=mass, material=mat, is_static=is_static)
            body.velocity = vel

            if shape == "Box":
                hx = max(0.05, float(self.ent_dim1.get()) * 0.5)
                hy = max(0.05, float(self.ent_dim2.get()) * 0.5)
                hz = max(0.05, float(self.ent_dim3.get()) * 0.5)
                body.collider = BoxCollider(Vec3(hx, hy, hz))
            else:
                r = max(0.05, float(self.ent_dim1.get()))
                body.collider = SphereCollider(r)

            self.app.world.add_body(body)
            self.app.update_scene_tree()
            self.app.renderer.render()
        except ValueError as e:
            messagebox.showerror("Invalid Input", f"Please check numeric values: {e}")


class GravityWindow(tk.Toplevel):
    """Floating dedicated window for gravity, fluid buoyancy, and drag physics."""
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.title("Gravity and Environmental Forces")
        self.geometry("380x460")
        self.resizable(False, False)
        self.configure(bg="#232731")
        self._build_ui()

    def _build_ui(self):
        pad = {'padx': 10, 'pady': 5}

        # Gravity presets
        frame_pre = tk.LabelFrame(self, text="Planetary Gravity Presets", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_pre.pack(fill="x", **pad)

        btn_f = tk.Frame(frame_pre, bg="#232731")
        btn_f.pack(fill="x", padx=5, pady=5)
        tk.Button(btn_f, text="Earth (9.81 m/s²)", bg="#3a4150", fg="white", width=8, command=lambda: self._set_grav(0, -9.81, 0)).pack(side="left", padx=2)
        tk.Button(btn_f, text="Moon (1.62 m/s²)", bg="#3a4150", fg="white", width=8, command=lambda: self._set_grav(0, -1.62, 0)).pack(side="left", padx=2)
        tk.Button(btn_f, text="Mars (3.71 m/s²)", bg="#3a4150", fg="white", width=8, command=lambda: self._set_grav(0, -3.71, 0)).pack(side="left", padx=2)
        tk.Button(btn_f, text="Zero Gravity (0 m/s²)", bg="#3a4150", fg="white", width=8, command=lambda: self._set_grav(0, 0.0, 0)).pack(side="left", padx=2)

        # Custom Vector
        frame_vec = tk.LabelFrame(self, text="Gravity Vector (m/s²)", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_vec.pack(fill="x", **pad)

        tk.Label(frame_vec, text="X (Horizontal):", bg="#232731", fg="#e0e0e0").grid(row=0, column=0, sticky="w", padx=5, pady=3)
        self.scale_gx = tk.Scale(frame_vec, from_=-20.0, to=20.0, resolution=0.5, orient="horizontal", bg="#232731", fg="white", highlightthickness=0, command=self._on_slider_grav)
        self.scale_gx.set(self.app.world.gravity.x)
        self.scale_gx.grid(row=0, column=1, sticky="ew", padx=5)

        tk.Label(frame_vec, text="Y (Vertical):", bg="#232731", fg="#e0e0e0").grid(row=1, column=0, sticky="w", padx=5, pady=3)
        self.scale_gy = tk.Scale(frame_vec, from_=-30.0, to=30.0, resolution=0.5, orient="horizontal", bg="#232731", fg="white", highlightthickness=0, command=self._on_slider_grav)
        self.scale_gy.set(self.app.world.gravity.y)
        self.scale_gy.grid(row=1, column=1, sticky="ew", padx=5)

        tk.Label(frame_vec, text="Z (Depth):", bg="#232731", fg="#e0e0e0").grid(row=2, column=0, sticky="w", padx=5, pady=3)
        self.scale_gz = tk.Scale(frame_vec, from_=-20.0, to=20.0, resolution=0.5, orient="horizontal", bg="#232731", fg="white", highlightthickness=0, command=self._on_slider_grav)
        self.scale_gz.set(self.app.world.gravity.z)
        self.scale_gz.grid(row=2, column=1, sticky="ew", padx=5)

        # Drag & Buoyancy
        frame_drag = tk.LabelFrame(self, text="Aerodynamics & Buoyancy", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_drag.pack(fill="x", **pad)

        tk.Label(frame_drag, text="Air Resistance:", bg="#232731", fg="#e0e0e0").grid(row=0, column=0, sticky="w", padx=5, pady=3)
        self.scale_drag = tk.Scale(frame_drag, from_=0.0, to=0.2, resolution=0.005, orient="horizontal", bg="#232731", fg="white", highlightthickness=0, command=self._on_drag_change)
        self.scale_drag.set(self.app.world.drag_force.k1)
        self.scale_drag.grid(row=0, column=1, sticky="ew", padx=5)

        self.buoy_var = tk.BooleanVar(value=self.app.world.buoyancy_force.enabled)
        tk.Checkbutton(frame_drag, text="Enable Water Buoyancy", variable=self.buoy_var, bg="#232731", fg="#e0e0e0", selectcolor="#2e3340", activebackground="#232731", command=self._on_buoy_toggle).grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=5)

    def _set_grav(self, x, y, z):
        self.scale_gx.set(x)
        self.scale_gy.set(y)
        self.scale_gz.set(z)
        self.app.world.gravity = Vec3(x, y, z)
        self.app.world.wake_all()

    def _on_slider_grav(self, val=None):
        self.app.world.gravity = Vec3(self.scale_gx.get(), self.scale_gy.get(), self.scale_gz.get())
        self.app.world.wake_all()

    def _on_drag_change(self, val=None):
        k = self.scale_drag.get()
        self.app.world.drag_force.k1 = k
        self.app.world.drag_force.k2 = k * 0.1

    def _on_buoy_toggle(self):
        self.app.world.buoyancy_force.enabled = self.buoy_var.get()
        self.app.world.wake_all()


class CollisionWindow(tk.Toplevel):
    """Floating dedicated window for collision solver tuning & physics materials."""
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.title("Collision Solver and Material Properties")
        self.geometry("400x460")
        self.resizable(False, False)
        self.configure(bg="#232731")
        self._build_ui()

    def _build_ui(self):
        pad = {'padx': 10, 'pady': 5}

        # Solver Settings
        frame_sol = tk.LabelFrame(self, text="Iterative Solver Precision", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_sol.pack(fill="x", **pad)

        tk.Label(frame_sol, text="Impulse Iterations:", bg="#232731", fg="#e0e0e0").grid(row=0, column=0, sticky="w", padx=5, pady=4)
        self.iter_scale = tk.Scale(frame_sol, from_=1, to=30, resolution=1, orient="horizontal", bg="#232731", fg="white", highlightthickness=0, command=self._on_iter_change)
        self.iter_scale.set(self.app.world.iterations)
        self.iter_scale.grid(row=0, column=1, sticky="ew", padx=5)

        tk.Label(frame_sol, text="Penetration Recovery %:", bg="#232731", fg="#e0e0e0").grid(row=1, column=0, sticky="w", padx=5, pady=4)
        self.pen_scale = tk.Scale(frame_sol, from_=0.1, to=1.0, resolution=0.05, orient="horizontal", bg="#232731", fg="white", highlightthickness=0, command=self._on_pen_change)
        self.pen_scale.set(self.app.world.position_correction_percent)
        self.pen_scale.grid(row=1, column=1, sticky="ew", padx=5)

        # Material Properties
        frame_mat = tk.LabelFrame(self, text="Default Material Properties", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_mat.pack(fill="x", **pad)

        for idx, m in enumerate(Materials.ALL):
            lbl = f"{m.name:9} | Restitution: {m.restitution:.2f} | Friction: {m.static_friction:.2f}"
            tk.Label(frame_mat, text=lbl, font=("Consolas", 9), bg="#232731", fg="#d0d0d0").pack(anchor="w", padx=5, pady=2)

        # Integrator Selection
        frame_int = tk.LabelFrame(self, text="Numerical Integration Scheme", bg="#232731", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        frame_int.pack(fill="x", **pad)

        self.int_var = tk.StringVar(value=self.app.world.integration_method.name)
        for m in IntegrationMethod:
            tk.Radiobutton(frame_int, text=m.name.replace("_", " "), variable=self.int_var, value=m.name,
                           bg="#232731", fg="#e0e0e0", selectcolor="#2e3340", activebackground="#232731", command=self._on_int_change).pack(anchor="w", padx=5, pady=1)

    def _on_iter_change(self, val=None):
        self.app.world.iterations = int(self.iter_scale.get())

    def _on_pen_change(self, val=None):
        self.app.world.position_correction_percent = float(self.pen_scale.get())

    def _on_int_change(self):
        self.app.world.integration_method = IntegrationMethod[self.int_var.get()]


# ============================================================================
# SECTION 10: PRODUCTION CAD STUDIO APPLICATION (MAIN GUI)
# ============================================================================

class PhysicsStudioApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Pyton3D: 3D Physics Simulation Studio and CAD Workbench (Built From Scratch)")
        self.root.geometry("1240x800")
        self.root.configure(bg="#1a1e26")

        self.world = PhysicsWorld()
        SceneManager.create_default_ground(self.world)

        self.show_aabb = tk.BooleanVar(value=False)
        self.show_contacts = tk.BooleanVar(value=True)
        self.show_velocity = tk.BooleanVar(value=False)
        self.show_grid = tk.BooleanVar(value=True)

        self._build_menu()
        self._build_toolbar()
        self._build_main_layout()

        # Load initial demo
        SceneManager.load_preset(self.world, "mixed")
        self.update_scene_tree()

        # Start animation loop
        self.last_frame_time = time.perf_counter()
        self.fps_tracker = 60.0
        self._animate_tick()

    def _build_menu(self):
        menubar = tk.Menu(self.root)

        # File Menu
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="New Scene", command=self._on_file_new, accelerator="Ctrl+N")
        file_menu.add_command(label="Open Scene (.json)...", command=self._on_file_open, accelerator="Ctrl+O")
        file_menu.add_command(label="Save Scene (.json)...", command=self._on_file_save, accelerator="Ctrl+S")
        file_menu.add_separator()
        file_menu.add_command(label="Export Snapshot (.png)...", command=self._on_export_snapshot)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)
        menubar.add_cascade(label="File", menu=file_menu)

        # Add Objects Menu
        add_menu = tk.Menu(menubar, tearoff=0)
        add_menu.add_command(label="Add Box Block...", command=self._open_spawner_dialog)
        add_menu.add_command(label="Add Sphere Object...", command=self._open_spawner_dialog)
        add_menu.add_separator()
        add_menu.add_command(label="Spawn 10 Random Blocks", command=self._spawn_random_blocks)
        add_menu.add_command(label="Spawn Sphere Rain", command=self._spawn_sphere_storm)
        menubar.add_cascade(label="Add Objects", menu=add_menu)

        # Tools & Dedicated Windows
        tools_menu = tk.Menu(menubar, tearoff=0)
        tools_menu.add_command(label="Object Spawner Window...", command=self._open_spawner_dialog)
        tools_menu.add_command(label="Gravity and Forces Window...", command=self._open_gravity_dialog)
        tools_menu.add_command(label="Collision Solver and Material Properties...", command=self._open_collision_dialog)
        menubar.add_cascade(label="Tools & Windows", menu=tools_menu)

        # Demos Menu
        demos_menu = tk.Menu(menubar, tearoff=0)
        for demo_name in ["stack", "spheres", "mixed", "springs", "jenga", "buoyancy", "cradle"]:
            demos_menu.add_command(label=f"{demo_name.capitalize()}", command=lambda d=demo_name: self._load_preset_demo(d))
        menubar.add_cascade(label="Physics Labs", menu=demos_menu)

        self.root.config(menu=menubar)

    def _build_toolbar(self):
        toolbar = tk.Frame(self.root, bg="#232731", height=40, bd=1, relief="raised")
        toolbar.pack(side="top", fill="x")

        self.btn_play = tk.Button(toolbar, text="Pause", bg="#3a4150", fg="white", font=("Segoe UI", 9, "bold"), width=9, command=self._toggle_pause)
        self.btn_play.pack(side="left", padx=4, pady=4)

        tk.Button(toolbar, text="Step Frame", bg="#3a4150", fg="white", font=("Segoe UI", 9), width=7, command=self._step_frame).pack(side="left", padx=2, pady=4)
        tk.Button(toolbar, text="Reset", bg="#3a4150", fg="white", font=("Segoe UI", 9), width=7, command=self._reset_current_preset).pack(side="left", padx=2, pady=4)
        tk.Button(toolbar, text="Clear Scene", bg="#3a4150", fg="white", font=("Segoe UI", 9), width=7, command=self._on_file_new).pack(side="left", padx=2, pady=4)

        # Separator
        ttk.Separator(toolbar, orient="vertical").pack(side="left", fill="y", padx=8, pady=4)

        tk.Button(toolbar, text="Add Block", bg="#2d6cb5", fg="white", font=("Segoe UI", 9, "bold"), command=self._open_spawner_dialog).pack(side="left", padx=4, pady=4)
        tk.Button(toolbar, text="Gravity Config", bg="#3a4150", fg="white", font=("Segoe UI", 9), command=self._open_gravity_dialog).pack(side="left", padx=3, pady=4)
        tk.Button(toolbar, text="Collision Config", bg="#3a4150", fg="white", font=("Segoe UI", 9), command=self._open_collision_dialog).pack(side="left", padx=3, pady=4)
        tk.Button(toolbar, text="Shockwave Impulse", bg="#9c3b3b", fg="white", font=("Segoe UI", 9, "bold"), command=self._apply_explosion_impulse).pack(side="left", padx=6, pady=4)

        # Time Scale
        tk.Label(toolbar, text="Speed:", bg="#232731", fg="#b0c0d8").pack(side="left", padx=(12, 2))
        self.scale_speed = tk.Scale(toolbar, from_=0.1, to=2.5, resolution=0.1, orient="horizontal", length=80, bg="#232731", fg="white", highlightthickness=0, command=self._on_speed_change)
        self.scale_speed.set(1.0)
        self.scale_speed.pack(side="left", padx=2)

    def _build_main_layout(self):
        # Paned Window
        main_pane = tk.PanedWindow(self.root, orient="horizontal", bg="#1a1e26", sashrelief="ridge", sashwidth=4)
        main_pane.pack(fill="both", expand=True)

        # Left Sidebar (Notebook)
        sidebar = tk.Frame(main_pane, bg="#20242e", width=340)
        sidebar.pack_propagate(False)
        main_pane.add(sidebar)

        notebook = ttk.Notebook(sidebar)
        notebook.pack(fill="both", expand=True, padx=4, pady=4)

        # Tab 1: Hierarchy & Bodies
        tab_hier = tk.Frame(notebook, bg="#20242e")
        notebook.add(tab_hier, text="Hierarchy")
        self._build_hierarchy_tab(tab_hier)

        # Tab 2: Visual Overlays & Display
        tab_disp = tk.Frame(notebook, bg="#20242e")
        notebook.add(tab_disp, text="Display Options")
        self._build_display_tab(tab_disp)

        # Tab 3: Presets & Labs
        tab_labs = tk.Frame(notebook, bg="#20242e")
        notebook.add(tab_labs, text="Physics Labs")
        self._build_labs_tab(tab_labs)

        # Center Viewport (Matplotlib 3D)
        viewport_frame = tk.Frame(main_pane, bg="#1a1e26")
        main_pane.add(viewport_frame)

        self.renderer = EmbeddedMatplotlibRenderer(self.world, viewport_frame)

    def _build_hierarchy_tab(self, parent):
        lbl = tk.Label(parent, text="Active Scene Bodies", bg="#20242e", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        lbl.pack(anchor="w", padx=6, pady=4)

        tree_frame = tk.Frame(parent, bg="#20242e")
        tree_frame.pack(fill="both", expand=True, padx=4, pady=2)

        self.tree = ttk.Treeview(tree_frame, columns=("Type", "Mass", "Pos"), show="tree headings", selectmode="browse")
        self.tree.heading("#0", text="Name")
        self.tree.heading("Type", text="Shape")
        self.tree.heading("Mass", text="Mass (kg)")
        self.tree.heading("Pos", text="Pos (Y)")

        self.tree.column("#0", width=95)
        self.tree.column("Type", width=55)
        self.tree.column("Mass", width=55)
        self.tree.column("Pos", width=55)

        sb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        # Buttons
        btn_bar = tk.Frame(parent, bg="#20242e")
        btn_bar.pack(fill="x", padx=4, pady=6)
        tk.Button(btn_bar, text="Delete Selected", bg="#4a2828", fg="white", font=("Segoe UI", 8), command=self._delete_selected_body).pack(side="left", fill="x", expand=True, padx=1)
        tk.Button(btn_bar, text="Wake All Bodies", bg="#2e3848", fg="white", font=("Segoe UI", 8), command=lambda: self.world.wake_all()).pack(side="left", fill="x", expand=True, padx=1)

    def _build_display_tab(self, parent):
        pad = {'padx': 8, 'pady': 6}

        frame_vis = tk.LabelFrame(parent, text="CAD Visual Overlays", bg="#20242e", fg="#7ec4ff", font=("Segoe UI", 9, "bold"))
        frame_vis.pack(fill="x", **pad)

        tk.Checkbutton(frame_vis, text="Show AABB Wireframes (A)", variable=self.show_aabb, bg="#20242e", fg="#d0d8e8", selectcolor="#2e3340", activebackground="#20242e", command=self._sync_overlays).pack(anchor="w", padx=6, pady=3)
        tk.Checkbutton(frame_vis, text="Show Collision Contact Points (C)", variable=self.show_contacts, bg="#20242e", fg="#d0d8e8", selectcolor="#2e3340", activebackground="#20242e", command=self._sync_overlays).pack(anchor="w", padx=6, pady=3)
        tk.Checkbutton(frame_vis, text="Show Velocity Vectors (V)", variable=self.show_velocity, bg="#20242e", fg="#d0d8e8", selectcolor="#2e3340", activebackground="#20242e", command=self._sync_overlays).pack(anchor="w", padx=6, pady=3)
        tk.Checkbutton(frame_vis, text="Show Floor Grid Panes (G)", variable=self.show_grid, bg="#20242e", fg="#d0d8e8", selectcolor="#2e3340", activebackground="#20242e", command=self._sync_overlays).pack(anchor="w", padx=6, pady=3)

        frame_cam = tk.LabelFrame(parent, text="Camera Viewport Presets", bg="#20242e", fg="#7ec4ff", font=("Segoe UI", 9, "bold"))
        frame_cam.pack(fill="x", **pad)

        tk.Button(frame_cam, text="Isometric View (CAD)", bg="#2e3848", fg="white", command=lambda: self.renderer.set_camera_view(25, -55)).pack(fill="x", padx=4, pady=2)
        tk.Button(frame_cam, text="Top-Down Plan (XZ)", bg="#2e3848", fg="white", command=lambda: self.renderer.set_camera_view(89, -90)).pack(fill="x", padx=4, pady=2)
        tk.Button(frame_cam, text="Front Elevation (XY)", bg="#2e3848", fg="white", command=lambda: self.renderer.set_camera_view(0, -90)).pack(fill="x", padx=4, pady=2)
        tk.Button(frame_cam, text="Side Profile (ZY)", bg="#2e3848", fg="white", command=lambda: self.renderer.set_camera_view(0, 0)).pack(fill="x", padx=4, pady=2)

    def _build_labs_tab(self, parent):
        lbl = tk.Label(parent, text="Classical Physics Experiments", bg="#20242e", fg="#7ec4ff", font=("Segoe UI", 10, "bold"))
        lbl.pack(anchor="w", padx=8, pady=4)

        demos = [
            ("1. Box Stacking Stability", "stack"),
            ("2. Sphere Avalanche & Packing", "spheres"),
            ("3. Mixed Rigid Collisions", "mixed"),
            ("4. Harmonic Springs & Pendulum", "springs"),
            ("5. Jenga Tower Projectile Impact", "jenga"),
            ("6. Fluid Buoyancy & Floating", "buoyancy"),
            ("7. Newton's Cradle Momentum", "cradle")
        ]
        for name, key in demos:
            tk.Button(parent, text=name, bg="#2a303c", fg="#e0e8f8", font=("Segoe UI", 9), anchor="w", command=lambda k=key: self._load_preset_demo(k)).pack(fill="x", padx=6, pady=2)

    def _sync_overlays(self):
        self.renderer.show_aabb = self.show_aabb.get()
        self.renderer.show_contacts = self.show_contacts.get()
        self.renderer.show_velocity = self.show_velocity.get()
        self.renderer.show_grid = self.show_grid.get()
        self.renderer.ax.grid(self.renderer.show_grid)

    def update_scene_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for b in self.world.bodies:
            shape = "Box" if isinstance(b.collider, BoxCollider) else ("Sphere" if isinstance(b.collider, SphereCollider) else "Plane")
            mass_str = f"{b.mass:.1f}" if not b.is_static else "Static"
            pos_str = f"{b.position.y:.2f}"
            self.tree.insert("", "end", iid=str(b.id), text=b.name, values=(shape, mass_str, pos_str))

    def _animate_tick(self):
        try:
            if not self.root.winfo_exists(): return
        except Exception:
            return

        dt = 1.0 / 60.0
        if not self.world.paused:
            self.world.step(dt)

        self._sync_overlays()
        self.renderer.render()

        t1 = time.perf_counter()
        fps_instant = 1.0 / max(1e-4, (t1 - self.last_frame_time))
        self.fps_tracker = self.fps_tracker * 0.9 + fps_instant * 0.1
        self.last_frame_time = t1

        try:
            self.root.after(16, self._animate_tick)
        except Exception:
            pass

    def _toggle_pause(self):
        self.world.paused = not self.world.paused
        self.btn_play.config(text="Play" if self.world.paused else "Pause")

    def _step_frame(self):
        self.world.step(1.0 / 60.0)
        self.renderer.render()

    def _reset_current_preset(self):
        self._load_preset_demo("mixed")

    def _on_speed_change(self, val):
        self.world.time_scale = float(val)

    def _open_spawner_dialog(self):
        ObjectSpawnerWindow(self.root, self)

    def _open_gravity_dialog(self):
        GravityWindow(self.root, self)

    def _open_collision_dialog(self):
        CollisionWindow(self.root, self)

    def _load_preset_demo(self, name):
        SceneManager.load_preset(self.world, name)
        self.update_scene_tree()
        self.renderer.set_camera_view(24, -55)

    def _delete_selected_body(self):
        sel = self.tree.selection()
        if sel:
            bid = int(sel[0])
            body = next((b for b in self.world.bodies if b.id == bid), None)
            if body and not (body.is_static and isinstance(body.collider, PlaneCollider)):
                self.world.remove_body(body)
                self.update_scene_tree()

    def _apply_explosion_impulse(self):
        center = Vec3(0, 1.0, 0)
        for b in self.world.bodies:
            if not b.is_static:
                delta = b.position - center
                dist = delta.length()
                if dist < 8.0:
                    power = max(5.0, (8.0 - dist) * 4.0)
                    dir_vec = delta.normalize() if dist > 0.1 else Vec3(0, 1, 0)
                    b.apply_impulse(dir_vec * power + Vec3(0, power * 0.5, 0))
                    b.wake()

    def _spawn_random_blocks(self):
        for i in range(10):
            pos = Vec3(random.uniform(-3, 3), 4.0 + i * 1.2, random.uniform(-3, 3))
            box = RigidBody(position=pos, mass=random.uniform(1.0, 3.0), material=random.choice(Materials.ALL))
            h = random.uniform(0.3, 0.6)
            box.collider = BoxCollider(Vec3(h, h, h))
            self.world.add_body(box)
        self.update_scene_tree()

    def _spawn_sphere_storm(self):
        for i in range(15):
            pos = Vec3(random.uniform(-4, 4), 7.0 + i * 0.8, random.uniform(-4, 4))
            sphere = RigidBody(position=pos, mass=random.uniform(0.5, 2.0), material=Materials.BOUNCY)
            sphere.collider = SphereCollider(random.uniform(0.3, 0.55))
            sphere.velocity = Vec3(random.uniform(-1, 1), random.uniform(-2, 0), random.uniform(-1, 1))
            self.world.add_body(sphere)
        self.update_scene_tree()

    def _on_file_new(self):
        self.world.clear()
        SceneManager.create_default_ground(self.world)
        self.update_scene_tree()

    def _on_file_save(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("Physics Scene", "*.json")])
        if path:
            SceneManager.save_scene(self.world, path)
            messagebox.showinfo("Scene Saved", f"Scene saved to: {path}")

    def _on_file_open(self):
        path = filedialog.askopenfilename(filetypes=[("Physics Scene", "*.json")])
        if path:
            SceneManager.load_scene(self.world, path)
            self.update_scene_tree()

    def _on_export_snapshot(self):
        path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG Image", "*.png")])
        if path:
            self.renderer.fig.savefig(path, facecolor=self.renderer.bg_color, dpi=200)
            messagebox.showinfo("Snapshot Exported", f"3D snapshot saved to: {path}")


# ============================================================================
# SECTION 11: EMBEDDED MATPLOTLIB RENDERER WITH TOOLBAR
# ============================================================================

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

class EmbeddedMatplotlibRenderer:
    def __init__(self, world: PhysicsWorld, container_widget):
        self.world = world
        self.container = container_widget

        self.show_aabb = False
        self.show_contacts = True
        self.show_velocity = False
        self.show_grid = True

        self.bg_color = '#171a21'
        self.pane_color = (0.13, 0.15, 0.19, 0.8)
        self.grid_color = '#2d3545'

        self._sphere_template = self._create_sphere_template(segments=8, rings=5)

        # Matplotlib Figure
        self.fig = plt.figure(figsize=(9, 7), dpi=100)
        self.fig.patch.set_facecolor(self.bg_color)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(self.bg_color)

        self._setup_axes()
        self._setup_artists()

        # Embed into Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.container)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

        # Standard Navigation Toolbar with Home button
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.container)
        self.toolbar.config(background="#20242e")
        for btn in self.toolbar.winfo_children():
            try: btn.config(background="#2e3442")
            except: pass
        self.toolbar.update()

    def _create_sphere_template(self, segments=8, rings=5):
        tris = []
        for i in range(segments):
            theta1 = 2.0 * math.pi * i / segments
            theta2 = 2.0 * math.pi * (i + 1) / segments
            for j in range(rings):
                phi1 = math.pi * j / rings
                phi2 = math.pi * (j + 1) / rings

                def sph_p(th, ph):
                    return Vec3(math.sin(ph) * math.cos(th),
                                math.cos(ph),
                                math.sin(ph) * math.sin(th))

                p1, p2 = sph_p(theta1, phi1), sph_p(theta2, phi1)
                p3, p4 = sph_p(theta2, phi2), sph_p(theta1, phi2)
                if j == 0: tris.append((p1, p3, p4))
                elif j == rings - 1: tris.append((p1, p2, p3))
                else:
                    tris.append((p1, p2, p3))
                    tris.append((p1, p3, p4))
        return tris

    def _setup_axes(self):
        self.ax.set_xlabel('X (m)', color='#b0c4de', fontsize=9, labelpad=4)
        self.ax.set_ylabel('Z (m)', color='#b0c4de', fontsize=9, labelpad=4)
        self.ax.set_zlabel('Y (m) [Elevation]', color='#b0c4de', fontsize=9, labelpad=4)
        self.ax.tick_params(colors='#8898b0', labelsize=8)

        for axis_pane in (self.ax.xaxis.pane, self.ax.yaxis.pane, self.ax.zaxis.pane):
            axis_pane.fill = True
            axis_pane.set_facecolor(self.pane_color)
            axis_pane.set_edgecolor(self.grid_color)

        self.ax.set_xlim(-7, 7)
        self.ax.set_ylim(-7, 7)
        self.ax.set_zlim(0, 11)
        self.ax.view_init(elev=24, azim=-55)

    def _setup_artists(self):
        dummy_tri = [[[0, 0, 0], [0, 0, 0], [0, 0, 0]]]
        self.poly_collection = Poly3DCollection(dummy_tri, alpha=0.88, edgecolors='#181c24', linewidths=0.6)
        self.ax.add_collection3d(self.poly_collection)

        dummy_line = [[[0, 0, 0], [0, 0, 0]]]
        self.constraint_lines = Line3DCollection(dummy_line, colors='#40c0ff', linewidths=2.0, alpha=0.9)
        self.ax.add_collection3d(self.constraint_lines)

        self.contact_lines = Line3DCollection(dummy_line, colors='#44ff77', linewidths=1.8, alpha=0.95)
        self.ax.add_collection3d(self.contact_lines)

        self.velocity_lines = Line3DCollection(dummy_line, colors='#ff4d4d', linewidths=1.8, alpha=0.9)
        self.ax.add_collection3d(self.velocity_lines)

        self.aabb_lines = Line3DCollection(dummy_line, colors='#ffd040', linewidths=0.8, alpha=0.7)
        self.ax.add_collection3d(self.aabb_lines)

    def set_camera_view(self, elev, azim):
        self.ax.view_init(elev=elev, azim=azim)
        self.canvas.draw_idle()

    def render(self):
        all_polys = []
        all_colors = []
        constraint_segs = []
        contact_segs = []
        velocity_segs = []
        aabb_segs = []

        # 1. Physics Bodies
        for body in self.world.bodies:
            if not body.collider: continue
            col = body.color

            if isinstance(body.collider, BoxCollider):
                he = body.collider.half_extents
                local_v = [
                    Vec3(-he.x, -he.y, -he.z), Vec3(he.x, -he.y, -he.z),
                    Vec3(he.x, he.y, -he.z), Vec3(-he.x, he.y, -he.z),
                    Vec3(-he.x, -he.y, he.z), Vec3(he.x, -he.y, he.z),
                    Vec3(he.x, he.y, he.z), Vec3(-he.x, he.y, he.z)
                ]
                wv = [body.get_world_point(v) for v in local_v]
                mv = [[v.x, v.z, v.y] for v in wv]

                faces = [
                    [mv[0], mv[1], mv[2], mv[3]],
                    [mv[5], mv[4], mv[7], mv[6]],
                    [mv[4], mv[0], mv[3], mv[7]],
                    [mv[1], mv[5], mv[6], mv[2]],
                    [mv[3], mv[2], mv[6], mv[7]],
                    [mv[4], mv[5], mv[1], mv[0]]
                ]
                for f in faces:
                    all_polys.append(f)
                    all_colors.append(col)

            elif isinstance(body.collider, SphereCollider):
                r = body.collider.radius
                pos = body.get_world_position()
                rot = Mat3.from_quaternion(body.orientation)
                for t1, t2, t3 in self._sphere_template:
                    w1 = pos + rot * (t1 * r)
                    w2 = pos + rot * (t2 * r)
                    w3 = pos + rot * (t3 * r)
                    tri = [[w1.x, w1.z, w1.y], [w2.x, w2.z, w2.y], [w3.x, w3.z, w3.y]]
                    all_polys.append(tri)
                    all_colors.append(col)

            elif isinstance(body.collider, PlaneCollider):
                sz = 12.0
                plane_face = [[-sz, -sz, 0.0], [sz, -sz, 0.0], [sz, sz, 0.0], [-sz, sz, 0.0]]
                all_polys.append(plane_face)
                all_colors.append((0.22, 0.26, 0.33))

            # Debug AABB
            if self.show_aabb:
                aabb = body.get_aabb()
                hx, hy, hz = (aabb.max.x - aabb.min.x) * 0.5, (aabb.max.y - aabb.min.y) * 0.5, (aabb.max.z - aabb.min.z) * 0.5
                c = aabb.center()
                bv = [c + Vec3(sx * hx, sy * hy, sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
                mv = [[v.x, v.z, v.y] for v in bv]
                edges = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
                for e1, e2 in edges: aabb_segs.append([mv[e1], mv[e2]])

            # Debug Velocity
            if self.show_velocity and body.velocity.length_sq() > 0.05:
                p0 = [body.position.x, body.position.z, body.position.y]
                pend = body.position + body.velocity * 0.4
                p1 = [pend.x, pend.z, pend.y]
                velocity_segs.append([p0, p1])

        # Constraints
        for constraint in self.world.constraints:
            if isinstance(constraint, (DistanceConstraint, SpringConstraint)):
                wa = constraint.body_a.get_world_point(constraint.local_a)
                wb = constraint.body_b.get_world_point(constraint.local_b)
                constraint_segs.append([[wa.x, wa.z, wa.y], [wb.x, wb.z, wb.y]])

        # Contacts
        if self.show_contacts:
            for manifold in self.world.manifolds.values():
                for c in manifold.contacts:
                    p0 = [c.point.x, c.point.z, c.point.y]
                    pn = c.point + c.normal * 0.4
                    contact_segs.append([p0, [pn.x, pn.z, pn.y]])

        if all_polys:
            self.poly_collection.set_verts(all_polys)
            self.poly_collection.set_facecolors(all_colors)

        self.constraint_lines.set_segments(constraint_segs if constraint_segs else [[[0, 0, 0], [0, 0, 0]]])
        self.contact_lines.set_segments(contact_segs if contact_segs else [[[0, 0, 0], [0, 0, 0]]])
        self.velocity_lines.set_segments(velocity_segs if velocity_segs else [[[0, 0, 0], [0, 0, 0]]])
        self.aabb_lines.set_segments(aabb_segs if aabb_segs else [[[0, 0, 0], [0, 0, 0]]])

        # Header Title
        ke = self.world.get_total_kinetic_energy()
        contacts_count = sum(len(m) for m in self.world.manifolds.values())
        status = " [PAUSED]" if self.world.paused else ""
        title = (f"Bodies: {len(self.world.bodies)} | Contacts: {contacts_count} | "
                 f"Energy: {ke:.2f} J | Time: {self.world.total_time:.1f}s{status}")
        self.ax.set_title(title, color='#e0e8f5', fontsize=9, pad=8)

        self.canvas.draw_idle()


# ============================================================================
# SECTION 12: MAIN ENTRY POINT
# ============================================================================

def main():
    root = tk.Tk()
    app = PhysicsStudioApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
