import pytest
import math
from pyton3d import Vec3, Mat3, Quaternion

def test_vec3_operations():
    a = Vec3(1, 2, 3)
    b = Vec3(4, 5, 6)
    assert a + b == Vec3(5, 7, 9)
    assert b - a == Vec3(3, 3, 3)
    assert a * 2.0 == Vec3(2, 4, 6)
    assert a.dot(b) == 32.0
    c = a.cross(b)
    assert c == Vec3(-3, 6, -3)
    assert c.dot(a) == pytest.approx(0.0, abs=1e-6)
    assert c.dot(b) == pytest.approx(0.0, abs=1e-6)

def test_quaternion_rodrigues():
    q = Quaternion.from_axis_angle(Vec3(0, 1, 0), math.pi * 0.5)
    v = Vec3(1, 0, 0)
    v_rot = q.rotate_vector(v)
    assert v_rot.x == pytest.approx(0.0, abs=1e-5)
    assert v_rot.y == pytest.approx(0.0, abs=1e-5)
    assert v_rot.z == pytest.approx(-1.0, abs=1e-5)

def test_mat3_inverse():
    m = Mat3([[2, 0, 0], [0, 3, 0], [0, 0, 4]])
    inv = m.inverse()
    assert inv.m00 == pytest.approx(0.5)
    assert inv.m11 == pytest.approx(1/3)
    assert inv.m22 == pytest.approx(0.25)
