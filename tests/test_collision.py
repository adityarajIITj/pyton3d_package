import pytest
from pyton3d import SphereCollider, BoxCollider, PlaneCollider, CollisionDetector, RigidBody, Vec3

def test_sphere_sphere_collision():
    s1 = RigidBody(position=Vec3(0, 0, 0), mass=1.0)
    s1.collider = SphereCollider(1.0)
    s2 = RigidBody(position=Vec3(1.5, 0, 0), mass=1.0)
    s2.collider = SphereCollider(1.0)

    contact = CollisionDetector.detect(s1.collider, s2.collider)
    assert contact is not None
    assert contact.penetration == pytest.approx(0.5, abs=1e-5)
    assert contact.normal.x == pytest.approx(1.0, abs=1e-5)

def test_sphere_box_collision():
    box = RigidBody(position=Vec3(0, 0, 0), is_static=True)
    box.collider = BoxCollider(Vec3(1, 1, 1))
    sphere = RigidBody(position=Vec3(0, 1.5, 0), mass=1.0)
    sphere.collider = SphereCollider(1.0)

    contact = CollisionDetector.detect(box.collider, sphere.collider)
    assert contact is not None
    assert contact.penetration == pytest.approx(0.5, abs=1e-5)
