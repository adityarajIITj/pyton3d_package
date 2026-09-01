import pytest
from pyton3d import PhysicsWorld, RigidBody, BoxCollider, Materials, Vec3

def test_free_fall_gravity():
    world = PhysicsWorld(gravity=Vec3(0, -10.0, 0))
    body = RigidBody(position=Vec3(0, 10.0, 0), mass=2.0)
    body.collider = BoxCollider(Vec3(0.5, 0.5, 0.5))
    world.add_body(body)

    for _ in range(60):
        world.step(1/60)

    assert body.velocity.y < -8.0
    assert body.position.y < 6.0
