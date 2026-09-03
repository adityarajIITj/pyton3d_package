import pytest
from pyton3d import PhysicsWorld, RigidBody, SphereCollider, BoxCollider, DistanceConstraint, SpringConstraint, Vec3

def test_distance_constraint():
    world = PhysicsWorld(gravity=Vec3(0, -9.81, 0))
    anchor = RigidBody(position=Vec3(0, 5, 0), is_static=True)
    anchor.collider = SphereCollider(0.2)
    world.add_body(anchor)

    bob = RigidBody(position=Vec3(0, 3, 0), mass=1.0)
    bob.collider = SphereCollider(0.2)
    world.add_body(bob)

    dist_joint = DistanceConstraint(anchor, bob, Vec3.zero(), Vec3.zero(), distance=2.0)
    world.add_constraint(dist_joint)

    for _ in range(60):
        world.step(1/60)

    current_dist = (bob.position - anchor.position).length()
    assert current_dist == pytest.approx(2.0, abs=0.08)

def test_spring_constraint():
    world = PhysicsWorld(gravity=Vec3.zero())
    b1 = RigidBody(position=Vec3(0, 0, 0), mass=1.0)
    b2 = RigidBody(position=Vec3(3, 0, 0), mass=1.0)
    world.add_body(b1)
    world.add_body(b2)

    # Rest length is 1.0, current distance is 3.0 -> spring pulls them together
    spring = SpringConstraint(b1, b2, Vec3.zero(), Vec3.zero(), rest_length=1.0, stiffness=50.0, damping=0.5)
    world.add_constraint(spring)

    world.step(1/60)
    world.step(1/60)
    # b1 should have positive X acceleration, b2 should have negative X acceleration
    assert b1.velocity.x > 0.0
    assert b2.velocity.x < 0.0
