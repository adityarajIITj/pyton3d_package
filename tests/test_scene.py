import os
import tempfile
import pytest
from pyton3d import PhysicsWorld, RigidBody, BoxCollider, SphereCollider, Materials, SceneManager, Vec3

def test_scene_save_and_load():
    world = PhysicsWorld(gravity=Vec3(0, -9.81, 0))
    b1 = RigidBody(position=Vec3(1, 2, 3), mass=2.5, name="Box1")
    b1.collider = BoxCollider(Vec3(0.5, 0.5, 0.5))
    b1.material = Materials.WOOD
    world.add_body(b1)

    b2 = RigidBody(position=Vec3(-1, 4, 0), mass=1.2, name="Sphere1")
    b2.collider = SphereCollider(0.4)
    b2.material = Materials.STEEL
    world.add_body(b2)

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
        temp_path = tf.name

    try:
        SceneManager.save_scene(world, temp_path)
        assert os.path.exists(temp_path)

        new_world = PhysicsWorld()
        SceneManager.load_scene(new_world, temp_path)

        # Ground + 2 bodies = 3 bodies total
        assert len(new_world.bodies) == 3
        names = {b.name for b in new_world.bodies}
        assert "Box1" in names
        assert "Sphere1" in names
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)

def test_all_presets_load_and_step():
    presets = ["stack", "spheres", "mixed", "springs", "jenga", "buoyancy", "cradle"]
    for preset in presets:
        world = PhysicsWorld()
        SceneManager.load_preset(world, preset)
        assert len(world.bodies) > 0
        # Step simulation 10 frames to ensure stability
        for _ in range(10):
            world.step(1/60)
