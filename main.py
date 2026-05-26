# main.py

import pybullet as p
import pybullet_data
import time
import numpy as np

from keyboard import (
    get_keyboard_input,
    print_controls,
    FlightState
)

# =========================================
# INIT
# =========================================

p.connect(p.GUI)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(0, 0, 0)

def load_obj_model(obj_path, position, rpy=[0,0,0]):
    quat = p.getQuaternionFromEuler(rpy)

    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=obj_path,
        meshScale=[1, 1, 1]
    )

    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=obj_path,
        meshScale=[1, 1, 1]
    )

    body = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
        baseOrientation=quat
    )

    return body

plane = load_obj_model("plane.obj", [-5, 0, 1])
rocket = load_obj_model("rocket.obj", [0, 0, 1])
plane_speed = 3.0

flight_state = FlightState()

print_controls()

# =========================================
# ROTATION
# =========================================

def rotate_to_velocity(body_id, velocity):

    velocity = np.array(velocity)

    speed = np.linalg.norm(velocity)

    if speed < 0.001:
        return

    direction = velocity / speed

    yaw = np.arctan2(
        direction[1],
        direction[0]
    )

    pitch = -np.arctan2(
        direction[2],
        np.sqrt(
            direction[0] ** 2 +
            direction[1] ** 2
        )
    )

    # Если модель смотрит боком:
    # yaw += np.pi / 2

    quaternion = p.getQuaternionFromEuler(
        [0, pitch, yaw]
    )

    position, _ = p.getBasePositionAndOrientation(
        body_id
    )

    # ВАЖНО:
    # Сначала сохраняем скорость
    linear_velocity, angular_velocity = (
        p.getBaseVelocity(body_id)
    )

    # Меняем только rotation
    p.resetBasePositionAndOrientation(
        body_id,
        position,
        quaternion
    )

    # Возвращаем скорость обратно
    p.resetBaseVelocity(
        body_id,
        linearVelocity=linear_velocity,
        angularVelocity=angular_velocity
    )

# =========================================
# MAIN LOOP
# =========================================

try:

    while True:

        # =================================
        # PLAYER PLANE
        # =================================

        direction = get_keyboard_input(
            flight_state
        )

        plane_velocity = [
            direction[0] * plane_speed,
            direction[1] * plane_speed,
            direction[2] * plane_speed
        ]

        p.resetBaseVelocity(
            plane,
            linearVelocity=plane_velocity
        )

        rotate_to_velocity(
            plane,
            plane_velocity
        )

        # =================================
        # POSITIONS
        # =================================

        rocket_pos, _ = (
            p.getBasePositionAndOrientation(
                rocket
            )
        )

        plane_pos, _ = (
            p.getBasePositionAndOrientation(
                plane
            )
        )

        # =================================
        # ROCKET GUIDANCE
        # =================================

        direction_to_target = (
            np.array(plane_pos) -
            np.array(rocket_pos)
        )

        distance = np.linalg.norm(
            direction_to_target
        )

        if distance > 0.001:
            direction_to_target = (
                direction_to_target /
                distance
            )

        rocket_speed = 5.0

        rocket_velocity = (
            direction_to_target *
            rocket_speed
        )

        p.resetBaseVelocity(
            rocket,
            linearVelocity=rocket_velocity.tolist()
        )

        rotate_to_velocity(
            rocket,
            rocket_velocity
        )

        # =================================
        # HIT CHECK
        # =================================

        if distance < 0.5:

            print("💥 HIT")

            break

        # =================================
        # STEP
        # =================================

        p.stepSimulation()

        time.sleep(1 / 60)

except KeyboardInterrupt:

    print("STOP")

finally:

    p.disconnect()