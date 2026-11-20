import csv
from math import ceil, isfinite, radians

import mujoco
import numpy as np

SERVO = 10
FIELDS = ("q", "dq", "kp", "kd", "tau")
JOINTS = [
    f"{leg}_{joint}_joint"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
]

MAX_STEP = radians(4.9999)
JUDGE_STEP = radians(0.1)
PENETRATION_LIMIT = 0.0008  # м, с запасом до порога 1 мм


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


class Safety:
    def __init__(self, model_dir):
        self.model = mujoco.MjModel.from_xml_path(str(model_dir / "scene.xml"))
        self.data = mujoco.MjData(self.model)

        joint_ids = [self.model.joint(name).id for name in JOINTS]
        self.addresses = self.model.jnt_qposadr[joint_ids]
        limits = self.model.jnt_range[joint_ids].astype(np.float32)
        self.lower = np.nextafter(limits[:, 0], np.float32(np.inf))
        self.upper = np.nextafter(limits[:, 1], np.float32(-np.inf))
        actuator_ids = [self.model.actuator(name.removesuffix("_joint")).id for name in JOINTS]
        self.max_tau = self.model.actuator_forcerange[actuator_ids, 1]

        self.pairs = set()
        with (model_dir / "t2_collision_pairs.csv").open(newline="") as table:
            for row in csv.DictReader(table):
                if row["diagnostic_policy"] == "checked":
                    a = int(row["geom1_id"])
                    b = int(row["geom2_id"])
                    self.pairs.add((min(a, b), max(a, b)))
        if len(self.pairs) != 655 or max(b for _, b in self.pairs) >= self.model.ngeom:
            raise ValueError("Таблица коллизий не соответствует модели T2")

        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.target = self.data.qpos[self.addresses].astype(np.float32)
        if not self.pose_is_safe(self.target):
            raise ValueError("В исходной позе Go1 есть самоколлизия")
        self.command = self.hold()

    def pose_is_safe(self, q):
        self.data.qpos[self.addresses] = q
        mujoco.mj_fwdPosition(self.model, self.data)
        for contact in self.data.contact:
            a, b = contact.geom1, contact.geom2
            if contact.dist < -PENETRATION_LIMIT and (min(a, b), max(a, b)) in self.pairs:
                return False
        return True

    def transition_is_safe(self, target):
        start = self.target.astype(np.float64)
        delta = target.astype(np.float64) - start
        steps = 2 * max(1, ceil(np.max(np.abs(delta)) / JUDGE_STEP))
        for i in range(1, steps + 1):
            if not self.pose_is_safe(start + delta * (i / steps)):
                return False
        return True

    def accept(self, motors):
        if len(motors) != 20:
            return False
        for motor in motors:
            if not all(isfinite(getattr(motor, field)) for field in FIELDS):
                return False

        target = []
        command = []
        for i, motor in enumerate(motors[:12]):
            if motor.mode != SERVO:
                return False
            previous = float(self.target[i])
            q = clamp(motor.q, self.lower[i], self.upper[i])
            q = np.float32(clamp(q, previous - MAX_STEP, previous + MAX_STEP))
            target.append(q)
            command.append((
                q,
                clamp(motor.dq, -30, 30),
                clamp(motor.kp, 0, 100),
                clamp(motor.kd, 0, 10),
                clamp(motor.tau, -self.max_tau[i], self.max_tau[i]),
            ))

        target = np.asarray(target, dtype=np.float32)
        if not np.array_equal(target, self.target):
            if not self.transition_is_safe(target):
                return False
        self.target = target
        self.command = command
        return True

    def hold(self):
        return [(q, 0.0, 40.0, 2.0, 0.0) for q in self.target]
