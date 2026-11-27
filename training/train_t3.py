import argparse
import csv
from concurrent.futures import ProcessPoolExecutor
from functools import partial
import json
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from t2_safety import Safety
from t3_policy import HOME, KD, KP, PERIOD, Policy, angles

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "vendor/go1-model"
WEIGHTS = ROOT / "tasks/t3_weights.json"
SPEED = .22
SEED = 5321
CENTER = [1.063, 1.4455, .01234, .31692, .78989, .02013, .23041, .4092, .03174, .00882]
SPREAD = [.19, .28, .006, .007, .035, .065, .065, .18, .09, .07]
BOUNDS = [[.55, 1.85], [.6, 2.4], [.004, .045], [.285, .34],
          [.69, .90], [-.25, .38], [-.25, .38], [0., 1.6], [-.35, .35], [0., .32]]
POPULATION, ELITES, GENERATIONS = 28, 6, 10


class Environment:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.model.actuator_gainprm[:, 0] = 0
        self.model.actuator_biasprm[:] = 0
        self.limits = np.tile([23.7, 23.7, 35.55], 4)
        self.key = self.model.key("home").id
        self.safety = Safety(MODEL)

    def episode(self, weights, seed, seconds, safety=False, stop_at=None):
        rng = np.random.default_rng(seed)
        d, m = self.data, self.model
        mujoco.mj_resetDataKeyframe(m, d, self.key)
        d.qpos[7:] += rng.uniform(-.018, .018, 12)
        d.qvel[:] = rng.uniform(-.025, .025, m.nv)
        mujoco.mj_forward(m, d)
        policy = Policy(weights)
        if safety:
            self.safety.target = HOME.astype(np.float32)
            self.safety.command = self.safety.hold()
        q_ref = HOME.copy()
        start = d.qpos[:2].copy()
        unsettled = 0.
        reward = 0.
        steps = round(PERIOD / m.opt.timestep)
        dt = steps * m.opt.timestep
        for i in range(round((seconds + 1.5) / dt)):
            warmup = d.time < 1.5
            if warmup:
                start = d.qpos[:2].copy()
            desired = policy.step(0. if warmup else SPEED, d.qpos[3:7],
                                  d.qvel[3:6], dt)
            if safety:
                motors = [SimpleNamespace(mode=10, q=float(q), dq=0., tau=0., kp=KP, kd=KD)
                          for q in np.r_[desired, np.zeros(8)]]
                self.safety.accept(motors)
                q_ref = self.safety.target
            else:
                q_ref += np.clip(desired - q_ref, -.08726, .08726)
            for _ in range(steps):
                tau = KP * (q_ref - d.qpos[7:]) - KD * d.qvel[6:]
                d.qfrc_applied[6:] = np.clip(tau, -self.limits, self.limits)
                mujoco.mj_step(m, d)
            roll, pitch, yaw = angles(d.qpos[3:7])
            low = d.qpos[2] < .16 or abs(roll) > np.radians(50) or abs(pitch) > np.radians(50)
            unsettled = unsettled + dt if low else 0.
            if unsettled > .5:
                return float(reward - 12), float(d.qpos[0] - start[0]), d.time - 1.5, True
            displacement = d.qpos[:2] - start
            if not warmup:
                reward += dt * (8.0 * np.clip(d.qvel[0], -.6, .6)
                                - 1.1 * abs(d.qvel[0] - SPEED)
                                - .5 * abs(d.qvel[1]) - .35 * abs(yaw)
                                - .8 * (abs(roll) + abs(pitch))
                                - .00015 * np.sum(tau**2))
            if stop_at is not None and not low and displacement[0] >= stop_at:
                break
        return float(reward), float(d.qpos[0] - start[0]), d.time - 1.5, False


def train(env):
    config = dict(seed=SEED, speed_mps=SPEED, population=POPULATION, elites=ELITES,
                  generations=GENERATIONS, center=CENTER, spread=SPREAD, bounds=BOUNDS,
                  rollout_seconds=12, training_filter="joint step limit",
                  eval_filter="full T2", kp=KP, kd=KD, control_period=PERIOD)
    (ROOT / "training/config.json").write_text(json.dumps(config, indent=2) + "\n")
    rng = np.random.default_rng(SEED)
    mean, spread = np.array(CENTER), np.array(SPREAD)
    lower, upper = np.array(BOUNDS).T
    best_score, best = -float("inf"), mean.copy()
    with (ROOT / "training/training.csv").open("w", newline="") as log:
        out = csv.writer(log)
        out.writerow(["generation", "candidate", "seed", "reward", "forward_m",
                      "seconds", "fell", "weights"])
        for gen in range(GENERATIONS):
            batch = np.clip(rng.normal(mean, spread, (POPULATION, len(mean))), lower, upper)
            batch[0] = best
            scores = []
            for idx, weights in enumerate(batch):
                seed = SEED + gen
                result = env.episode(weights, seed, 12)
                out.writerow([gen, idx, seed, *result, json.dumps(weights.tolist())])
                scores.append(result[0])
            order = np.argsort(scores)[-ELITES:]
            if scores[order[-1]] > best_score:
                best_score, best = scores[order[-1]], batch[order[-1]].copy()
            mean = batch[order].mean(axis=0)
            spread = np.maximum(batch[order].std(axis=0), np.array(SPREAD) * .19)
            log.flush()
            print(f"Generation {gen + 1}: best reward {max(scores):.3f}", flush=True)
    WEIGHTS.write_text(json.dumps(dict(speed_mps=SPEED, weights=best.tolist()), indent=2) + "\n")


def evaluate_episode(weights, seed):
    result = Environment().episode(weights, seed, 90, safety=True, stop_at=3.)
    return (seed, *result, result[1] >= 3. and not result[3])


def evaluate(episodes, jobs):
    weights = json.loads(WEIGHTS.read_text())["weights"]
    with (ROOT / "training/evaluation.csv").open("w", newline="") as log:
        out = csv.writer(log)
        out.writerow(["seed", "reward", "forward_m", "seconds_after_warmup", "fell", "success"])
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            seeds = range(9030, 9030 + episodes)
            for result in pool.map(partial(evaluate_episode, weights), seeds):
                out.writerow(result)
                log.flush()
                print(*result, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--evaluate", type=int, default=0)
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    if args.evaluate:
        evaluate(args.evaluate, args.jobs)
    else:
        train(Environment())
