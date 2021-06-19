import numpy as np
from gym import utils
from gym.envs.mujoco.mujoco_env import MujocoEnv
import os
from scipy.spatial.transform import Rotation

"""
NAVI Model

    Diagram
    -------
               -------
        (Rear) |     | (Front)
          [1]  -------   [0]
             (0)/  \ (2)
            (1)/    \(3)
              /      \  at ground



    Action Space
    ------------

        action = np.array([0.0, 0.0, # Front leg [0]
                            0.0, 0.0]) # Rear leg [1]

    Position limits
    ---------------

        Action limits for all actuators: [-1 1]

        Right Hip: 0~90 degrees ==> 0~pi/2 ==>  1 ctr/(pi rad) * (pi/3 rad)  ==> 0~1/2
            [-1/3 1/3]
        Femur: +/-90 degrees ==> pi/2 ==>  1 ctr/(pi rad) * (pi/2 rad)  ==> 1/2
            [-1/2 1/2]
        Tibia: Same as Femur
            [-1/2 1/2]

    length
    ------

    0.0558
    0.39152
    0.39857
    total : 0.84589

    0.05 + 0.05between right and left

"""


class NabiSlopeEnv(MujocoEnv, utils.EzPickle):
    def __init__(self, **kwargs):

        self.MAPPING = {0: [0, 1],
                        1: [2, 3]}

        self.REST_POSE = np.array([-1/4, 1/8, 1/4, -1/8])

        self.RIGHT_HIP_INDEX = 0
        self.RIGHT_KNEE_INDEX = 1
        self.LEFT_HIP_INDEX = 2
        self.LEFT_KNEE_INDEX = 3

        self.pos = self.REST_POSE.copy()
        self.reward = 0
        self.numofmotor = 4

        self.debug = kwargs["debug"]

        xml_path = os.path.dirname(
            os.path.abspath(__file__)) + '/../../envs/model/NabiSlope-v0.xml'
        frame_skip = 1
        MujocoEnv.__init__(self, xml_path, frame_skip)
        utils.EzPickle.__init__(self)

    def feed_action(self, a):
        self.pos = a

        self.pos[self.RIGHT_HIP_INDEX] = np.clip(
            self.pos[self.RIGHT_HIP_INDEX], -1, 1)
        self.pos[self.LEFT_HIP_INDEX] = np.clip(
            self.pos[self.LEFT_HIP_INDEX], -1, 1)

        self.pos[self.RIGHT_KNEE_INDEX] = np.clip(
            self.pos[self.RIGHT_KNEE_INDEX], -1, 1)
        self.pos[self.LEFT_KNEE_INDEX] = np.clip(
            self.pos[self.LEFT_KNEE_INDEX], -1, 1)

        return self.pos

    def advance(self):
        self.pos = self.REST_POSE.copy()
        # self.reward = 0.0

    def step(self, a):
        xposbefore = self.get_body_com("base_link")[0]
        yposbefore = self.get_body_com("base_link")[1]
        zposbefore = self.get_body_com("base_link")[2]

        a = self.feed_action(a)

        if self.debug:
            print("Before:", self.sim.data.qpos[:3])
            print("Action:", a)

        self.do_simulation(a, self.frame_skip)

        xposafter = self.get_body_com("base_link")[0]
        yposafter = self.get_body_com("base_link")[1]
        zposafter = self.get_body_com("base_link")[2]

        if self.debug:
            print("After:", self.sim.data.qpos[:3])
            print("==" * 25)

        # because the robot is walking in y+ direction
        forward_vel = (yposafter - yposbefore) / self.dt
        forward_reward = 10 * forward_vel

        not_x_reward = 10 * abs(xposafter - xposbefore) / self.dt
        actuator_cost = abs(self.data.actuator_velocity[:]).sum()

        weighting = 0.0008
        survive_reward = 3.

        reward = forward_reward + survive_reward - \
            not_x_reward - weighting * actuator_cost
        self.reward = reward

        done = zposafter <= 0.2  # Max height of robot: 0.84 | Min Height of robot: 0.64
        obs = self._get_obs()
        self.advance()
        return obs, reward, done, dict(
            reward_forward=forward_reward,
            reward_actuator=-actuator_cost,
            reward_survive=survive_reward,
            reward_not_x_reward=-not_x_reward
        )

    def _get_obs(self):

        root_qpos = self.sim.data.get_joint_qpos("root")
        root_x = root_qpos[0]
        root_y = root_qpos[1]
        root_z = root_qpos[2]
        root_quat = root_qpos[3:].tolist()
        root_quat_scipy = root_quat[1:] + [root_quat[0]]
        root_euler = Rotation.from_quat(root_quat_scipy).as_euler('xyz')

        RH_pos = self.sim.data.get_joint_qpos(
            "Right_Hip_Joint")  # np.float scalar # angle
        RK_pos = self.sim.data.get_joint_qpos(
            "Right_Knee_Joint")  # np.float scalar # angle
        LH_pos = self.sim.data.get_joint_qpos(
            "Left_Hip_Joint")  # np.float scalar # angle
        LK_pos = self.sim.data.get_joint_qpos(
            "Left_Knee_Joint")  # np.float scalar # angle

        # np array: [linear vel along x,y,z, angular vel along x,y,z]
        root_vel = self.sim.data.get_joint_qvel("root")
        # np.float scalar # angular velocity
        RH_vel = self.sim.data.get_joint_qvel("Right_Hip_Joint")
        # np.float scalar # angular velocity
        RK_vel = self.sim.data.get_joint_qvel("Right_Knee_Joint")
        # np.float scalar # angular velocity
        LH_vel = self.sim.data.get_joint_qvel("Left_Hip_Joint")
        # np.float scalar # angular velocity
        LK_vel = self.sim.data.get_joint_qvel("Left_Knee_Joint")

        # Ignoring the position of root in the obs
        obs = np.array([
            root_x,  # 1
            root_y,  # 1
            root_z,  # 1
            *(root_euler.tolist()),  # 3
            # *root_quat, # 4
            RH_pos,  # 1
            RK_pos,  # 1
            LH_pos,  # 1
            LK_pos,  # 1
            *(root_vel.tolist()),  # 6
            RH_vel,  # 1
            RK_vel,  # 1
            LH_vel,  # 1
            LK_vel,  # 1
            # 1 # bias term. Adding randomness to avoid Nan in MeanStdFilter
            1 + np.random.randn() * 0.01
        ])  # total 21
        return obs

    def reset_model(self):
        qpos = self.init_qpos
        qvel = self.init_qvel
        self.set_state(qpos, qvel)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5
        self.viewer.cam.azimuth = 90
        self.viewer.cam.elevation = -14
