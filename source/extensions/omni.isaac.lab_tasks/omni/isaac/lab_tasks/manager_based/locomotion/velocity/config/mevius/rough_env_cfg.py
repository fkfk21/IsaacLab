# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab_assets.mevius import MEVIUS_CFG  # isort: skip

from omni.isaac.lab.utils.noise import AdditiveUniformNoiseCfg as Unoise

@configclass
class MeviusRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # switch robot to anymal-d
        self.scene.robot = MEVIUS_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")  # type: ignore
        self.scene.num_envs = 1024

        # scale down the terrains because the robot is small
        # self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)  # type: ignore
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)  # type: ignore
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01  # type: ignore

        # reduce action scale
        self.actions.joint_pos.scale = 0.25

        # event
        self.events.push_robot = None  # type: ignore
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 1.0)
        self.events.add_base_mass.params["asset_cfg"].body_names = "base"
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base"
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # commands
        # self.commands.base_velocity.heading_command = True
        # self.commands.base_velocity.ranges.lin_vel_x = (-0.7, 0.7)
        # self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        # self.commands.base_velocity.ranges.ang_vel_z = (-0.7, 0.7)
        # self.commands.base_velocity.ranges.heading = (-math.pi, math.pi)

        # observations
        # self.observations.policy.joint_pos.noise = Unoise(n_min=-0.05, n_max=0.05)
        # self.observations.policy.joint_vel.noise = Unoise(n_min=-1.5, n_max=1.5)
        # self.observations.policy.base_lin_vel.noise = Unoise(n_min=-0.2, n_max=0.2)
        # self.observations.policy.base_ang_vel.noise = Unoise(n_min=-0.3, n_max=0.3)
        # self.observations.policy.projected_gravity.noise = Unoise(n_min=-0.1, n_max=0.1)
        # self.observations.policy.height_scan.noise = Unoise(n_min=-0.1, n_max=0.1)

        # rewards
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"
        self.rewards.feet_air_time.weight = 0.01
        self.rewards.undesired_contacts = None
        # self.rewards.undesired_contacts.params["sensor_cfg"].body_names = ["base", ".*_thigh", ".*_scapula"]
        # self.rewards.undesired_contacts.weight = 0.0
        # # reward scales
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        # self.rewards.action_rate_l2.weight = -0.1
        # self.rewards.flat_orientation_l2.weight = -10.0
        # self.rewards.lin_vel_z_l2.weight = -10.0
        # self.rewards.ang_vel_xy_l2.weight = -0.1
        # self.rewards.feet_air_time.weight = 0.001
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.dof_acc_l2.weight = -2.5e-7
        # self.rewards.dof_vel_l2.weight = -1.0e-7
        # # self.rewards.stand_still.weight = -10.0
        # self.rewards.dof_pos_limits.weight = -10.0
        # self.rewards.undesired_contacts.weight = -100.0

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["base", ".*_thigh", ".*_scapula"]


@configclass
class MeviusRoughEnvCfg_PLAY(MeviusRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
