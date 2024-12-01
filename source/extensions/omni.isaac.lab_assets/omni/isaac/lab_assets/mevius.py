# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the ANYbotics robots.

The following configuration parameters are available:

* :obj:`ANYMAL_B_CFG`: The ANYmal-B robot with ANYdrives 3.0
* :obj:`ANYMAL_C_CFG`: The ANYmal-C robot with ANYdrives 3.0
* :obj:`ANYMAL_D_CFG`: The ANYmal-D robot with ANYdrives 3.0

Reference:

* https://github.com/ANYbotics/anymal_b_simple_description
* https://github.com/ANYbotics/anymal_c_simple_description
* https://github.com/ANYbotics/anymal_d_simple_description

"""

# from omni.isaac.lab.sensors.camera.camera_cfg import CameraCfg
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import DCMotorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

##
# Configuration - Actuators.
##

SIMPLE_ACTUATOR_CFG = DCMotorCfg(
    joint_names_expr=[".*_collar_joint", ".*_hip_joint", ".*_knee_joint"],
    saturation_effort=25.0,
    effort_limit=20.0,
    velocity_limit=7.5,
    stiffness={".*_collar_joint": 50.0, ".*_hip_joint": 50.0, ".*_knee_joint": 30.0},
    damping={".*_collar_joint": 2.0, ".*_hip_joint": 2.0, ".*_knee_joint": 2.0},
)
"""Configuration for mevius DC actuator model."""

ISAACLAB_TEST_DIR = "/home/fkfk21/core_ws/isaaclab_test"

##
# Configuration - Articulation.
##
MEVIUS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_TEST_DIR}/robots/mevius/mevius.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.33),  # x,y,z [m]
        joint_pos={  # = target angles [rad] when action = 0.0
            '[F,B]R_collar_joint': -0.1,
            '[F,B]L_collar_joint': -0.1,
            'F[R,L]_hip_joint': 0.8,
            'B[R,L]_hip_joint': 1.0,
            '.*knee_joint': -1.4,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={"legs": SIMPLE_ACTUATOR_CFG},
    soft_joint_pos_limit_factor=0.95,
)
"""Configuration of mevius robot using simple actuator config.

Note:
    Since we don't have a publicly available actuator network for ANYmal-D, we use the same network as ANYmal-C.
    This may impact the sim-to-real transfer performance.
"""


##
# Configuration - Sensors.
##
# MEVIUS_REALSENSE = CameraCfg(

# )
# """Configuration for the Realsense T265 mounted on the mevius robot's base."""
