import numpy as np
import utils.geom as geom
from flexible_ik_solver import flex_ik_py
import simulator.sim_util as sim_util

# try:
#     from juliacall import Main as jl
#     jl.seval("using FlexIK")
#     USE_JULIA_SOLVER = True
# except:
#     USE_JULIA_SOLVER = False
USE_JULIA_SOLVER = False

class BaseIKConfig:
    DEFAULT_HAND_POSE = {
            "pos": np.array([0.1, -0.3, 0.8]),
            "quat": np.array([0.0, 0.0, 0.0, 1.0]),
        }
    TIME_STEP = None
    URDF_PATH = None
    ARM_LINK_NAME = None


class ArmIKSolver(object):

    def __init__(self, config, sim, link_name, joint_names, **kwargs) -> None:

        self._time_step = config.TIME_STEP

        self._sim_muj = sim
        self._link_name = link_name
        self._joint_names = joint_names

        joint_ranges =sim_util.get_joint_range(sim, self._joint_names)
        joint_ranges = np.array([joint_ranges[joint] for joint in self._joint_names])
        self._min_q = joint_ranges[:, 0] + config.POS_LIMIT_MARGIN
        self._max_q = joint_ranges[:, 1] - config.POS_LIMIT_MARGIN

        self._max_qd = config.VEL_LIMIT_SCALING_FACTOR * np.array([config.MAX_Q_VEL] * config.ARM_DOFS_JOINT, dtype=np.float64)
        self._max_qdd = config.ACC_LIMIT_SCALING_FACTOR * np.array([config.MAX_Q_ACC] * config.ARM_DOFS_JOINT, dtype=np.float64)

        self._joint_indices_nullspace_bias = config.NULL_SPACE_JOINT_INDICES
        self._gains_nullspace_bias = [config.ARM_NULLSPACE_GAIN] * config.ARM_DOFS_JOINT
        self._pos_tracking_gain = config.POS_TRACKING_GAIN
        self._rot_tracking_gain = config.ROT_TRACKING_GAIN

        self._invSolver = flex_ik_py.ExtendedSingularityRobustSolver(1e-1, 1e-1)
        self._velIkSolver = flex_ik_py.VelIkSolver(self._invSolver)

        self._nsBiasJntProj = np.zeros((config.ARM_DOFS_JOINT, config.ARM_DOFS_JOINT), dtype=np.float64)
        self._selected_nullspace_bias_jnts = self._joint_indices_nullspace_bias
        self._nsBiasJntProj[self._selected_nullspace_bias_jnts, self._selected_nullspace_bias_jnts] = 1.0

    def solve(
        self, goal_hand_pose, goal_joint_pos, cur_hand_pose, cur_joint_pos, **kwargs
    ):

        cur_q = np.array(list(cur_joint_pos.values()), dtype=np.float64)
        goal_q = np.array(list(goal_joint_pos.values()), dtype=np.float64)

        # Construct the full joint state vector.
        err_hand = geom.get_pose_error_vector(cur_hand_pose, goal_hand_pose)

        # Set the constraints to prevent joint limit violation.
        self._velIkSolver.set_jnt_saturation_constraints(
            cur_q, self._min_q, self._max_q, self._max_qd, self._max_qdd, self._time_step
        )

        # Compute Jacobian.
        J_hand = sim_util.get_link_jacobian(self._sim_muj,
                                            link_name=self._link_name,
                                            joint_names=self._joint_names)

        # Scale the pose error with the gains.
        dxGoal = np.zeros_like(err_hand)
        dxGoal[:3] = self._pos_tracking_gain * err_hand[:3]
        dxGoal[3:] = self._rot_tracking_gain * err_hand[3:]

        # Compute nullspace bias to stay close to the initial configuration
        # (for the selected joints) in the secondary task objective.
        qdNsArm = self._gains_nullspace_bias * (goal_q - cur_q)
        qdNs = qdNsArm.reshape(-1, 1)

        # Construct the tasks.
        dxGoalData = [dxGoal, qdNs]
        JData = [J_hand, self._nsBiasJntProj]

        # Solve the velocity IK problem.
        d_q, s_data, _ = self._velIkSolver.solve(dxGoalData, JData)

        delta_q = d_q * self._time_step

        q_des = cur_q + delta_q

        return {
            "arm_joint": {key: value for key, value in zip(cur_joint_pos.keys(), q_des)}
        }



class TorsoIKSolver(ArmIKSolver):

    def __init__(self, config, sim, link_name, head_name, joint_names, **kwargs) -> None:

        super().__init__(config, sim, link_name, joint_names, **kwargs)

        self._head_name = head_name
        self._head_tracking_gain = config.HEAD_TRACKING_GAIN


    def solve(
        self, goal_hand_pose, goal_joint_pos, goal_head_pose, cur_hand_pose, cur_joint_pos, cur_head_pose, **kwargs
    ):

        cur_q = np.array(list(cur_joint_pos.values()), dtype=np.float64)
        goal_q = np.array(list(goal_joint_pos.values()), dtype=np.float64)

        # Construct the full joint state vector.
        err_hand = geom.get_pose_error_vector(cur_hand_pose, goal_hand_pose)
        err_head = geom.get_pose_error_vector(cur_head_pose, goal_head_pose)[3:]

        # Set the constraints to prevent joint limit violation.
        self._velIkSolver.set_jnt_saturation_constraints(
            cur_q, self._min_q, self._max_q, self._max_qd, self._max_qdd, self._time_step
        )

        # Compute Jacobian.
        J_hand = sim_util.get_link_jacobian(self._sim_muj,
                                            link_name=self._link_name,
                                            joint_names=self._joint_names)
        J_head = sim_util.get_link_jacobian(self._sim_muj,
                                            link_name=self._head_name,
                                            joint_names=self._joint_names)[3:]
        J_target = np.concatenate((J_hand, J_head), axis=0)

        # Scale the pose error with the gains.
        dxGoal = np.concatenate([self._pos_tracking_gain * err_hand[:3],
                                 self._rot_tracking_gain * err_hand[3:],
                                 self._head_tracking_gain * err_head])

        # Compute nullspace bias to stay close to the initial configuration
        # (for the selected joints) in the secondary task objective.
        qdNsArm = self._gains_nullspace_bias * (goal_q - cur_q)
        qdNs = qdNsArm.reshape(-1, 1)

        # Construct the tasks.
        dxGoalData = [dxGoal, qdNs]
        JData = [J_target, self._nsBiasJntProj]

        # Solve the velocity IK problem.
        d_q, s_data, _ = self._velIkSolver.solve(dxGoalData, JData)

        delta_q = d_q * self._time_step

        q_des = cur_q + delta_q

        ctrl_dict =  {key: value for key, value in zip(cur_joint_pos.keys(), q_des)}
        return {ctrl_key: ctrl_dict for ctrl_key in ['right_arm_joint', 'left_arm_joint', 'waist_joint', 'head_joint']}



class WholeBodyIKSolver(ArmIKSolver):

    def __init__(self, config, sim, link_name, body_name, joint_names, use_julia=True, **kwargs) -> None:

        super().__init__(config, sim, link_name, joint_names, **kwargs)

        self._body_name = body_name
        self._base_dof_vel = config.BASE_DOFS_VEL

        self._vel_base_max = config.MAX_BASE_VEL * np.ones(config.BASE_DOFS_VEL, dtype=np.float64)
        self._acc_base_max = config.MAX_BASE_ACC * np.ones(config.BASE_DOFS_VEL, dtype=np.float64)

        self._joint_indices_nullspace_bias = [idx for idx in range(config.BASE_DOFS_VEL)]
        self._gains_nullspace_bias_base = [config.BASE_NULLSPACE_GAIN] * config.BASE_DOFS_VEL
        self._gains_nullspace_bias_arm = [config.ARM_NULLSPACE_GAIN] * config.ARM_DOFS_JOINT

        self._nsBiasJntProj = np.zeros((config.ARM_DOFS_JOINT + config.BASE_DOFS_VEL, config.ARM_DOFS_JOINT + config.BASE_DOFS_VEL), dtype=np.float64)
        self._selected_nullspace_bias_jnts = self._joint_indices_nullspace_bias
        self._nsBiasJntProj[self._selected_nullspace_bias_jnts, self._selected_nullspace_bias_jnts] = 1.0

        self._upp_torso_bounds = np.array(config.TORSO_POSE_HALF_RANGE, dtype=np.float64)
        self._low_torso_bounds = -np.array(config.TORSO_POSE_HALF_RANGE, dtype=np.float64)

        # init julia solver
        self._use_julia = use_julia and USE_JULIA_SOLVER

        # init julia solver
        if self._use_julia:
            self._julia_inv_params = jl.FlexIK.InverseParams(tolerance=1e-1, damping_coeff=1e-1)


    def solve(
        self,
        goal_joint_pos,
        goal_body_pose,
        goal_hand_pose,
        cur_joint_pos,
        cur_body_pose,
        cur_hand_pose,
        des_body_pose,
        **kwargs
    ):

        err_body = geom.get_pose_error_vector(cur_body_pose, goal_body_pose)
        err_hand = geom.get_pose_error_vector(cur_hand_pose, goal_hand_pose)

        # Construct the full joint state vector.
        goal_q = np.array(list(goal_joint_pos.values()), dtype=np.float64)
        cur_q = np.array(list(cur_joint_pos.values()), dtype=np.float64)

        # Compute Jacobian.
        J_hand = sim_util.get_link_jacobian(self._sim_muj,
                                            link_name=self._link_name,
                                            whole_body_name=self._body_name,
                                            joint_names=self._joint_names
                                            )

        # Scale the pose error with the gains.
        dxGoal = np.zeros_like(err_hand)
        dxGoal[:3] = self._pos_tracking_gain * err_hand[:3]
        dxGoal[3:] = self._rot_tracking_gain * err_hand[3:]

        # Compute nullspace bias to stay close to the initial configuration
        # (for the selected joints) in the secondary task objective.
        qdNsBase = self._gains_nullspace_bias_base * err_body
        qdNsArm = self._gains_nullspace_bias_arm * (goal_q - cur_q)
        qdNs = np.concatenate((qdNsBase, qdNsArm)).reshape(-1, 1)

        # Construct the tasks.
        dxGoalData = [dxGoal, qdNs]
        JData = [J_hand, self._nsBiasJntProj]

        # Set the constraints to prevent joint limit violation.
        # Solve the velocity IK problem.
        if self._use_julia:
            output = jl.FlexIK.solve_wholebody_vel_ik(
                err_body,
                self._low_torso_bounds,
                self._upp_torso_bounds,
                self._vel_base_max,
                self._acc_base_max,
                cur_q,
                self._min_q,
                self._max_q,
                self._max_qd,
                self._max_qdd,
                self._time_step,
                dxGoalData,
                JData,
                self._julia_inv_params,
            )
            d_q_full = np.array(output[0]).ravel()

        else:
            self._velIkSolver.set_wholebody_saturation_constraints(
                error=err_body,
                bounds_low=self._low_torso_bounds,
                bounds_upp=self._upp_torso_bounds,
                vel_max=self._vel_base_max,
                acc_max=self._acc_base_max,
                q=cur_q,
                q_min=self._min_q,
                q_max=self._max_q,
                qd_max=self._max_qd,
                qdd_max=self._max_qdd,
                time_step=self._time_step,
            )
            d_q_full, _, _ = self._velIkSolver.solve(dxGoalData, JData)

        delta_q_full = d_q_full * self._time_step
        new_des_joint_pos = {key: value for key, value in zip(cur_joint_pos.keys(), cur_q + delta_q_full[self._base_dof_vel:])}
        new_des_body_pose = geom.integrate_pose_with_delta_pose(des_body_pose, delta_q_full[:self._base_dof_vel])

        return {
            'arm_joint': new_des_joint_pos,
            'mobile_joint': {
                'joint_body': {
                    'pos': new_des_body_pose['pos'],
                    'quat': new_des_body_pose['quat']
                }
                },
            }


class QuadrupedalArmIKSolver(WholeBodyIKSolver):

    def __init__(self, **kwargs) -> None:

        super().__init__(base_dof_vel=6, **kwargs)

    def solve(self, **kwargs):
            
        action = super().solve(**kwargs)
        action['quad_joint'] = {
            'body_pos': action['mobile_joint']['joint_body']['pos'],
            'body_quat': action['mobile_joint']['joint_body']['quat']
        }
        return action
