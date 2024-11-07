# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
from enum import Enum
from typing import Optional, Tuple, List

import numpy as np
from numpy.linalg import pinv

from .inverse_solver import InverseSolver, PseudoInverseSolver

logger = logging.getLogger("flexVelIk")


class VelIkSolver:
    MARGIN_LIMIT = 1e-4
    THRES_TOO_LARGE = 1e10
    THRES_TOO_SMALL = 1e-10

    class Types(Enum):
        ESNS_MT = "extended_sns_multi_task_velocity_ik_solver"

    class ExitCodes(Enum):
        SUCCESS = 0
        MAX_ITERATION_REACHED = 1
        INVALID_INPUT = 2
        INVERSE_SOLVER_UNSTABLE = 3
        LIMIT_VIOLATION = 4

    def __init__(
        self,
        inverse_solver: Optional[InverseSolver] = None,
        type: Types = Types.ESNS_MT,
    ) -> None:
        """
        Initializes the VelocityIKSolver object.

        Args:
            inverse_solver: An optional inverse solver object.
            If not provided, a PseudoInverseSolver object will be used.
            type : The type of the solver. Defaults to Types.ESNS_MT.
        """
        self._inverse_solver = inverse_solver if inverse_solver is not None else PseudoInverseSolver()
        self._type = type
        self._C: Optional[np.ndarray] = None
        self._lim_low: Optional[np.ndarray] = None
        self._lim_upp: Optional[np.ndarray] = None

    @property
    def type(self) -> Types:
        return self._type

    @property
    def get_inv_solver(self) -> InverseSolver:
        return self._inverse_solver

    def set_inverse_solver(self, value: InverseSolver) -> None:
        self._inverse_solver = value

    def set_constraints(self, C: np.ndarray, lim_low: np.ndarray, lim_upp: np.ndarray) -> None:
        """
        Set constraints for the solver.

        Args:
            C: The constraint matrix (lim_low <= C * dq < lim_upp).
            lim_low: The lower limits for the constraints.
            lim_upp: The upper limits for the constraints.

        Raises:
            ValueError: If there is a dimension mismatch between C, lim_low, and lim_upp.

        """
        if not (C.shape[0] == len(lim_low) == len(lim_upp)):
            error_msg = (
                f"Dimension mismatch between C matrix ({C.shape[0]}), lim_low ({len(lim_low)}) and lim_upp"
                f" ({len(lim_upp)})."
            )
            logger.error(error_msg)
            raise ValueError(error_msg)
        self._C = C
        self._lim_low = lim_low
        self._lim_upp = lim_upp

    def append_constraints(
        self,
        C: np.ndarray,
        lim_low: np.ndarray,
        lim_upp: np.ndarray,
        insertion_position: int = -1,
    ) -> None:
        """
        Append constraints to the top or bottom of the existing ones/sequence.

        Parameters:
        - C: the constraint matrix to be appended.
        - lim_low: the lower limit array to be appended.
        - lim_upp: the upper limit array to be appended.
        - insertion_position: int, indicates whether to append the constraints
            at the bottom (-1) or top (0) of the existing ones.

        Raises:
        - ValueError: If the dimensions of C, lim_low, and lim_upp do not match.

        Returns:
        - None
        """
        if not (C.shape[0] == len(lim_low) == len(lim_upp)):
            error_msg = (
                f"Dimension mismatch between C matrix ({C.shape[0]}), lim_low ({len(lim_low)}) and lim_upp"
                f" ({len(lim_upp)})."
            )
            logger.error(error_msg)
            raise ValueError(error_msg)

        self._C = np.insert(self._C, insertion_position, C, axis=0)
        self._lim_low = np.insert(self._lim_low, insertion_position, lim_low)
        self._lim_upp = np.insert(self._lim_upp, insertion_position, lim_upp)

    def set_joint_constraints(self, lim_low: np.ndarray, lim_upp: np.ndarray) -> None:
        """
        Set joint constraints for the solver.

        Args:
            lim_low: The lower limits of the joint constraints.
            lim_upp: The upper limits of the joint constraints.

        Raises:
            ValueError: If the dimensions of lim_low and lim_upp do not match.

        Notes:
            If the constraint is only for joint space, then the constraint matrix is set to identity automatically.
        """
        if not len(lim_low) == len(lim_upp):
            error_msg = f"Dimension mismatch between lim_low ({len(lim_low)}) and lim_upp ({len(lim_upp)})."
            logger.error(error_msg)
            raise ValueError(error_msg)
        self._C = np.eye(lim_low.shape[0])
        self._lim_low = lim_low
        self._lim_upp = lim_upp

    def calculate_box_constraints(
        margin_low: np.ndarray,
        margin_upp: np.ndarray,
        vel_max: np.ndarray,
        acc_max: np.ndarray,
        time_step: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate box constraints based on the given inputs.
        The box constraints are calculated based on the given margin
        (signed distance to the configuration constraint), maximum velocity,
        maximum acceleration, and time step. This constraint ensures that the velocity
        satisfying the constraints does not violate the limits.

        Args:
            margin_low: The lower margin values (negative sign).
            margin_upp: The upper margin values (positive sign).
            vel_max: The maximum velocity values.
            acc_max: The maximum acceleration values.
            time_step: The time step value in seconds.

        Returns:
            A tuple containing the lower and upper box constraints.

        Notes:
            If the position constraint is violated, the velocity in the corresponding direction will be set to zero.
        """
        margin_low = np.minimum(margin_low, 0)  # ensure it is non-positive
        margin_upp = np.maximum(margin_upp, 0)  # ensure it is non-negative

        lim_low = np.maximum(
            np.maximum(margin_low / time_step, -vel_max),
            -np.sqrt(2 * acc_max * np.fabs(margin_low)),
        )
        lim_upp = np.minimum(
            np.minimum(margin_upp / time_step, vel_max),
            np.sqrt(2 * acc_max * np.fabs(margin_upp)),
        )
        return lim_low, lim_upp

    def construct_jnt_saturation_constraints(
        self,
        q: np.ndarray,
        q_min: np.ndarray,
        q_max: np.ndarray,
        qd_max: np.ndarray,
        qdd_max: np.ndarray,
        time_step: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Set the joint saturation constraints.

        This method constructs the joint saturation constraints for the velocity inverse kinematics solver.
        The constraints ensure that the joint positions (q) and velocities (qd) remain within the specified limits.
        The constraints are defined as follows:
        - q_min <= q + qd * time_step <= q_max
        - -qd_max <= qd <= qd_max
        - -qdd_max <= qdd <= qdd_max (when approaching q_min and q_max)

        Args:
            q: The current joint positions.
            q_min: The minimum joint positions.
            q_max: The maximum joint positions.
            qd_max: The maximum joint velocities.
            qdd_max: The maximum joint accelerations.
            time_step: The time step for the constraints in seconds.

        Returns:
            A tuple containing the constraint matrix (C),
            the lower limit array (lim_low), and the upper limit array (lim_upp).
        """
        if not all(len(var) == len(q) for var in [q_min, q_max, qd_max, qdd_max]):
            error_msg = (
                f"Dimension mismatch: (q ({len(q)}), q_min ({len(q_min)}), "
                + f"q_max ({len(q_max)}), qd_max ({len(qd_max)}, qdd_max ({len(qdd_max)}))."
            )
            logger.error(error_msg)
            raise ValueError(error_msg)

        # create copies of the input arrays to prevent modifying the original arrays
        q_min = q_min.copy()
        q_max = q_max.copy()
        qd_max = qd_max.copy()
        qdd_max = qdd_max.copy()

        # apply small margin to the limits to consider numerical issues
        q_min += VelIkSolver.MARGIN_LIMIT
        q_max -= VelIkSolver.MARGIN_LIMIT
        qd_max -= VelIkSolver.MARGIN_LIMIT
        qdd_max -= VelIkSolver.MARGIN_LIMIT

        C = np.eye(q.shape[0])
        margin_low = q_min - q  # negative margin
        margin_upp = q_max - q  # positive margin

        lim_low, lim_upp = VelIkSolver.calculate_box_constraints(margin_low, margin_upp, qd_max, qdd_max, time_step)

        return C, lim_low, lim_upp

    def set_jnt_saturation_constraints(
        self,
        q: np.ndarray,
        q_min: np.ndarray,
        q_max: np.ndarray,
        qd_max: np.ndarray,
        qdd_max: np.ndarray,
        time_step: float,
    ) -> None:
        """
        Sets the joint saturation constraints for the velocity IK solver.

        Parameters:
        - q: The current joint positions.
        - q_min: The lower joint limits.
        - q_max: The upper joint limits.
        - qd_max: The maximum joint velocities.
        - qdd_max: The maximum joint accelerations.
        - time_step: The time step for the solver in seconds.

        Returns:
        - None
        """
        C, lim_low, lim_upp = self.construct_jnt_saturation_constraints(q, q_min, q_max, qd_max, qdd_max, time_step)
        self.set_constraints(C, lim_low, lim_upp)

    def construct_base_saturation_constraints(
        self,
        error: np.ndarray,
        bounds_low: np.ndarray,
        bounds_upp: np.ndarray,
        vel_max: np.ndarray,
        acc_max: np.ndarray,
        time_step: float,
    ) :
        """
        Construct the saturation constraints for the floating base axes.

        Args:
            error: The error with the reference pose used to compute the constraint.
            bounds_low: The minimum bounds for each axis.
            bounds_upp: The maximum bounds for each axis.
            vel_max: The maximum axis velocities.
            acc_max: The maximum axis accelerations.
            time_step: The time step for the constraint calculation in seconds.

        Returns:
            A tuple containing the following arrays:
            - C: The identity matrix with the same shape as the error array.
            - lim_low: The lower box constraints for each axis.
            - lim_upp: The upper box constraints for each axis.
        """

        assert (
            len(error) == len(bounds_low) == len(bounds_upp) == len(vel_max) == len(acc_max)
        ), "Dimension mismatch: (error, bounds_low, bounds_upp, vel_max, acc_max)."

        # create copies of the input arrays to prevent modifying the original arrays
        bounds_low = bounds_low.copy()
        bounds_upp = bounds_upp.copy()
        vel_max = vel_max.copy()
        acc_max = acc_max.copy()

        # apply small margin to the limits to consider numerical issues
        bounds_low += VelIkSolver.MARGIN_LIMIT
        bounds_upp -= VelIkSolver.MARGIN_LIMIT
        vel_max -= VelIkSolver.MARGIN_LIMIT
        acc_max -= VelIkSolver.MARGIN_LIMIT

        C = np.eye(error.shape[0])
        margin_low = bounds_low + error  # negative margin
        margin_upp = bounds_upp + error  # positive margin

        lim_low, lim_upp = VelIkSolver.calculate_box_constraints(margin_low, margin_upp, vel_max, acc_max, time_step)

        return C, lim_low, lim_upp

    def set_wholebody_saturation_constraints(
        self,
        error: np.ndarray,
        bounds_low: np.ndarray,
        bounds_upp: np.ndarray,
        vel_max: np.ndarray,
        acc_max: np.ndarray,
        q: np.ndarray,
        q_min: np.ndarray,
        q_max: np.ndarray,
        qd_max: np.ndarray,
        qdd_max: np.ndarray,
        time_step: float,
    ) -> None:
        """
        Sets the whole-body joint saturation constraints for the velocity IK solver.
        The whole-body constraints include the floating base and joint saturation constraints.

        Args:
            error: The error vector.
            bounds_low: The lower bounds of the error vector.
            bounds_upp: The upper bounds of the error vector.
            vel_max: The maximum velocity vector.
            acc_max: The maximum acceleration vector.
            q: The current joint positions.
            q_min: The lower bounds of the joint positions.
            q_max: The upper bounds of the joint positions.
            qd_max: The maximum joint velocities.
            qdd_max: The maximum joint accelerations.
            time_step: The time step in seconds.

        Returns:
            None
        """
        C_base, lim_low_base, lim_upp_base = self.construct_base_saturation_constraints(
            error, bounds_low, bounds_upp, vel_max, acc_max, time_step
        )

        C_jnts, lim_low_jnts, lim_upp_jnts = self.construct_jnt_saturation_constraints(
            q, q_min, q_max, qd_max, qdd_max, time_step
        )

        # Append the constraints diagonally to the existing constraints
        C = np.eye(error.shape[0] + lim_low_jnts.shape[0])
        C[: C_base.shape[0], : C_base.shape[1]] = C_base
        C[C_base.shape[0] :, C_base.shape[1] :] = C_jnts

        lim_low = np.concatenate((lim_low_base, lim_low_jnts))
        lim_upp = np.concatenate((lim_upp_base, lim_upp_jnts))

        self.set_constraints(C, lim_low, lim_upp)

    def solve(
        self,
        task_goal_data: List[np.ndarray],
        task_jacobian_data: List[np.ndarray],
    ) :
        """
        Calculates joint velocities given a set of Jacobians and task goals satisfying the constraints.

        Args:
            task_goal_data: A list of vectors representing the task goals in the order of priorities.
            task_jacobian_data: A list of matrices representing the task Jacobians in the order of priorities.

        Returns:
            A tuple containing the joint velocities, scale factors in the range [0, 1],
            and an exit code indicating the success or failure of the solver.
        """

        # input validation
        # check the number of elements in the list of Jacobians and task goals
        if not len(task_jacobian_data) == len(task_goal_data):
            error_msg = (
                f"The number of Jacobians ({len(task_jacobian_data)}) and "
                "task goals ({len(task_goal_data)}) must be the same."
            )
            logger.error(error_msg)
            raise ValueError(error_msg)

        # check the dimensions of all the inputs to be equal
        for taskJacobian_entry, taskGoal_entry in zip(task_jacobian_data, task_goal_data):
            if not taskJacobian_entry.shape[0] == taskGoal_entry.shape[0]:
                error_msg = (
                    f"Dimension mismatch between Jacobian ({taskJacobian_entry.shape}) and task goal"
                    f" ({taskGoal_entry.shape})."
                )
                logger.error(error_msg)
                raise ValueError(error_msg)

        if not self._type == VelIkSolver.Types.ESNS_MT:
            logger.error(  # type: ignore
                f"The solver type ({self._type}) is not supported. Using ESNS_MT solver instead."
            )

        return self.extended_sns_vel_ik(
            self._C,
            self._lim_low,
            self._lim_upp,
            task_goal_data,
            task_jacobian_data,
            self._inverse_solver,
        )

    @staticmethod
    def find_scale_factor(low: float, upp: float, a: float) -> Tuple[float, float]:
        """
        Computes feasible task scale factors for SNS IK algorithms.

        This function computes task scale factors from upper and lower margins
        and the desired task for a single component. This function is called by
        all of SNS IK algorithms.

        INPUTS:
            low: lower margin
            upp: upper margin
            a: desired task

        OUTPUTS:
            Tuple[float, float]: (sMax, sMin)
                sMax: the maximum feasible scale factor [0, 1]
                sMin: the minimum feasible scale factor [0, 1]

        NOTES:
        """

        # input validation
        if low > upp:
            error_msg = f"The lower margin ({low}) must be <= the upper margin ({upp})."
            logger.error(error_msg)
            raise ValueError(error_msg)

        sUpp = 1.0  # the feasible scale factor considering upper margin
        sLow = 0.0  # the feasible scale factor considering lower margin
        if VelIkSolver.THRES_TOO_SMALL < abs(a) < VelIkSolver.THRES_TOO_LARGE:
            if a < 0.0 and low < 0.0 and a <= upp:
                sUpp = low / a
                sLow = upp / a
            elif a > 0.0 and upp > 0.0 and a >= low:
                sUpp = upp / a
                sLow = low / a
        return (min(1.0, sUpp), max(0.0, sLow))

    @staticmethod
    def is_identity_matrix(matrix: np.ndarray) -> bool:
        """
        Checks if the given matrix is an identity matrix.

        Parameters:
        matrix: The matrix to be checked.

        Returns: True if the matrix is an identity matrix, False otherwise.
        """

        if not matrix.shape[0] == matrix.shape[1]:
            return False

        return np.allclose(matrix, np.eye(matrix.shape[0]))

    @staticmethod
    def is_zero_matrix(matrix: np.ndarray) -> bool:
        """
        Checks if the given matrix is a zero matrix.

        Parameters:
        matrix: The matrix to be checked.

        Returns: True if the matrix is a zero matrix, False otherwise.
        """
        return np.all(matrix == 0)

    @staticmethod
    def check_limit_exceeded(val: float, lb: float, ub: float, tol: float = 1e-6) -> bool:
        return np.any(val < (lb - tol)) or np.any(val > (ub + tol))

    @staticmethod
    def extended_sns_vel_ik(
        C: np.ndarray,
        lim_low: np.ndarray,
        lim_upp: np.ndarray,
        task_goal_data: List[np.ndarray],
        task_jacobian_data: List[np.ndarray],
        solver: Optional[InverseSolver] = None,
    ) -> Tuple[np.ndarray, np.ndarray, ExitCodes]:
        """
        Calculates joint velocities given a set of Jacobians and task goals
        using the inverse kinematics algorithm with velocity-based multi-task
        prioritization and constraints. This function uses a provided inverse
        solver for computing a pseudo inverse of Jacobian.

        Args:
        C: constraint matrix.
        lim_low: lower limits.
        lim_upp: upper limits.
        task_goal_data: list of vectors, each containing the desired
            velocity for each task in the order of priorities.
        task_jacobian_data: list of matrices, each containing the Jacobian matrix
            for each task in the order of priorities.
        inverse_solver: InverseSolver for matrix inversion
            (if not provided, the default PINV solver will be used)

        Returns:
        tuple: A tuple containing the following elements:
            dq: The joint velocities that achieve the task goals.
            sData: An array containing the task scaling factors in the range [0, 1],
                scaling each task's velocity to satisfy the constraints.
            exitCode: Success or failure with a brief reason
        """

        if solver is None:
            solver = PseudoInverseSolver()

        # get the number of tasks
        nTask = len(task_jacobian_data)
        sData = np.zeros(nTask)

        # initialization
        exitCode = VelIkSolver.ExitCodes.SUCCESS
        tol = 1e-6  # a tolerance used for various numerical checks
        tolTight = 1e-10  # a tighter tolerance
        nIterationsMax = 20

        nJnt = task_jacobian_data[0].shape[1]
        k = C.shape[0] - nJnt
        JntEye = np.eye(nJnt)
        Pi = JntEye
        Sact = np.zeros((nJnt + k, nJnt + k))
        dqi = np.zeros((nJnt, 1))
        w = np.zeros((nJnt + k, 1))
        Cact = np.zeros((nJnt + k, C.shape[1]))

        C = (
            1 if VelIkSolver.is_identity_matrix(C) else C
        )  # This is to avoid matrix multiplication with identity matrix.

        for iTask in range(nTask):
            # get i-th task jacobian
            Ji = task_jacobian_data[iTask]
            ndxGoal = Ji.shape[0]

            # get i-th task velocity
            dxGoali = task_goal_data[iTask].reshape(-1, 1)

            # update variables for previous projection matrix and solution
            PiPrev = Pi
            dqiPrev = dqi

            # initialize variables for i-th task
            PiBar = PiPrev
            si = 1.0
            siStar = 0.0

            limitExceeded = True
            cntLoop = 1

            SactStar = Sact
            wStar = w

            PiBarStar = np.zeros_like(PiBar)
            PiHatStar = np.zeros((nJnt, nJnt + k))

            if np.sum(np.abs(dxGoali)) < tolTight:
                dqi = dqiPrev
                limitExceeded = False

            # Pre-computing for optimization
            inv_JiPiBar = solver.inv(Ji @ PiBar)
            Ji_dqiPrev = Ji @ dqiPrev

            if VelIkSolver.is_zero_matrix(Sact):
                Cact_PiPrev = np.zeros((nJnt + k, nJnt))
                Cact_dqiPrev = np.zeros((nJnt + k, 1))
            else:
                Cact_PiPrev = Cact @ PiPrev
                Cact_dqiPrev = Cact @ dqiPrev

            while limitExceeded:
                limitExceeded = False

                # update PiHat
                PiHat = (JntEye - inv_JiPiBar @ Ji) @ np.linalg.pinv(Cact_PiPrev, tol)
                # compute a solution without task scale factor
                dqi = dqiPrev + inv_JiPiBar @ (dxGoali - Ji_dqiPrev) + PiHat @ (w - Cact_dqiPrev)

                # Do the following only if the solution violates the constraints
                limitExceeded = VelIkSolver.check_limit_exceeded(np.ravel(np.dot(C, dqi)), lim_low, lim_upp)

                a = np.dot(C, inv_JiPiBar) @ dxGoali
                b = np.dot(C, dqi) - a.reshape(-1, 1)

                if a_norm := np.linalg.norm(a) < tol:
                    # if the projected goal velocity is close to zero, set scale factor to one (feasible)
                    si = 1
                    mclIdx = 0
                elif a_norm > VelIkSolver.THRES_TOO_LARGE:
                    # if the projected goal velocity is too large, set scale factor to zero (infeasible)
                    logger.debug(
                        f"the projected goal for task {iTask + 1} is too large (inverse solver output might be"
                        " invalid)."
                    )
                    si = 0
                    mclIdx = 0
                    exitCode = VelIkSolver.ExitCodes.INVERSE_SOLVER_UNSTABLE
                else:
                    # compute the scale factor and identify the critical joint
                    marginL = lim_low - np.ravel(b)
                    marginU = lim_upp - np.ravel(b)

                    sMax = np.zeros(nJnt + k)
                    sMin = np.zeros(nJnt + k)
                    for iLim in range(nJnt + k):
                        if Sact[iLim, iLim] == 1:
                            sMax[iLim] = np.inf
                        else:
                            sMax[iLim], sMin[iLim] = VelIkSolver.find_scale_factor(
                                marginL[iLim], marginU[iLim], a[iLim]
                            )

                    # most critical limit index
                    mclIdx = np.argmin(sMax)
                    si = sMax[mclIdx]

                # scale factor should be zero if the projected goal velocity is too large
                si = 0 if np.isinf(si) else si

                # do the following only if the task is feasible and the scale
                # factor calculated is correct
                if (iTask == 0 or si > 0) and cntLoop < nIterationsMax:
                    scaledDqi = dqiPrev + inv_JiPiBar @ (si * dxGoali - Ji_dqiPrev) + PiHat @ (w - Cact_dqiPrev)

                    z = np.ravel(np.dot(C, scaledDqi))
                    if si > siStar and not VelIkSolver.check_limit_exceeded(z, lim_low, lim_upp):
                        siStar = si
                        SactStar = Sact
                        wStar = w
                        PiBarStar = PiBar
                        PiHatStar = PiHat

                    Sact[mclIdx, mclIdx] = 1
                    Cact = np.dot(Sact, C)
                    w[mclIdx, 0] = min(max(lim_low[mclIdx], z[mclIdx]), lim_upp[mclIdx])

                    Cact_PiPrev = Cact @ PiPrev

                    PiBar = PiPrev - pinv(Cact_PiPrev, tol) @ Cact_PiPrev

                    taskRank = np.linalg.matrix_rank(Ji @ PiBar, tol)
                    if taskRank < ndxGoal:
                        si = siStar
                        Sact = SactStar
                        Cact = np.dot(Sact, C)
                        w = wStar
                        PiBar = PiBarStar
                        PiHat = PiHatStar

                        dqi = (
                            dqiPrev
                            + solver.inv(Ji @ PiBar) @ (si * dxGoali - Ji_dqiPrev)
                            + PiHat @ (w - Cact @ dqiPrev)
                        )

                        limitExceeded = False

                else:  # if the current task is infeasible
                    si = 0
                    dqi = dqiPrev
                    limitExceeded = False

                    if cntLoop == nIterationsMax:
                        logger.warn("the maximum number of iteration has been reached!")
                        exitCode = VelIkSolver.ExitCodes.MAX_ITERATION_REACHED
                        return dqi, sData, exitCode

                cntLoop += 1

                if si > 0:
                    # update nullspace projection
                    Pi = PiPrev - pinv(Ji @ PiPrev, tol) @ Ji @ PiPrev

                # Update the pre-computed terms
                inv_JiPiBar = solver.inv(Ji @ PiBar)
                Cact_PiPrev = Cact @ PiPrev
                Cact_dqiPrev = Cact @ dqiPrev
                Ji_dqiPrev = Ji @ dqiPrev

            sData[iTask] = si

        dqi = np.ravel(dqi)

        # check if the solution violated the constraints while dqi is non-zero (i.e., any of scale factors > 0)
        z = np.ravel(np.dot(C, dqi))
        if np.sum(sData) > tol and VelIkSolver.check_limit_exceeded(z, lim_low, lim_upp):
            logger.error("The solution violated the constraints!")
            logger.debug(f"dqi: {dqi}")
            logger.debug(f"lim_low: {lim_low}")
            logger.debug(f"lim_upp: {lim_upp}")
            exitCode = VelIkSolver.ExitCodes.LIMIT_VIOLATION

        return dqi, sData, exitCode
