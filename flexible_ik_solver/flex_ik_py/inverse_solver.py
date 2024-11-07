# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from abc import ABC, abstractmethod

import numpy as np

EPS = 1e-8


class InverseSolver(ABC):
    DEFAULT_TOLERANCE = 1e-6
    DEFAULT_DAMPING_COEFF = 1e-1

    """
    Class for computing the inverse of a matrix using various methods.
    """

    @abstractmethod
    def inv(self, A: np.ndarray) -> np.ndarray:
        """
        Compute inverse of matrix A using the specified method and parameters.

        Args:
            A: Input matrix to be inverted.

        Returns:
            A_inv: Inverse of matrix A.
        """

    def __repr__(self) -> str:
        """
        Get the current method of inversion.

        Returns:
            method (str): Name of the current subclass.
        """
        return self.__class__.__name__


class PseudoInverseSolver(InverseSolver):
    """
    Compute pseudo-inverse of matrix A using the pinv method.

    Parameters:
        A: Input matrix to be inverted.

    Returns:
        A_inv: Pseudo-inverse of matrix A.
    """

    def inv(self, A: np.ndarray) -> np.ndarray:
        return np.linalg.pinv(A, rcond=1e-8)


class DampedLeastSquaresSolver(InverseSolver):
    """
    Compute damped least-square inverse of matrix A.

    Parameters:
        A: Input matrix to be inverted.
        tolerance: Tolerance value for damping methods.

    Returns:
        A_inv: Damped least-square inverse of matrix A.
    """

    def __init__(self, tolerance: float = InverseSolver.DEFAULT_TOLERANCE):
        self._tolerance = tolerance

    def inv(self, A: np.ndarray) -> np.ndarray:
        lambda_val = self._tolerance
        A_inv = A.T @ np.linalg.inv(A @ A.T + lambda_val**2 * np.eye(A.shape[0]))
        return A_inv


class SingularityRobustSolver(InverseSolver):
    def __init__(
        self,
        tolerance: float = InverseSolver.DEFAULT_TOLERANCE,
        damping_coeff: float = InverseSolver.DEFAULT_DAMPING_COEFF,
    ):
        self._tolerance = tolerance
        self._damping_coeff = damping_coeff

    def inv(self, A: np.ndarray) -> np.ndarray:
        """
        Compute singularity-robust inverse of matrix A.

        Parameters:
            A: Input matrix to be inverted.
            tolerance: Tolerance value for damping methods.
            damping_coeff: Maximum damping coefficient.

        Returns:
            A_inv: Singularity-robust inverse of matrix A.
        """
        # this is a scalar value which is a damping coefficient
        # for all the axes corresponding to the singular values below tolerance
        beta = 0

        # this is a scalar damping constant for the entire matrix
        # it is used to numerically stabilize the matrix when the determinant is below threshold
        lambda_val = self._tolerance**2

        U, S, V = np.linalg.svd(A)
        s_min = np.min(S)

        # scale the damping coefficient from 0 to beta_max based on the minimum singular value
        u_sum = np.zeros((A.shape[0], A.shape[0]))
        if s_min < self._tolerance:
            for ind in range(S.size - 1, -1, -1):
                if S[ind] < self._tolerance:
                    u_sum += np.outer(U[:, ind], U[:, ind])
                else:
                    break
            beta = (1 - (s_min / self._tolerance) ** 2) * self._damping_coeff

        # reconstruct the inverse
        tempMat = A @ A.T + lambda_val * np.eye(A.shape[0]) + beta * u_sum
        A_inv = A.T @ np.linalg.inv(tempMat)
        return A_inv


class ExtendedSingularityRobustSolver(InverseSolver):
    """
    Compute extended singularity-robust inverse of matrix A.

    Parameters:
        A: Input matrix to be inverted.
        tolerance: Tolerance value for damping methods.
        damping_coeff: Maximum damping coefficient.

    Returns:
        A_inv: Extended singularity-robust inverse of matrix A.
    """

    def __init__(
        self,
        tolerance: float = InverseSolver.DEFAULT_TOLERANCE,
        damping_coeff: float = InverseSolver.DEFAULT_DAMPING_COEFF,
    ):
        self._tolerance = tolerance
        self._damping_coeff = damping_coeff

    def inv(self, A: np.ndarray) -> np.ndarray:
        # SVD of the matrix
        U, S, _ = np.linalg.svd(A)
        AAT = A @ A.T

        # Compute determinant and lambda value.
        # Note that lamdba_val is a scalar damping constant for the entire matrix,
        # which is used to numerically stabilize the matrix when the determinant is below threshold
        threshold = self._tolerance**2
        determinant = np.linalg.det(AAT)
        lambda_val = (1.0 - (determinant / threshold) ** 2) * threshold if determinant < threshold else 0.0

        # Check if the singular values are below the tolerance,
        # and compute the damping coefficients accordingly.
        # This damping gets applied to only singular directions.
        tempMat = AAT + lambda_val * np.eye(A.shape[0])
        if np.any(below_tolerance := S < self._tolerance):
            beta = np.where(
                below_tolerance,
                self._damping_coeff * (1 - (S / self._tolerance) ** 2),
                0,
            )
            U_reduced = U[:, below_tolerance]
            u_sum_scaled = U_reduced @ np.diag(beta[below_tolerance]) @ U_reduced.T
            tempMat += u_sum_scaled

        # Reconstruct the inverse
        A_inv = A.T @ np.linalg.inv(tempMat)
        return A_inv
