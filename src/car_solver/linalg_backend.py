"""Sparse linear algebra backend boundary for SIMP solvers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu


class LinearSolveFactorization(Protocol):
    """Factorized sparse system that can solve for multiple right-hand sides."""

    def solve(self, rhs: np.ndarray) -> np.ndarray:
        """Solve the factorized system for a single right-hand side."""


class LinearAlgebraBackend(Protocol):
    """Backend interface for sparse matrix assembly and factorization."""

    def assemble(self, sK: np.ndarray, iK: np.ndarray, jK: np.ndarray, shape: tuple[int, int]):
        """Assemble a sparse global stiffness matrix."""

    def factorize_free_matrix(self, K, free_dofs: np.ndarray) -> LinearSolveFactorization:
        """Extract and factorize the free-free block of the global system."""


@dataclass(frozen=True)
class SciPySparseBackend:
    """Reference CPU backend using SciPy sparse assembly and LU factorization."""

    def assemble(self, sK: np.ndarray, iK: np.ndarray, jK: np.ndarray, shape: tuple[int, int]):
        return coo_matrix((sK, (iK, jK)), shape=shape).tocsc()

    def factorize_free_matrix(self, K, free_dofs: np.ndarray) -> LinearSolveFactorization:
        K_free = K[free_dofs, :][:, free_dofs]
        return splu(K_free.tocsc())
