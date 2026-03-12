import numpy as np


class PolyTraj():
    def __init__(self, p=3, dim=4):
        self.p = p
        self.dim = dim
        # coeffs shape: (dim, p+1) - each row contains polynomial coefficients for one dimension
        self.coeffs = np.zeros((dim, p + 1))

    def _get_time_basis(self, t) -> np.ndarray:
        """Get time basis vector [1, t, t^2, ..., t^p]"""
        return t ** np.arange(self.p + 1)

    def get_pos(self, t) -> np.ndarray:
        time_basis = self._get_time_basis(t)
        return np.dot(self.coeffs, time_basis)

    def get_pos_array(self, cmd_hz, duration) -> np.ndarray:
        t_array = np.arange(0, duration, 1.0 / cmd_hz)
        # Fully vectorized computation using broadcasting
        time_basis_matrix = t_array[:, None] ** np.arange(self.p + 1)
        # Matrix multiplication: (n_times, p+1) @ (p+1, dim) -> (n_times, dim)
        return np.dot(time_basis_matrix, self.coeffs.T)

    def update_coeffs(self, coeffs: np.ndarray):
        """Update polynomial coefficients
        Args:
            coeffs: shape (dim, p+1) or (p+1,) for single dimension
        """
        if coeffs.ndim == 1:
            if len(coeffs) != self.p + 1:
                raise ValueError(f"Expected {self.p + 1} coefficients, got {len(coeffs)}")
            if self.dim != 1:
                raise ValueError(f"Single dimension coeffs provided but trajectory has {self.dim} dimensions")
            self.coeffs = coeffs.reshape(1, -1)
        elif coeffs.ndim == 2:
            if coeffs.shape != (self.dim, self.p + 1):
                raise ValueError(f"Expected shape ({self.dim}, {self.p + 1}), got {coeffs.shape}")
            self.coeffs = coeffs
        else:
            raise ValueError(f"Coefficients must be 1D or 2D array, got {coeffs.ndim}D")
