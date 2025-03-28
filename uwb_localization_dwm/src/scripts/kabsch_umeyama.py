#!/usr/bin/env python3

import numpy as np


"""
The implementation of umeyama alignment in evo and the calling:
(1)Calling in `evo/core/trajectory.py`

        if n == -1:
            r_a, t_a, s = geometry.umeyama_alignment(self.positions_xyz.T,
                                                     traj_ref.positions_xyz.T,
                                                     with_scale)
        else:
            r_a, t_a, s = geometry.umeyama_alignment(
                self.positions_xyz[:n, :].T, traj_ref.positions_xyz[:n, :].T,
                with_scale)

(2)Implementation in `evo/core/geometry.py` is shown as following:
Same(but reverse) with the second implementation.
"""
import typing

UmeyamaResult = typing.Tuple[np.ndarray, np.ndarray, float]

def umeyama_alignment(x: np.ndarray, y: np.ndarray,
                      with_scale: bool = False) -> UmeyamaResult:
    """
    Computes the least squares solution parameters of an Sim(m) matrix
    that minimizes the distance between a set of registered points.
    Umeyama, Shinji: Least-squares estimation of transformation parameters
                     between two point patterns. IEEE PAMI, 1991
    :param x: mxn matrix of points, m = dimension, n = nr. of data points
    :param y: mxn matrix of points, m = dimension, n = nr. of data points
    :param with_scale: set to True to align also the scale (default: 1.0 scale)
    :return: r, t, c - rotation matrix, translation vector and scale factor

    Transformation: error = min||x_b - (c R x_a +t)||
    """
    if x.shape != y.shape:
        print("Data matrices must have the same shape")

    # m = dimension, n = nr. of data points
    m, n = x.shape

    mean_x = x.mean(axis=1)
    mean_y = y.mean(axis=1)

    # variance
    sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis])**2)

    # covariance matrix
    outer_sum = np.zeros((m, m))
    for i in range(n):
        outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
    cov_xy = np.multiply(1.0 / n, outer_sum)

    # SVD
    u, d, v = np.linalg.svd(cov_xy)
    if np.count_nonzero(d > np.finfo(d.dtype).eps) < m - 1:
        print("Degenerate covariance rank, Umeyama alignment is not possible")

    # S matrix
    s = np.eye(m)
    if np.linalg.det(u) * np.linalg.det(v) < 0.0:
        # Ensure a RHS coordinate system (Kabsch algorithm).
        s[m - 1, m - 1] = -1

    # rotation, eq. 40
    r = u.dot(s).dot(v)

    # scale & translation, eq. 42 and 41
    c = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
    t = mean_y - np.multiply(c, r.dot(mean_x))

    return r, t, c

# method2
def kabsch_umeyama(A, B, with_scale: bool = False):
    """
    References
    - [Umeyama's paper](http://edge.cs.drexel.edu/Dmitriy/Matching_and_Metrics/Umeyama/um.pdf)
    -[Raw introducation adn implementation](https://zpl.fi/aligning-point-patterns-with-kabsch-umeyama-algorithm/)
    - [CarloNicolini's python implementation](https://gist.github.com/CarloNicolini/7118015)
    Transformation
        error = min||x_b - (c R x_a +t)||
        """

    assert A.shape == B.shape
    n, m = A.shape

    EA = np.mean(A, axis=0)
    EB = np.mean(B, axis=0)
    VarA = np.mean(np.linalg.norm(A - EA, axis=1) ** 2)

    H = ((A - EA).T @ (B - EB)) / n
    U, D, VT = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U) * np.linalg.det(VT))
    S = np.diag([1] * (m - 1) + [d])

    R = U @ S @ VT
    c = VarA / np.trace(np.diag(D) @ S) if with_scale else 1.0
    t = EA - c * R @ EB

    return R, t, c

def main():
    """
    Simulation data.
    """

    import matplotlib.pyplot as plt

    A = np.array([[ 23, 178],
                [ 66, 173],
                [ 88, 187],
                [119, 202],
                [122, 229],
                [170, 232],
                [179, 199]])
    B = np.array([[232, 38],
                [208, 32],
                [181, 31],
                [155, 45],
                [142, 33],
                [121, 59],
                [139, 69]])

    R, t, c = kabsch_umeyama(A, B)
    # print("R,c,t = :\n", end=' ')
    # # print(R, c, t)
    # print(R)
    # print(t)
    # print(c)

    plt.plot(A[:,0], A[:,1], 'y-', label='y=f(x)', linewidth=2)
    # plt.show()
    plt.plot(B[:,0], B[:,1])
    plt.show()

    B_new = np.array([t + c * R @ b for b in B])

    # Plot
    plt.plot(A[:,0], A[:,1], 'y-', label='y=f(x)', linewidth=2)
    # plt.show()
    plt.plot(B_new[:,0], B_new[:,1])
    plt.show()

    R, t, c = umeyama_alignment(B.T, A.T) # same with 2
    B_new = np.array([t + c * R @ b for b in B])
    A_new = np.array([t + c * R @ b for b in A])

    # Plot
    plt.plot(A[:,0], A[:,1], 'y-', label='y=f(x)', linewidth=2)

    plt.plot(B_new[:,0], B_new[:,1])

    plt.show()


if __name__ == '__main__':
    main()
