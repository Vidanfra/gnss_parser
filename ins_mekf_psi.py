import numpy as np
from scipy.linalg import expm

def smtrx(v):
    """
    Returns the 3×3 skew‐symmetric matrix of a 3‐vector v.
    """
    return np.array([
        [   0.0,   -v[2],  v[1]],
        [  v[2],     0.0, -v[0]],
        [ -v[1],   v[0],   0.0]
    ])


def gravity(mu):
    """
    WGS-84 normal gravity model at latitude mu (radians).
    Uses: g0=9.7803253359, k=0.00193185265241, e²=0.00669437999013.
    Returns g in m/s².
    """
    g0 = 9.7803253359
    k = 0.00193185265241
    e2 = 0.00669437999013
    s = np.sin(mu)
    return g0 * (1 + k * s**2) / np.sqrt(1 - e2 * s**2)


def Rquat(q):
    """
    Converts a unit quaternion q=[q0,q1,q2,q3] (scalar‐first) into a 3×3
    rotation matrix (body→NED).
    """
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*(q2**2 + q3**2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
        [    2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2),     2*(q2*q3 - q0*q1)],
        [    2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])


def Tquat(w):
    """
    Builds the 4×4 quaternion‐rate matrix for angular rate w=[wx,wy,wz].
    Used so that: q̇ = 0.5 * Tquat(w) * q.
    """
    wx, wy, wz = w
    return np.array([
        [ 0.0, -wx, -wy, -wz],
        [  wx,  0.0,  wz, -wy],
        [  wy, -wz,  0.0,  wx],
        [  wz,  wy, -wx,  0.0]
    ]) * 0.5


def quatprod(q, r):
    """
    Hamilton product of two quaternions q and r (each in [q0,q1,q2,q3], scalar‐first).
    Returns q ⊗ r.
    """
    q0, q1, q2, q3 = q
    r0, r1, r2, r3 = r
    s = q0*r0 - q1*r1 - q2*r2 - q3*r3
    x = q0*r1 + q1*r0 + q2*r3 - q3*r2
    y = q0*r2 - q1*r3 + q2*r0 + q3*r1
    z = q0*r3 + q1*r2 - q2*r1 + q3*r0
    return np.array([s, x, y, z])


def ssa(angle):
    """
    Wraps an angle (radians) into [-π, +π].
    """
    return (angle + np.pi) % (2*np.pi) - np.pi


def invQR(M):
    """
    Placeholder for a QR‐based inversion. Here we simply call NumPy's inv().
    """
    return np.linalg.inv(M)


def ins_mekf_psi(
    x_ins, P_prd, mu, h, Qd, Rd,
    f_imu, w_imu, y_psi=None, y_pos=None, y_vel=None
):
    """
    Multiplicative ESKF for INS aided by compass and slow position/velocity.

    Parameters
    ----------
    x_ins : (16,) ndarray
        INS state: [p(3), v(3), b_acc(3), q(4), b_ars(3)].
    P_prd : (15,15) ndarray
        Prediction covariance (error‐state).
    mu    : float
        Latitude (rad) for gravity calculation.
    h     : float
        Sampling time (s).
    Qd, Rd: ndarray
        Process / measurement noise covariances for the error‐state filter.
    f_imu : (3,) ndarray
        Specific‐force measurement (accelerometer).
    w_imu : (3,) ndarray
        Angular‐rate measurement (gyroscope).
    y_psi : float or None
        Compass (yaw) measurement, or None if unavailable.
    y_pos : (3,) ndarray or None
        Slow GNSS position measurement, or None if no position update this step.
    y_vel : (3,) ndarray or None
        Slow velocity measurement (optional), or None if unavailable.

    Returns
    -------
    x_ins_new : (16,) ndarray
        Updated INS state.
    P_prd_new : (15,15) ndarray
        Updated prediction covariance.
    """

    #── 1) Extract and initialize ─────────────────────────────────────────────
    # Time constants for bias models
    T_acc = 500.0
    T_ars = 500.0

    O3 = np.zeros((3, 3))
    I3 = np.eye(3)
    v01 = np.array([0.0, 0.0, -1.0])  # Gravity reference in body‐frame

    # Decompose the incoming INS state vector
    p_ins     = x_ins[ 0: 3].copy()   # position (NED)
    v_ins     = x_ins[ 3: 6].copy()   # velocity (NED)
    b_acc_ins = x_ins[ 6: 9].copy()   # accelerometer bias
    q_ins     = x_ins[ 9:13].copy()   # quaternion [q0,q1,q2,q3]
    b_ars_ins = x_ins[13:16].copy()   # gyro bias

    # Compute WGS‐84 gravity in NED at latitude mu
    g_n = np.array([0.0, 0.0, gravity(mu)])

    # Rotation matrix body→NED from quaternion
    R = Rquat(q_ins)

    # Bias‐compensated IMU measurements
    f_ins = f_imu - b_acc_ins
    w_ins = w_imu - b_ars_ins

    #── 2) Build continuous‐time error‐state A matrix (15×15) ─────────────────
    A = np.block([
        [  O3,          I3,            O3,                   O3,            O3 ],
        [  O3,          O3,           -R,      -R @ smtrx(f_ins),            O3 ],
        [  O3,          O3,  -(1.0/T_acc)*I3,                   O3,            O3 ],
        [  O3,          O3,            O3,           -smtrx(w_ins),         -I3 ],
        [  O3,          O3,            O3,                   O3, -(1.0/T_ars)*I3 ]
    ])

    # Discretize via matrix exponential: Ad = expm(A*h)
    Ad = expm(A * h)

    #── 3) Build measurement matrix Cd depending on which aiding signals exist ─
    # If y_pos is None, no GNSS aiding this step, so Cd is empty
    if y_pos is None:
        Cd = np.zeros((0, 15))
    else:
        # If y_vel is None: position + gravity only
        if y_vel is None:
            # [ pos(3) ;     gravity(3) ]
            Cd = np.block([
                [ I3,  O3,    O3,     O3,    O3 ],
                [ O3,  O3,    O3, smtrx(R.T @ v01), O3 ]
            ])
        else:
            # [ pos(3) ; vel(3) ; gravity(3) ]
            Cd = np.block([
                [ I3,  O3,    O3,     O3,    O3 ],
                [ O3,  I3,    O3,     O3,    O3 ],
                [ O3,  O3,    O3, smtrx(R.T @ v01), O3 ]
            ])

        # If we have a compass yaw measurement y_psi, augment Cd with a row for that:
        if y_psi is not None:
            # Compute partials of yaw w.r.t. Gibbs vector a = (2/q0)*[q1,q2,q3]
            a = (2.0 / q_ins[0]) * q_ins[1:4]          # 3×1 Gibbs vector
            u_y = 2.0 * (a[0]*a[1] + 2.0*a[2])
            u_x = 4.0 + a[0]**2 - a[1]**2 - a[2]**2
            du  = 1.0 / (1.0 + (u_y/u_x)**2)
            denom = (4.0 + a[0]**2 - a[1]**2 - a[2]**2)**2
            c_psi = du / denom * np.array([
                -2.0*((a[0]**2 + a[2]**2 - 4.0)*a[1] + a[1]**3 + 4.0*a[0]*a[2]),
                 2.0*((a[1]**2 - a[2]**2 + 4.0)*a[0] + a[0]**3 + 4.0*a[1]*a[2]),
                 4.0*(a[2]**2 + a[0]*a[1]*a[2] + a[0]**2 - a[1]**2 + 4.0)
            ])  # partial of yaw wrt Gibbs(a)

            # We append one row: [ 0(9), c_psiᵀ, 0(3) ]
            row = np.concatenate([np.zeros(9), c_psi, np.zeros(3)])
            Cd  = np.vstack([Cd, row])


    #── 4) Build process‐noise “spread” matrix Ed (15×12) ──────────────────────
    Ed = h * np.block([
        [  O3,    O3,    O3,   O3 ],
        [ -R,     O3,    O3,   O3 ],
        [  O3,    I3,    O3,   O3 ],
        [  O3,    O3,   -I3,   O3 ],
        [  O3,    O3,    O3,   I3 ]
    ])

    #── 5) Kalman‐gain update (Corrector) ────────────────────────────────────
    if y_pos is None:
        # No GNSS aiding this step → skip update
        P_hat = P_prd.copy()
    else:
        # Build measurement residual vector eps:
        eps_pos = y_pos - p_ins                # 3×1 position error
        v1      = f_ins / np.linalg.norm(f_ins) # normalized body‐force
        eps_g   = v1 - R.T @ v01               # gravity error in NED

        if y_psi is not None:
            # Compute predicted yaw from quaternion → use atan2(u_y, u_x)
            yaw_body = np.arctan2(2*(a[0]*a[1] + 2*a[2]),
                                  4 + a[0]**2 - a[1]**2 - a[2]**2)
            eps_psi  = ssa(y_psi - yaw_body)     # wrap into [-π,π]
        else:
            eps_psi = np.array([])  # no compass measurement

        if y_vel is None:
            # Residual: [ eps_pos(3); eps_g(3); eps_psi(1) ]
            eps = np.concatenate([eps_pos, eps_g, np.atleast_1d(eps_psi)])
        else:
            eps_vel = y_vel - v_ins  # 3×1 velocity error
            # Residual: [ eps_pos(3); eps_vel(3); eps_g(3); eps_psi(1) ]
            eps = np.concatenate([eps_pos, eps_vel, eps_g, np.atleast_1d(eps_psi)])

        # Compute Kalman gain: K = P_prd * Cdᵀ * inv( Cd * P_prd * Cdᵀ + Rd )
        S = Cd @ P_prd @ Cd.T + Rd
        K = P_prd @ Cd.T @ invQR(S)
        IKC = np.eye(15) - K @ Cd

        # Covariance update
        P_hat = IKC @ P_prd @ IKC.T + K @ Rd @ K.T

        # Error‐state correction: delta_x_hat (15×1)
        delta_x_hat = K @ eps

        # Extract delta_a = delta_x_hat[9:12] (Gibbs vector approximation)
        delta_a = delta_x_hat[9:12]  # 3×1

        # Form the “small” quaternion correction: delta_q = [2, delta_a] / sqrt(4 + ||delta_a||²)
        norm_factor = np.sqrt(4.0 + delta_a @ delta_a)
        delta_q     = np.concatenate([[2.0], delta_a]) / norm_factor  # 4×1

        # Reset INS states
        p_ins     = p_ins     + delta_x_hat[0:3]      # position
        v_ins     = v_ins     + delta_x_hat[3:6]      # velocity
        b_acc_ins = b_acc_ins + delta_x_hat[6:9]      # accel bias
        b_ars_ins = b_ars_ins + delta_x_hat[12:15]    # gyro bias

        # Attitude update: quaternion
        q_ins = quatprod(q_ins, delta_q)
        q_ins /= np.linalg.norm(q_ins)  # Normalize

    #── 6) Covariance prediction: P_prd[new] = Ad * P_hat * Adᵀ + Ed * Qd * Edᵀ ─
    P_prd_new = Ad @ P_hat @ Ad.T + Ed @ Qd @ Ed.T

    #── 7) INS state propagation (predict p_ins, v_ins, q_ins) ────────────────
    a_ins = R @ f_ins + g_n
    p_ins = p_ins + h * v_ins + 0.5 * (h**2) * a_ins
    v_ins = v_ins + h * a_ins

    # Quaternion propagation via exact exponential map: qₙₑw = expm( Tquat(w_ins) * h ) * q_old
    Omega = Tquat(w_ins) * h
    q_ins = expm(Omega) @ q_ins
    q_ins /= np.linalg.norm(q_ins)

    # Re‐assemble INS state vector (16×1)
    x_ins_new = np.concatenate([p_ins, v_ins, b_acc_ins, q_ins, b_ars_ins])

    return x_ins_new, P_prd_new
