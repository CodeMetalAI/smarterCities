import numpy as np


def kalman_filter(X, P, F, Q, H, Y, R):
    """
    Kalman filter prediction and update step
    
    Parameters:
    X: State vector
    P: State covariance matrix
    F: State transition function/matrix
    Q: Process noise function/covariance
    H: Measurement function/matrix
    Y: Measurement vector
    R: Measurement noise covariance
    
    Returns:
    X: Updated state vector
    P: Updated state covariance matrix
    """
    # Prediction step
    if callable(F):
        X, P = transformSigma(X, P, F)
    else:
        X = np.matmul(F, X)
        P = np.matmul(np.matmul(F, P), F.T)
    
    if callable(Q):
        X, P = transformSigma(X, P, Q)
    else:
        P = P + Q
    
    # Update step
    if callable(H):
        yh, Pyh, C = transformSigma(X, P, H)
    else:
        yh = np.matmul(H, X)
        Pyh = np.matmul(np.matmul(H, P), H.T)
        C = np.matmul(P, H.T)
    
    res = Y - yh
    S = R + Pyh
    
    # Handle different dimensionality for matrix division
    if np.isscalar(S) or (isinstance(S, np.ndarray) and S.size == 1):
        K = C / S
    else:
        K = np.matmul(C, np.linalg.inv(S))
    
    X = X + np.matmul(K, res).reshape(-1)
    # P = P - np.matmul(np.matmul(C, K.T), np.eye(P.shape[0]))
    P = P - np.matmul(C, K.T)
    
    return X, P



def ProcessNoiseCV(phi, ts, n):
    """
    Compute process noise matrix for constant velocity model
    
    Parameters:
    phi: Process noise scalar or vector
    ts: Time step in seconds
    n: Number of position dimensions
    
    Returns:
    Q: Process noise matrix
    """
    # Convert scalar to vector if needed
    if np.isscalar(phi):
        phi = np.ones(n) * phi
        
    # Precompute time steps
    ts2 = ts**2
    ts3 = ts2*ts/2
    ts4 = ts2*ts2/3
    
    # Initialize process noise matrix (including space for rssi_ref)
    Q = np.zeros((2*n, 2*n))
    
    # Square the noise parameters
    q2 = phi**2
    
    # Fill the process noise matrix for each dimension
    for i in range(n):
        # Position-position
        Q[i, i] = ts4 * q2[i]
        
        # Position-velocity and velocity-position
        Q[i, n+i] = ts3 * q2[i]
        Q[n+i, i] = ts3 * q2[i]
        
        # Velocity-velocity
        Q[n+i, n+i] = ts2 * q2[i]
    
    return Q



def transformSigma(X, P, h_func):
    """
    Unscented transform for measurement update
    Parameters:
    X: State vector
    P: State covariance
    h_func: Measurement function
    Returns:
    z_pred: Predicted measurement
    S: Innovation covariance
    K: Kalman gain
    """
    n = X.shape[0]
    # UT parameters
    alpha = 1e-3
    kappa = 0
    beta = 2
    # Generate sigma points
    lambda_param = alpha**2 * (n + kappa) - n
    scale = np.sqrt(n + lambda_param)
    # Weights
    wm0 = lambda_param / (n + lambda_param)
    wc0 = wm0 + (1 - alpha**2 + beta)
    wi = 1 / (2 * (n + lambda_param))
    wm = np.array([wm0] + [wi] * (2*n))
    wc = np.array([wc0] + [wi] * (2*n))
    # Try to get Cholesky decomposition
    ct = -8
    passed = False
    while not passed and ct < 0:
        try:
            L = np.linalg.cholesky(P)
            passed = True
        except np.linalg.LinAlgError:
            P += np.eye(P.shape[0]) * 10**ct # caution this overwrites
            ct += 1
            print(f"Adjusting covariance, attempt {ct}")
            passed = False
    if not passed:
        print("Failed to make covariance positive definite, track should be dropped")
        return None, None, None # Return None to signal track should be dropped
    # Generate sigma points
    sigma_points = np.zeros((n, 2*n + 1))
    sigma_points[:, 0] = X
    for i in range(n):
        sigma_points[:, i+1] = X + scale * L[:, i]
        sigma_points[:, i+n+1] = X - scale * L[:, i]
    # Transform sigma points through measurement function
    h0 = np.atleast_1d(h_func(sigma_points[:, 0]))
    m = h0.shape[0]
    # m = h0.shape[0] if hasattr(h0, 'shape') else 1  # Output dimension
    Z = np.zeros((m, 2*n + 1))
    Z[:, 0] = h0
    for i in range(1, 2*n + 1):
        Z[:, i] = h_func(sigma_points[:, i])
    # Calculate predicted measurement and covariance
    z_pred = np.zeros((m, 1))
    for i in range(2*n + 1):
        z_pred += wm[i] * Z[:, i].reshape(-1, 1)
    # Innovation covariance
    S = np.zeros((m, m))
    for i in range(2*n + 1):
        y_diff = (Z[:, i] - z_pred.flatten()).reshape(-1, 1)
        S += wc[i] * (y_diff @ y_diff.T)
    # Cross-covariance
    Pxz = np.zeros((n, m))
    for i in range(2*n + 1):
        x_diff = (sigma_points[:, i] - X).reshape(-1, 1)
        y_diff = (Z[:, i] - z_pred.flatten()).reshape(-1, 1)
        Pxz += wc[i] * (x_diff @ y_diff.T)
    return z_pred, S, Pxz


def transformCV(X, P, ts, phi):
    """
    Constant velocity propagation of state and covariance
    
    Parameters:
    X: State vector [x, y, vx, vy, rssi_ref]
    P: State covariance matrix
    ts: Time step in seconds
    phi: Process noise matrix or scalar
    n: Number of position states (default: auto-detect)
    """
    if n is None:
        n = X.shape[0] // 2  
    
    a = slice(0, n)  # Position indices
    b = slice(n, 2*n)  # Velocity indices
    X[a] += X[b]*ts

    P[a, a] += (P[b, a] + P[a, b]) * ts + P[b, b] * (ts**2)
    tmp = P[b, b] * ts
    P[a, b] += tmp
    P[b, a] += tmp
    
    # Add process noise
    Q = ProcessNoiseCV(phi, ts, n)
    P += Q
    return 



# ### if main transformCV test: 

# np.round(ProcessNoiseCV(1, .1, n=2),5)


# X = np.array([1.,1.,1.,1.])
# P = np.eye(4)
# ts = .1

# transformCV(X,P,ts)

# F = np.eye(4)
# F[0,2] = ts
# F[1,3] = ts
# # F@P@F.T
# H = np.zeros((2,4))
# H[0,0] = 1
# H[1,1] = 1
# def h_func(X):
#     return H@X

