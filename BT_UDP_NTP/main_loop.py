

# python main_loop.py
import socket
import json
import time
import threading
import numpy as np
# import ulab as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import pandas as pd
plt.ion()


from scipy.linalg import block_diag

from trackFunctions import kalman_filter,ProcessNoiseCV,transformSigma

# Scanner positions
SCANNER_POSITIONS = {
    "192.168.222.113": np.array([10,0]),
    "192.168.222.108": np.array([7.5,0]),
    "192.168.222.93": np.array([3.5,0]),
    "192.168.222.72": np.array([0, 3.5]),
    "192.168.222.247": np.array([0, 7.5]),
    "192.168.222.122": np.array([0, 10.5]),
}

RSSI_phi = .01 # process noise dB/s 
PROCESS_NOISE_STD = 5.  # m/s^2 Process noise or m/s depending on model
MEAS_NOISE_STD = 17. # Measurement noise standard deviation
rssi_ref_0 = -50
prop_factor = 12

detection_batch_size = 10
TRACK_DELETION_TIME = 30.0  # (s) before deleting inactive tracks
BUFFER_MAX_SIZE = 100  # Maximum buffer size
BUFFER_PROCESS_THRESHOLD = 0.5   

### constant velocity and estimate transmit power
# X0 = np.array([7.0, 7.0, 0.0, 0.0, -50.0])
# lb = [0,0,-10,-10,-70]
# ub = [10,10,10,10,-40]
# P0 = np.diag(np.array([10.0, 10.0, 3.0, 3.0, 10.0])**2)
# def F_func(ts):
#     F = np.eye(5)
#     F[0,2] = ts
#     F[1,3] = ts
#     return F
# def Q_func(ts):
#     return block_diag(ProcessNoiseCV(PROCESS_NOISE_STD, ts, 2),(ts*RSSI_phi)**2)

### constant position and estimate transmit power
# lb = np.array([0,0,rssi_ref_0-5])
# ub = np.array([10,10,rssi_ref_0+5])
# P0 = np.diag(np.array([10.0, 10.0, 10.0])**2)
# def F_func(ts):
#     F = np.eye(3)
#     return F
# def Q_func(ts):
#     return np.diag(np.array([PROCESS_NOISE_STD,PROCESS_NOISE_STD,RSSI_phi])*ts**2)

### constant 3-d position and estimate transmit power
lb = np.array([0,0,-2,rssi_ref_0-5])
ub = np.array([10,10,10,rssi_ref_0+5])
P0 = np.diag(np.array([10., 10.,10., 10.])**2)
def F_func(ts):
    F = np.eye(4)
    return F
def Q_func(ts):
    return np.diag(np.array([PROCESS_NOISE_STD,PROCESS_NOISE_STD,PROCESS_NOISE_STD/5,RSSI_phi])*ts**2)

X0 = np.array((lb+ub)/2)

def position_to_rssi(X, scanner_pos):
    """
    Measurement function: convert state to expected RSSI
    
    Parameters:
    X: State vector [x, y, vx, vy, rssi_ref]
    scanner_pos: Position of the scanner [x, y]
    
    Returns:
    Expected RSSI value
    """
    x= X[0]
    y =  X[1]
    z = X[2]
    if len(X) == 2:
        rssi_ref = rssi_ref_0
    else:
        rssi_ref = X[-1] 
    
    # Calculate distance to scanner
    distance_squared = z**2+(x - scanner_pos[0])**2 + (y - scanner_pos[1])**2 
    return rssi_ref - prop_factor * np.log10(distance_squared)

# Global state
tracks = {}  # Dictionary to store all tracks
buffer_list = []  # Sorted buffer list for detections
buffer_lock = threading.Lock()  # Thread safety for the buffer
running = False  # Flag to control threads

def update_track(mac, detection):
    """
    Update a track with a new detection
    
    Parameters:
    mac: MAC address of the device
    detection: Detection data dictionary
    """
    global tracks
    
    timestamp = detection['timestamp']
    rssi = detection['rssi']
    scanner_ip = detection['scanner_ip']
    name = detection['name']
    
    # Skip if scanner position unknown
    if scanner_ip not in SCANNER_POSITIONS:
        print('unknown scanner: '+scanner_ip)
        return
    
    scanner_pos = SCANNER_POSITIONS[scanner_ip]
    
    # Check if track exists
    if mac not in tracks:
        tracks[mac] = {
            'X': X0.copy(),
            'P': P0.copy(),
            'last_timestamp': timestamp,
            'last_update_time': time.time(),
            'name': name,
            'active': True
        }
    else:
        track = tracks[mac]
        
        # Check if this detection is older than the last one
        if timestamp < track['last_timestamp']:
            print('OOD')
            return  # Skip backward detections
        
        # Time step in seconds
        ts = (timestamp - track['last_timestamp']) / 1000.0

        # context dependent Kalman Filter values 
        F = F_func(ts)
        
        Q = Q_func(ts)
        def h_func(state_vector):
            return position_to_rssi(state_vector, scanner_pos)
        R = (MEAS_NOISE_STD*(rssi/60)**2)**2
        Y = rssi

        # Update track information 
        track['X'],track['P'] = kalman_filter(track['X'],track['P'], F, Q, h_func, Y, R)
        track['X'].clip(lb,ub, out=track['X'])

        tracks[mac]['last_timestamp'] = timestamp
        tracks[mac]['last_update_time'] = time.time()
        if name != 'Unknown':
            tracks[mac]['name'] = name  # Update name in case it changed
        tracks[mac]['active'] = True

def prune_inactive_tracks():
    """Remove inactive tracks"""
    global tracks
    current_time = time.time()
    
    for mac in list(tracks.keys()):
        if current_time - tracks[mac]['last_update_time'] > TRACK_DELETION_TIME:
            del tracks[mac]

def detection_receiver(port):
    """
    Thread function to receive detections via UDP
    
    Parameters:
    port: UDP port number to listen on
    """
    global running, buffer_list, buffer_lock
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.settimeout(0.5)  # Non-blocking socket
    
    print(f"Listening for detections on port {port}...")
    
    try:
        while running:
            try:
                data, addr = sock.recvfrom(2048)
                detection = json.loads(data)
                
                # Add to sorted buffer list, dropping oldest if full
                with buffer_lock:
                    # If buffer is full, remove the oldest detection
                    if len(buffer_list) >= BUFFER_MAX_SIZE:
                        buffer_list.pop(0)  # Remove oldest (first) item
                        print("Warning: Buffer full, dropping oldest detection")
                    
                    # Insert new detection in the right position to maintain sorted order
                    timestamp = detection['timestamp']
                    i = 0
                    while i < len(buffer_list) and buffer_list[i]['timestamp'] < timestamp:
                        i += 1
                    buffer_list.insert(i, detection)
                    # if you have a small buffer, out of order detections can happen becausee UDP is not 
                    
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                print("Warning: Invalid JSON received")
            except Exception as e:
                print(f"Error in receiver: {e}")
    finally:
        sock.close()

def detection_processor():
    """Thread function to process detections from buffer"""
    global running, buffer_list, buffer_lock, tracks
    
    while running:
        # try:
        # Check if buffer has enough detections to process
        process_detections = False
        with buffer_lock:
            if len(buffer_list) >= BUFFER_MAX_SIZE * BUFFER_PROCESS_THRESHOLD:
                process_detections = True
        
        if process_detections:
            # Get batch of detections to process
            batch = []
            with buffer_lock:
                # Get up to half the buffer
                batch_size = min(len(buffer_list) // 2, detection_batch_size)  # Don't take more than 10 at once
                batch = buffer_list[:batch_size]
                buffer_list = buffer_list[batch_size:]
            
            # Process each detection
            for detection in batch:
                mac = detection['mac']
                update_track(mac, detection)
            
            # Prune inactive tracks
            prune_inactive_tracks()
        else:
            # Sleep longer if not enough detections
            time.sleep(0.1)
                
        # except Exception as e:
        #     print(f"Error in processor: {e}")
        #     time.sleep(0.01)  # Small sleep to prevent CPU hogging in case of errors

def get_track_positions():
    """
    Get current positions of all active tracks
    
    Returns:
    Dictionary of track positions and metadata
    """
    results = {}
    
    for mac, track in tracks.items():
        if track['active']:
            results[mac] = {
                'x': track['X'][0],
                'y': track['X'][1],
                'rssi_ref': track['X'][-1], 
                'name': track['name'],
                'uncertainty': np.sqrt(track['P'][0, 0] + track['P'][1, 1])
            }
    
    return results

def get_buffer_status():
    """
    Get the current buffer status
    
    Returns:
    Dictionary with buffer information
    """
    with buffer_lock:
        return {
            'buffer_size': len(buffer_list),
            'buffer_max_size': BUFFER_MAX_SIZE,
            'buffer_threshold': BUFFER_PROCESS_THRESHOLD,
            'buffer_threshold_count': int(BUFFER_MAX_SIZE * BUFFER_PROCESS_THRESHOLD),
            'processing_active': len(buffer_list) >= BUFFER_MAX_SIZE * BUFFER_PROCESS_THRESHOLD
        }

def start_tracking(port=12345, buffer_max_size=BUFFER_MAX_SIZE, buffer_threshold=0.5):
    """
    Start the tracking system
    
    Parameters:
    port: UDP port to listen on
    buffer_max_size: Maximum buffer size
    buffer_threshold: Process threshold (0-1)
    """
    global running, BUFFER_MAX_SIZE, BUFFER_PROCESS_THRESHOLD
    
    if running:
        print("Tracking already running")
        return
    
    # Update buffer parameters
    BUFFER_MAX_SIZE = buffer_max_size
    BUFFER_PROCESS_THRESHOLD = buffer_threshold
    
    running = True
    
    # Start threads
    receiver_thread = threading.Thread(target=detection_receiver, args=(port,))
    processor_thread = threading.Thread(target=detection_processor)
    
    receiver_thread.daemon = True
    processor_thread.daemon = True
    
    receiver_thread.start()
    processor_thread.start()
    
    print(f"Tracking started with buffer size {buffer_max_size} and threshold {buffer_threshold}")
    return receiver_thread, processor_thread

def stop_tracking():
    """Stop the tracking system"""
    global running
    running = False
    print("Tracking stopped")

# Start tracking with custom buffer parameters
threads = start_tracking(port=12345, buffer_max_size=BUFFER_MAX_SIZE, buffer_threshold=0.5)

# Setup plotting
# Load floor plan image
img = mpimg.imread('floorPlan.png')
fig, ax = plt.subplots(figsize=(10, 6))
Extent = [-1.8, 31.8, -1.1, 10.8]
ax.imshow(img, extent=Extent)
ax.grid(True, linestyle='--', alpha=0.7, color='red')

# Add compass
center_x, center_y = -0.5, -0.5
arrow_size = 0.4
ax.arrow(center_x, center_y, 0, arrow_size, head_width=0.1, head_length=0.1, fc='black', ec='black', lw=2)
ax.arrow(center_x, center_y, arrow_size, 0, head_width=0.1, head_length=0.1, fc='black', ec='black', lw=2)
ax.text(center_x, center_y + arrow_size + 0.1, 'N', ha='center', fontsize=14, fontweight='bold')
ax.text(center_x + arrow_size + 0.1, center_y, 'E', va='center', fontsize=14, fontweight='bold')

# Dictionary to store plot objects for each device
device_plots = {}

# Main loop
try:
    while True:
        # Get current track positions
        positions = get_track_positions()
        buffer_status = get_buffer_status()
        
        print(f"Buffer: {buffer_status['buffer_size']}/{buffer_status['buffer_max_size']} " +
              f"(threshold: {buffer_status['buffer_threshold_count']})")
        print(f"Active tracks: {len(positions)}")
        unique_ips = set(item['scanner_ip'] for item in buffer_list)
        unique_ips_str = ', '.join(unique_ips)
        print('Active scanners (' +str(len(unique_ips))+'): '+ unique_ips_str)
        
        # Clear old plots that are no longer active
        for mac in list(device_plots.keys()):
            if mac not in positions:
                if device_plots[mac] in ax.get_children():
                    device_plots[mac].remove()
                del device_plots[mac]
        
        # Update plots for each device
        for mac, pos in positions.items():
            print(f"{pos['name']} ({mac[-8:]}): ({pos['x']:.1f}, {pos['y']:.1f}), RSSI ref: {pos['rssi_ref']:.1f}")
            
            # if "Razer Stereo" in pos['name']:
            # if True:
            # if pos['uncertainty'] < 2:
            if mac == "44:5e:cd:0f:db:41":
                x, y = pos['x'], pos['y']
                # vx, vy = pos['vx'], pos['vy']
                
                # Create or update plot
                if mac in device_plots:
                    # Update existing plot
                    device_plots[mac].set_offsets(np.array([[x, y]]))
                else:
                    # Create new plot
                    device_plots[mac] = ax.scatter(
                        x, y, s=100, marker='o', color='blue',
                        label=f"{pos['name']} ({mac[-8:]})"
                    )
                    
        # Draw updated figure
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        
        time.sleep(1)
except KeyboardInterrupt:
    stop_tracking()


##### plotting velocity when it's estimated
# # Create velocity arrow (optional)
# arrow_scale = 2.0  # Scale factor for velocity vector
# if hasattr(device_plots[mac], 'arrow') and device_plots[mac].arrow:
#     device_plots[mac].arrow.remove()

# if np.sqrt(vx**2 + vy**2) > 0.05:  # Only draw arrow if velocity is significant
# device_plots[mac].arrow = ax.arrow(
#         x, y, vx * arrow_scale, vy * arrow_scale,
#         head_width=0.2, head_length=0.3, fc='red', ec='red'
#     )
                    # device_plots[mac].arrow = None  # Place to store arrow reference

##################################################
##################################################
##################################################
## KF verified to match matlab code with this example: 
# # Tracking parameters


# true_state = np.array([1.0, 1.0, 0.5, 0.3, -50.0])

# locs = [[0,0],[0,10],[10,0]]

# ts = .1
# X = X0
# P = P0
# F = np.eye(5)
# F[0,2] = ts
# F[1,3] = ts
# Q = block_diag(ProcessNoiseCV(PROCESS_NOISE_STD, ts, 2),(ts*RSSI_phi)**2)
# R = MEAS_NOISE_STD**2
# ct = -1


# ct += 1
# true_state = F@true_state

# scanner_pos = locs[ct%3]
# def h(state_vector):
#     return position_to_rssi(state_vector, scanner_pos)
# Y = h(true_state)


# X = np.matmul(F, X)
# P = np.matmul(np.matmul(F, P), F.T)
# P = P + Q

# yh, Pyh, C = transformSigma(X, P, h)
# res = Y - yh
# S = R + Pyh

# if np.isscalar(S) or (isinstance(S, np.ndarray) and S.size == 1):
#     K = C / S
# else:
#     K = np.matmul(C, np.linalg.inv(S))

# X = X + np.matmul(K, res)
# P = P - np.matmul(np.matmul(C, K.T), np.eye(P.shape[0]))
##################################################
##################################################
##################################################

# #### graveyard 
# def transformSigma(X, P, h_func):
#     """
#     Unscented transform for measurement update
    
#     Parameters:
#     X: State vector
#     P: State covariance
#     h_func: Measurement function
    
#     Returns:
#     z_pred: Predicted measurement
#     S: Innovation covariance
#     K: Kalman gain
#     """
#     n = X.shape[0]
    
#     # UT parameters
#     alpha = 1e-3
#     kappa = 0
#     beta = 2
    
#     # Generate sigma points
#     lambda_param = alpha**2 * (n + kappa) - n
#     scale = np.sqrt(n + lambda_param)
    
#     # Weights
#     wm0 = lambda_param / (n + lambda_param)
#     wc0 = wm0 + (1 - alpha**2 + beta)
#     wi = 1 / (2 * (n + lambda_param))
    
#     wm = np.array([wm0] + [wi] * (2*n))
#     wc = np.array([wc0] + [wi] * (2*n))
    
#     # Try to get Cholesky decomposition
#     ct = -8
#     passed = False
#     while not passed and ct < 0:  
#         try:
#             L = np.linalg.cholesky(P)
#             passed = True
#         except np.linalg.LinAlgError:
#             P += np.eye(P.shape[0]) * 10**-ct # caution this overwrites
#             ct += 1
#             print(f"Adjusting covariance, attempt {ct}")
#             passed = False
    
#     if not passed:
#         print("Failed to make covariance positive definite, track should be dropped")
#         return None, None, None  # Return None to signal track should be dropped
    
#     # Generate sigma points
#     sigma_points = np.zeros((n, 2*n + 1))
#     sigma_points[:, 0] = X
    
#     for i in range(n):
#         sigma_points[:, i+1] = X + scale * L[:, i]
#         sigma_points[:, i+n+1] = X - scale * L[:, i]
    
#     # Transform sigma points through measurement function
#     h0 = h_func(sigma_points[:, 0])
#     Z = np.zeros((h0.shape[0],2*n + 1))
#     for i in range(1,2*n + 1):
#         Z[:,i] = h_func(sigma_points[:, i])
    
#     # Calculate predicted measurement and covariance
#     z_pred = Z @ wm
    
#     # Innovation covariance
#     Zoffset = (Z - z_pred[:, np.newaxis])
#     S = Zoffset@Zoffset.T
#     S = np.sum(wc * (Z - z_pred[:, np.newaxis])**2)
    
#     # Cross-covariance
#     Pxz = np.zeros(n)
#     for i in range(2*n + 1):
#         Pxz += wc[i] * (sigma_points[:, i] - X) * (Z[i] - z_pred)
    
#     return z_pred, S, Pxz

# def h_func(X): 
    # return(F@X)


### was rawdogging KF in the track update, pulling that out to be its own function 


# # Prediction step
# transformCV(track['X'], track['P'], 
# dt, 
# PROCESS_NOISE_STD,
# n = 2  # 2 position states (x, y)
# )
# # Update step using UT
# z_pred, Pyh, C = transformSigma(track['X'], track['P'], h_func)
# if z_pred is None:
# print(f"Dropping track {mac} due to numerical issues with covariance")
# del tracks[mac]
# return
# R = MEAS_NOISE_STD**2
# S = R + Pyh
# K = C / S

# # Innovation
# residual = rssi - z_pred

# # Update state and covariance
# tmp = K * residual






################################################
################################################
################################################
################################################
#### one step deeper 
# true_positions = []
# filtered_positions = []
# ct = 0
# ip_addresses = list(SCANNER_POSITIONS.keys())
# for t in np.arange(0, SIM_DURATION, DELTA_T):
#     ct += 1
#     scannerID = ct%len(SCANNER_POSITIONS)
#     true_positions.append(true_state[:2].copy())
#     true_state[0] += true_state[2] * DELTA_T
#     true_state[1] += true_state[3] * DELTA_T
#     scanner_ip = ip_addresses[scannerID]
#     scanner_pos = SCANNER_POSITIONS[scanner_ip]
#     true_rssi = position_to_rssi(true_state, scanner_pos)
#     noisy_rssi = true_rssi + np.random.normal(0, MEAS_NOISE_STD)
    
#     # Create detection object in the format expected by update_track
#     detection = {
#         'timestamp': int(t * 1000),  # milliseconds
#         'rssi': noisy_rssi,
#         'scanner_ip': scanner_ip,
#         'name': 'Simulated Device',
#         'mac': 'AA:BB:CC:DD:EE:FF'  # Use a fixed MAC for the simulation
#     }
    
#     # Update the track with this detection
#     update_track('AA:BB:CC:DD:EE:FF', detection)
    
#     # Get the current estimate after incorporating all scanner measurements
#     # if 'AA:BB:CC:DD:EE:FF' in tracks:
#         # Get position estimate
#     est_position = tracks['AA:BB:CC:DD:EE:FF']['X'][:2].copy()
#     filtered_positions.append(est_position)

# # Convert to numpy arrays for plotting
# true_positions = np.array(true_positions)
# filtered_positions = np.array(filtered_positions)

# # Calculate position error
# pos_errors = np.sqrt(np.sum((true_positions - filtered_positions)**2, axis=1))
# rmse = np.sqrt(np.mean(pos_errors**2))

# # Plot results
# plt.figure(figsize=(10, 6))
# plt.plot(true_positions[:, 0], true_positions[:, 1], 'b-', label='True Path')
# plt.plot(filtered_positions[:, 0], filtered_positions[:, 1], 'r--', label='Filtered Path')

# # Add scanner positions
# for scanner_id, pos in SCANNER_POSITIONS.items():
#     plt.plot(pos[0], pos[1], 'ko', markersize=8)
#     plt.annotate(scanner_id[-3:], (pos[0], pos[1]), xytext=(0,10), 
#                 textcoords="offset points", ha='center')

# plt.xlabel('X Position (m)')
# plt.ylabel('Y Position (m)')
# plt.grid(True)
# plt.legend()
# plt.title(f'Position RMSE: {rmse:.3f} m')
# plt.show()
# plt.show(block=True)
################################################
################################################
################################################
################################################

# # Simulation parameters
# SIM_DURATION = 20  # seconds
# DELTA_T = 0.1      # time step in seconds
# MEAS_NOISE_STD = 1.0  # Measurement noise dB

# tracks.clear()
# true_state = np.array([1.0, 1.0, 0.5, 0.3, -50.0])
# X = X0
# P = P0


# df = pd.read_csv('detections_0311.csv')
# # df.name == 'R'
# device_id = 'Razer Stereo'
# df_1 = df.loc[df['name'] == device_id]
# df_1 = df_1.sort_values('timestamp')
# delta = np.diff(df_1.timestamp)
# np.sort(df_1.timestamp)
# plt.plot(range(len(delta)),delta)
# plt.show()

