%Step 1 - AGV Parameters
dt = 0.1;              % Time step (s)
T = 20;               % Simulation time (s)
N = T/dt;             % Steps
wheel_circ = 31.4;    % cm
ppr = 10;             % Pulses per rev
wheelbase = 20;       % cm
k_pwm = 0.1;          % PWM to speed (cm/s per PWM)
mass = 5;             % kg (approx)


% Step 2 - Quick test
disp(['Simulation steps: ', num2str(N)]);
disp(['Wheel circumference: ', num2str(wheel_circ), ' cm']);

% Initial state
x = 0; y = 0; theta = 0; v_left = 0; v_right = 0;

% Storage
x_hist = zeros(1,N); y_hist = zeros(1,N); theta_hist = zeros(1,N);

% Open-loop simulation
pwm_left = 100; pwm_right = 100;  % Fixed PWM
tau = 0.1;  % Motor time constant

for i = 1:N
    % Motor dynamics
    v_left = v_left + (k_pwm * pwm_left - v_left) * dt / tau;
    v_right = v_right + (k_pwm * pwm_right - v_right) * dt / tau;
    
    % AGV kinematics
    v = (v_left + v_right) / 2;
    omega = (v_right - v_left) / wheelbase;
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
    
    % Store
    x_hist(i) = x; y_hist(i) = y; theta_hist(i) = theta;
end

% Plot
t = 0:dt:T-dt;
figure; plot(x_hist, y_hist); 
title('Open-Loop Path'); xlabel('x (cm)'); ylabel('y (cm)');
grid on;

% Step 3 - Add to top
function enc_dist = get_encoder_distance(v_left, v_right, dt, wheel_circ, ppr)
    true_dist = (v_left + v_right) / 2 * dt;
    enc_dist = true_dist + randn * 0.5;  % Noise ±0.5 cm
end

function [accel, yaw_rate] = get_imu_data(v, v_prev, dt)
    accel = (v - v_prev) / dt + randn * 50;  % Noise ±50 cm/s²
    yaw_rate = randn * 0.1;                  % Drift ±0.1 rad/s
end

% Step 4 - Update loop
v_hist = zeros(1,N);
for i = 1:N
    % Motor dynamics
    v_left = v_left + (k_pwm * pwm_left - v_left) * dt / tau;
    v_right = v_right + (k_pwm * pwm_right - v_right) * dt / tau;
    
    % AGV kinematics
    v = (v_left + v_right) / 2;
    omega = (v_right - v_left) / wheelbase;
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
    
    % Sensor data
    enc_dist = get_encoder_distance(v_left, v_right, dt, wheel_circ, ppr);
    [accel, yaw_rate] = get_imu_data(v, v_hist(max(i-1,1)), dt);
    
    % Kalman filter
    [x_est, v_est, P] = kalman_filter(x_est, v_est, P, enc_dist, accel, dt);
    
    % Store
    x_hist(i) = x; y_hist(i) = y; theta_hist(i) = theta;
    v_hist(i) = v; x_est_hist(i) = x_est; v_est_hist(i) = v_est;
end

% Plotting
t = 0:dt:T-dt;
figure;
subplot(2,1,1); plot(t, x_hist, 'b', t, x_est_hist, 'r--');
title('Position'); xlabel('Time (s)'); ylabel('x (cm)');
legend('True', 'Estimated');
subplot(2,1,2); plot(t, v_hist, 'b', t, v_est_hist, 'r--');
title('Speed'); xlabel('Time (s)'); ylabel('v (cm/s)');
legend('True', 'Estimated');
ylim([0 20]);  % Limit y-axis for clarity

% Function definitions (local functions at the end)
function enc_dist = get_encoder_distance(v_left, v_right, dt, wheel_circ, ppr)
    true_dist = (v_left + v_right) / 2 * dt;
    enc_dist = true_dist + randn * 0.5;  % Noise ±0.5 cm
end

function [accel, yaw_rate] = get_imu_data(v, v_prev, dt)
    accel = (v - v_prev) / dt + randn * 50;  % Noise ±50 cm/s²
    yaw_rate = randn * 0.1;                  % Drift ±0.1 rad/s
end

function [x_est, v_est, P] = kalman_filter(x_est, v_est, P, enc_dist, accel, dt)
    Q = [0.05 0; 0 0.02]; R = [0.5 0; 0 10];
    A = [1 dt; 0 1]; B = [0; dt]; H = [1 0; 0 1];
    
    % Prediction
    x_pred = A(1,1) * x_est + A(1,2) * v_est;
    v_pred = A(2,2) * v_est + B(2) * accel;
    P_pred = A * P * A' + Q;
    
    % Update
    imu_vel = v_est + accel * dt;  % Simple integration for IMU velocity
    z = [enc_dist; imu_vel];       % Measurement: encoder distance, IMU velocity
    y = z - H * [x_pred; v_pred];
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;           % Kalman gain
    x_est = x_pred + K(1) * y(1);
    v_est = v_pred + K(2) * y(2);  % Corrected: use y(2) for velocity
    P = (eye(2) - K * H) * P_pred;
end