% Cart-Pole balancing using LQR with enhanced animation
% System Parameters
m = 0.1;     % Mass of the pole (kg)
M = 1.0;     % Mass of the cart (kg)
L = 0.5;     % Length of the pole (m)
g = 9.81;    % Gravitational acceleration (m/s^2)
d = 0.0;     % Damping coefficient 

% State-Space Model
% States: x (cart position), x_dot (cart velocity), theta (pole angle), theta_dot (angular velocity)
A = [0 1 0 0;
     0 0 -(m*g)/M 0;
     0 0 0 1;
     0 0 (m+M)*g/(M*L) 0];
B = [0; 1/M; 0; -1/(M*L)];
C = [1 0 0 0;  % Output: cart position 
     0 0 1 0]; % Output: pole angle
D = [0; 0];

% LQR Controller Design
Q = diag([10 1 100 10]); % State weighting matrix
R = 0.1;                 % Control input weighting
[K, ~, ~] = lqr(A, B, Q, R); % LQR gain matrix

% Closed-Loop System
Acl = A - B*K; % Closed-loop system matrix
sys_cl = ss(Acl, B, C, D);

% Simulation Parameters
tspan = 0:0.01:10; 
x0 = [0; 0; 0.1; 0]; % Initial condition: small angle perturbation (0.1 rad)
[t, x] = ode45(@(t, x) Acl*x, tspan, x0); 

% Extract states
x_cart = x(:, 1);     % Cart position
theta = x(:, 3);      % Pole angle
u_control = -K*x';    % Control input (force)

% Plot Results (Static Plots)
figure('Name', 'Cart-Pole Simulation Results');
subplot(3,1,1);
plot(t, x_cart, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Cart Position (m)');
title('Cart Position'); grid on;

subplot(3,1,2);
plot(t, theta*180/pi, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Pole Angle (deg)');
title('Pole Angle'); grid on;

subplot(3,1,3);
plot(t, u_control', 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Control Force (N)');
title('Control Input'); grid on;

% Animation
figure('Name', 'Cart-Pole Animation', 'Position', [100, 100, 800, 400]);
for i = 1:5:length(t) % Step by 5 for smoother animation
    % Clear previous frame
    clf;
    hold on;
    
    % Cart and pole dimensions
    cart_width = 0.4; 
    cart_height = 0.2;
    wheel_radius = 0.05;
    pole_width = 0.02;
    pole_length = L;
    mass_radius = 0.05; 
    
    % Current state
    x_c = x_cart(i);
    th = theta(i);
    
    % Pole end position
    pole_x = x_c + pole_length*sin(th);
    pole_y = cart_height + pole_length*cos(th);
    
    % Draw cart
    rectangle('Position', [x_c - cart_width/2, wheel_radius, cart_width, cart_height], ...
              'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw wheels
    rectangle('Position', [x_c - cart_width/2 + 0.05, 0, wheel_radius*2, wheel_radius*2], ...
              'Curvature', 1, 'FaceColor', 'k');
    rectangle('Position', [x_c + cart_width/2 - 0.15, 0, wheel_radius*2, wheel_radius*2], ...
              'Curvature', 1, 'FaceColor', 'k');
    
    % Draw pole
    plot([x_c pole_x], [cart_height pole_y], 'r-', 'LineWidth', 3);
    
    % Draw mass at pole end
    rectangle('Position', [pole_x - mass_radius, pole_y - mass_radius, mass_radius*2, mass_radius*2], ...
              'Curvature', 1, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'k');
    
    % Draw ground and track
    plot([-3 3], [0 0], 'k-', 'LineWidth', 2); % Ground
    plot([-3 3], [wheel_radius wheel_radius], 'k--', 'LineWidth', 1); % Track
    
    % Set plot properties
    axis([-2 2 -0.3 1.2]); 
    axis equal; 
    xlabel('Position (m)'); ylabel('Height (m)');
    title(sprintf('Cart-Pole Balancing (LQR), Time: %.2f s', t(i)));
    grid on;
    
    % Visualize control input
    force_scale = 0.05; 
    force = u_control(i) * force_scale;
    quiver(x_c, cart_height + 0.1, force, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    hold off;
    drawnow;
    pause(0.02); 
end