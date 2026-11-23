% analyze_thruster_configuration.m
% Script for iterative analysis of rotational thruster configurations
% Allows you to define thruster positions and directions, visualizes them,
% and calculates the resulting torque capabilities (Roll, Pitch, Yaw).

clear; clc; close all;

%% 1. Satellite Geometry & Mass Properties Parameters
% Assuming the longitudinal axis is X, and the hexagonal face is in the Y-Z plane
L_sat = 1.0;     % Length of the satellite along X-axis (m)
R_hex = 0.5;     % Circumradius of the hexagonal cross-section (m)
F_max = 1.0;     % Maximum thrust per thruster (Newtons)

% Center of Mass (COM) assumed at origin for torque calculations
COM = [0, 0, 0]; 

% X-coordinate of the bottom face where thrusters are mounted
X_bottom = -L_sat / 2; 

%% 2. Define Thruster Configuration
% We define thrusters by their Position (P) and Direction of Thrust (D)
% P is a 1x3 vector [x, y, z] relative to COM
% D is a 1x3 unit vector [dx, dy, dz] indicating the direction the FORCE is applied
% (i.e., if thruster nozzle points in +Z, it pushes in -Z. D should be the force direction)

thrusters = []; % Initialize empty array for thrusters

% Example: 12-thruster configuration around the hexagonal base (2 per corner)
% You can iteratively modify this loop, or manually define rows of 'thrusters'
theta_corners = linspace(0, 2*pi, 7); 
theta_corners = theta_corners(1:6); % 6 corners of the hexagon

% Cant angle to avoid plume impingement on the satellite body
% -19 degrees outward tilt (Optimized for zero heat)
cant_angle_deg = -19; 
cant_rad = deg2rad(cant_angle_deg);

thruster_id = 1;
for i = 1:6
    % Corner position in Y-Z plane
    y_corner = R_hex * cos(theta_corners(i));
    z_corner = R_hex * sin(theta_corners(i));
    pos = [X_bottom, y_corner, z_corner];
    
    % Radial vector (pointing outward from center of hexagon)
    radial_dir = [0, cos(theta_corners(i)), sin(theta_corners(i))];
    
    % Mechanical Bracket Offset
    % Thrusters have physical volume. We must offset the mounting point 
    % radially outward so the hardware doesn't clip the satellite body.
    bracket_offset = 0.08; % 8 cm bracket offset
    pos = pos + radial_dir * bracket_offset;
    
    % Tangential vector
    tangent_dir = [0, -sin(theta_corners(i)), cos(theta_corners(i))];
    
    % To get bidirectional roll, alternate the tangential direction
    if mod(i, 2) == 1
        % Odd corners: point one way (e.g., +Roll)
        roll_dir = tangent_dir;
    else
        % Even corners: point the opposite way (e.g., -Roll)
        roll_dir = -tangent_dir;
    end
    
    % Thruster A: Roll Thruster
    % Combine roll direction with an outward radial cant to avoid plume impingement
    dir_A = roll_dir * cos(cant_rad) + radial_dir * sin(cant_rad);
    
    % Thruster B: Longitudinal Thruster (for Pitch/Yaw and +X translation)
    % Force direction along +X. We also cant it slightly outward (e.g., 5 deg) 
    % so the expanding plume clears the side panels completely.
    long_cant_rad = deg2rad(-15);
    dir_B = [cos(long_cant_rad), radial_dir(2)*sin(long_cant_rad), radial_dir(3)*sin(long_cant_rad)];
    
    % Add to configuration array
    % Format: [pos_x, pos_y, pos_z, dir_x, dir_y, dir_z]
    thrusters(thruster_id, :) = [pos, dir_A];
    thruster_id = thruster_id + 1;
    
    thrusters(thruster_id, :) = [pos, dir_B];
    thruster_id = thruster_id + 1;
end

% You can also add manual thrusters here:
% thrusters(end+1, :) = [X_bottom, 0, 0,  1, 0, 0]; % Example center thruster pushing +X

%% Extract and Normalize Vectors
P = thrusters(:, 1:3);
D = thrusters(:, 4:6);

% Normalize direction vectors to ensure they are unit vectors
for i = 1:size(D, 1)
    if norm(D(i,:)) > 0
        D(i,:) = D(i,:) / norm(D(i,:));
    end
end
num_thrusters = size(thrusters, 1);

%% 3. Calculate Torque Control Matrix (B)
% Torque = r x F
% tau = B * u, where u is the thrust vector [u1; u2; ... uN]'
B = zeros(3, num_thrusters);
for i = 1:num_thrusters
    r = P(i, :) - COM;
    B(:, i) = cross(r', D(i, :)');
end

% Introduce 2-degree random misalignment to simulate manufacturing errors
disp('Applying up to 2-deg random misalignment to physical thruster mounting...');
max_err_deg = 2.0;
B_actual = zeros(3, num_thrusters);
for i = 1:num_thrusters
    % Generate a random unit vector for rotation axis
    rand_axis = randn(1,3); rand_axis = rand_axis / norm(rand_axis);
    theta_err = rand() * deg2rad(max_err_deg);
    
    % Use Rodrigues' rotation formula (avoids needing Aerospace toolbox)
    v = D(i,:); k = rand_axis;
    D_actual = v*cos(theta_err) + cross(k,v)*sin(theta_err) + k*dot(k,v)*(1-cos(theta_err));
    D_actual = D_actual / norm(D_actual);
    
    r = P(i, :) - COM;
    B_actual(:, i) = cross(r', D_actual');
end

%% 4. Evaluate Control Authority
disp('======================================================');
disp('          THRUSTER CONFIGURATION ANALYSIS             ');
disp('======================================================');
fprintf('Number of Thrusters: %d\n\n', num_thrusters);

% Display the control matrix
disp('Control Matrix B (Torque generated per 1N of thrust):');
disp('Rows: [Roll (X), Pitch (Y), Yaw (Z)] | Cols: Thrusters 1 to N');
disp(B);

% Calculate maximum pure torque in each axis (+X, -X, +Y, -Y, +Z, -Z)
max_torques = zeros(1,6); 
axis_labels = {'+Roll (X)', '-Roll (X)', '+Pitch (Y)', '-Pitch (Y)', '+Yaw (Z)', '-Yaw (Z)'};
target_dirs = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1]';

disp('Maximum Pure Torques (assuming F_max per thruster, nullifying cross-torques):');
try
    % Use linprog to find max torque in each direction with zero cross-torque
    options = optimoptions('linprog','Display','none');
    for i = 1:6
        target_tau = target_dirs(:, i);
        
        % We want to maximize T such that B*u = T * target_tau
        % Let x = [u; T]. Minimize -T (i.e. c' * x)
        c = [zeros(num_thrusters, 1); -1];
        
        % Equality constraints: B*u - T*target_tau = 0
        Aeq = [B, -target_tau];
        beq = [0; 0; 0];
        
        % Bounds: 0 <= u_i <= F_max, T >= 0
        lb = [zeros(num_thrusters, 1); 0];
        ub = [F_max * ones(num_thrusters, 1); Inf];
        
        [x, ~, exitflag] = linprog(c, [], [], Aeq, beq, lb, ub, options);
        
        if exitflag == 1 && x(end) > 1e-6
            max_torques(i) = x(end);
            fprintf('  %10s : %8.3f Nm\n', axis_labels{i}, max_torques(i));
        else
            fprintf('  %10s : INCAPABLE (Cannot generate pure torque)\n', axis_labels{i});
        end
    end
catch
    disp('Optimization toolbox (linprog) not available for pure torque calculation.');
    disp('Displaying maximum possible torque per axis ignoring cross-coupling:');
    for i = 1:6
        target_tau = target_dirs(:, i);
        contributions = target_tau' * B;
        max_t = sum(contributions(contributions > 0)) * F_max;
        fprintf('  %10s : %8.3f Nm (Upper bound, ignoring cross-torque)\n', axis_labels{i}, max_t);
    end
end
disp('======================================================');

%% 5. Visualization
fig = figure('Name', 'Thruster Configuration Viewer', 'Color', 'w', 'Position', [100, 100, 900, 700]);
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
xlabel(ax, 'X (Roll/Longitudinal)'); ylabel(ax, 'Y (Pitch)'); zlabel(ax, 'Z (Yaw)');

% Draw Hexagonal Satellite Body with Heatmap Grid
hex_y = R_hex * cos(theta_corners);
hex_z = R_hex * sin(theta_corners);

% Top and Bottom faces
fill3(X_bottom * ones(1,6), hex_y, hex_z, [0.8 0.8 0.8], 'FaceAlpha', 0.2, 'EdgeColor', 'k');
fill3((X_bottom + L_sat) * ones(1,6), hex_y, hex_z, [0.8 0.8 0.8], 'FaceAlpha', 0.2, 'EdgeColor', 'k');

disp('Generating Simons Model analytical density heatmap on the bus panels...');
% Define grid resolution for panels
nx = 20; % Points along length
nyz = 10; % Points across each panel width
x_grid = linspace(X_bottom, X_bottom+L_sat, nx);

max_density = 0;
for pnl = 1:6
    y1 = hex_y(pnl); z1 = hex_z(pnl);
    if pnl < 6; y2 = hex_y(pnl+1); z2 = hex_z(pnl+1);
    else;       y2 = hex_y(1);     z2 = hex_z(1); end
    
    y_lin = linspace(y1, y2, nyz);
    z_lin = linspace(z1, z2, nyz);
    
    [Xg, ~] = meshgrid(x_grid, y_lin);
    Yg = repmat(y_lin', 1, nx);
    Zg = repmat(z_lin', 1, nx);
    
    % Calculate analytical density at each grid point
    Density_Grid = zeros(nyz, nx);
    for j = 1:nyz
        for k = 1:nx
            pt = [Xg(j,k), Yg(j,k), Zg(j,k)];
            rho_total = 0;
            for t = 1:num_thrusters
                r_vec = pt - P(t,:);
                r_dist = norm(r_vec);
                if r_dist < 1e-3; continue; end
                
                plume_centerline = -D(t,:); % Plume expands opposite to force
                cos_theta = dot(r_vec/r_dist, plume_centerline);
                
                if cos_theta > 0 % Only calculate if point is "in front" of nozzle
                    % Simons analytical density equation: rho = A / r^2 * cos^n(theta)
                    A_factor = 1.0; % Arbitrary scaling factor for visualization
                    n_plume = 8; % Focus parameter (higher = tighter beam)
                    rho_total = rho_total + (A_factor / r_dist^2) * (cos_theta)^n_plume;
                end
            end
            Density_Grid(j,k) = rho_total;
        end
    end
    max_density = max(max_density, max(Density_Grid(:)));
    
    % Plot the panel as a colored surface
    surf(ax, Xg, Yg, Zg, Density_Grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
end

% Set heatmap colors
colormap(ax, hot);
if max_density > 0
    caxis(ax, [0, max(0.1, max_density)]);
    colorbar(ax, 'Location', 'eastoutside', 'Color', 'k');
end

% Draw COM
plot3(COM(1), COM(2), COM(3), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
text(COM(1), COM(2), COM(3)+0.1, ' COM', 'Color', 'r', 'FontWeight', 'bold');

% Draw Thrusters (Physical hardware and force vectors)
quiver_scale = 0.3; % Scale for the force arrows
hardware_length = 0.15; % 15 cm physical thruster length
hardware_radius = 0.04; % 4 cm physical thruster radius

for i = 1:num_thrusters
    % 1. Draw physical thruster hardware (grey cone extending along +D)
    [Xc_hw, Yc_hw, Zc_hw] = cylinder([hardware_radius, 0], 12); % Cone tapering to nozzle at P
    Zc_hw = Zc_hw * hardware_length;
    
    % Rotate hardware cone to align with D(i,:)
    hw_dir = D(i, :);
    v_orig = [0, 0, 1];
    v_dest = hw_dir / norm(hw_dir);
    if norm(cross(v_orig, v_dest)) < 1e-6 && dot(v_orig, v_dest) < 0
        R_mat_hw = diag([1, -1, -1]);
    elseif norm(cross(v_orig, v_dest)) < 1e-6
        R_mat_hw = eye(3);
    else
        v = cross(v_orig, v_dest); s = norm(v); c = dot(v_orig, v_dest);
        vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        R_mat_hw = eye(3) + vx + vx^2 * ((1-c)/s^2);
    end
    
    for j = 1:size(Xc_hw, 1)
        for k = 1:size(Xc_hw, 2)
            pt = [Xc_hw(j,k), Yc_hw(j,k), Zc_hw(j,k)] * R_mat_hw';
            Xc_hw(j,k) = pt(1) + P(i,1);
            Yc_hw(j,k) = pt(2) + P(i,2);
            Zc_hw(j,k) = pt(3) + P(i,3);
        end
    end
    surf(ax, Xc_hw, Yc_hw, Zc_hw, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    
    % 2. Plot Force vector (Red Arrow)
    quiver3(P(i,1), P(i,2), P(i,3), D(i,1)*quiver_scale, D(i,2)*quiver_scale, D(i,3)*quiver_scale, 0, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
    % Label
    text(P(i,1) + D(i,1)*quiver_scale, P(i,2) + D(i,2)*quiver_scale, P(i,3) + D(i,3)*quiver_scale, ...
        sprintf('T%d', i), 'FontSize', 10, 'FontWeight', 'bold');
end

%% 6. Plume Impingement Analysis & Visualization
% Model the exhaust plume as a cone pointing OPPOSITE to the force vector.
plume_length = 0.8; % Length of the visible plume cone (m)
plume_half_angle_deg = 15; % Half-angle of the exhaust cone (deg)
plume_radius = plume_length * tan(deg2rad(plume_half_angle_deg));

disp('Analyzing exhaust plumes for bus impingement...');
impingement_results = zeros(num_thrusters, 1);

for i = 1:num_thrusters
    % Plume direction is exactly opposite to the force direction
    plume_dir = -D(i, :);
    
    % Create a cone mesh (pointing along +Z by default)
    % Using more vertices for a higher-resolution collision check
    [Xc, Yc, Zc] = cylinder([0, plume_radius], 30); 
    Zc = Zc * plume_length; % Scale length
    
    % Calculate rotation matrix to point cone along plume_dir
    v_orig = [0, 0, 1];
    v_dest = plume_dir / norm(plume_dir);
    
    if norm(cross(v_orig, v_dest)) < 1e-6 && dot(v_orig, v_dest) < 0
        R_mat = diag([1, -1, -1]); % 180 deg flip
    elseif norm(cross(v_orig, v_dest)) < 1e-6
        R_mat = eye(3);
    else
        v = cross(v_orig, v_dest);
        s = norm(v);
        c = dot(v_orig, v_dest);
        vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        R_mat = eye(3) + vx + vx^2 * ((1-c)/s^2);
    end
    
    % Apply rotation and translation to the cone vertices
    is_impinging = false;
    for j = 1:size(Xc, 1)
        for k = 1:size(Xc, 2)
            pt = [Xc(j,k), Yc(j,k), Zc(j,k)] * R_mat';
            pt_x = pt(1) + P(i,1);
            pt_y = pt(2) + P(i,2);
            pt_z = pt(3) + P(i,3);
            
            Xc(j,k) = pt_x;
            Yc(j,k) = pt_y;
            Zc(j,k) = pt_z;
            
            % Mathematical Impingement Check:
            % Skip the very tip of the nozzle (j=1) to prevent false positives from the origin point
            if j > 1 
                % Check if point is longitudinally alongside the bus 
                % (add 1mm tolerance to ignore flush mounting errors)
                if pt_x > (X_bottom + 1e-3) && pt_x < (X_bottom + L_sat)
                    % Check if point is inside the hexagonal cross-section
                    if inpolygon(pt_y, pt_z, hex_y, hex_z)
                        is_impinging = true;
                    end
                end
            end
        end
    end
    
    % Color code and log the result
    if is_impinging
        impingement_results(i) = 1;
        fprintf('  WARNING: Thruster T%d plume is mathematically intersecting the bus!\n', i);
        % Draw impinging plume as solid RED
        surf(ax, Xc, Yc, Zc, 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.4, 'LineStyle', 'none');
    else
        % Draw safe plume as translucent GREEN
        surf(ax, Xc, Yc, Zc, 'FaceColor', [0, 1, 0], 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'LineStyle', 'none');
    end
end

if sum(impingement_results) == 0
    disp('  SUCCESS: Zero plume impingement detected across all thrusters.');
else
    fprintf('  FAILED: %d thrusters have impingement issues. Increase cant angles.\n', sum(impingement_results));
end

title(ax, 'Rotational Thruster Configuration & Plume Analysis', 'FontSize', 14);
hold(ax, 'off');
rotate3d(fig, 'on');

disp('Visualization ready. Safe plumes are green, impinging plumes are red.');

%% 7. Dynamic Attitude Control Simulation (PD + PWM)
disp(' ');
disp('======================================================');
disp('          DYNAMIC PD + PWM SIMULATION                 ');
disp('======================================================');

% 7.1 Spacecraft Inertia Property (Assumed 500 kg uniform prism)
mass = 500; % kg
% Inertia approximation: Ixx = 1/2*m*r^2, Iyy/Izz = 1/12*m*(3*r^2 + h^2)
Ixx = 0.5 * mass * R_hex^2; 
Iyy = (1/12) * mass * (3*R_hex^2 + L_sat^2);
Izz = Iyy;
J = diag([Ixx, Iyy, Izz]);
invJ = inv(J);

% 7.2 Simulation Parameters
sim_time = 60;      % Total simulation time (seconds)
dt = 0.1;           % Control loop period (10 Hz frequency)
N_steps = round(sim_time / dt);
time = (0:N_steps-1) * dt;

% 7.3 PD Controller Gains (Tuned heuristically for the inertia)
Kp = diag([5, 5, 5]);    % Proportional gain
Kd = diag([15, 15, 15]); % Derivative gain

% 7.4 State Initialization
% Initial attitude error: 15 deg roll, -10 deg pitch, 20 deg yaw
euler_angles = deg2rad([15; -10; 20]);
omega = [0; 0; 0]; % Initially at rest

% Storage arrays for plotting
history_euler = zeros(3, N_steps);
history_omega = zeros(3, N_steps);
history_cmd   = zeros(num_thrusters, N_steps);

disp('Simulating attitude recovery maneuver...');

% 7.5 Simulation Loop
for k = 1:N_steps
    % Store current state
    history_euler(:, k) = rad2deg(euler_angles);
    history_omega(:, k) = rad2deg(omega);
    
    % PD Controller calculates desired restoring torque
    tau_cmd = -Kp * euler_angles - Kd * omega;
    
    % Thruster Allocator (Non-negative Least Squares)
    % Finds continuous force T_cmd (>= 0) such that B * T_cmd ~= tau_cmd
    options = optimset('Display','none');
    T_cmd = lsqnonneg(B, tau_cmd, options);
    
    % PWM mapping (Duty Cycle = Commanded Thrust / Max Thrust)
    duty_cycle = min(T_cmd / F_max, 1.0);
    
    % Store commands
    history_cmd(:, k) = duty_cycle;
    
    % Actual torque applied to spacecraft (average over dt)
    % The physics engine uses B_actual which contains the hidden 2-degree mounting errors!
    tau_actual = B_actual * (duty_cycle * F_max);
    
    % Kinematics (Euler Angle Rates)
    phi = euler_angles(1); theta = euler_angles(2);
    euler_rates = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                   0, cos(phi),           -sin(phi);
                   0, sin(phi)/cos(theta), cos(phi)/cos(theta)] * omega;
               
    % Rigid Body Dynamics (Euler Equations)
    omega_dot = invJ * (tau_actual - cross(omega, J*omega));
    
    % Integrate (Simple Euler Integration)
    euler_angles = euler_angles + euler_rates * dt;
    omega = omega + omega_dot * dt;
end

disp('Simulation complete.');

%% 8. Plot Dynamic Results
fig2 = figure('Name', 'Attitude Control Simulation Results', 'Color', 'w', 'Position', [150, 150, 1000, 600]);

% Plot Euler Angles
subplot(2, 1, 1);
plot(time, history_euler(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(time, history_euler(2,:), 'g', 'LineWidth', 1.5);
plot(time, history_euler(3,:), 'b', 'LineWidth', 1.5);
grid on;
title('Satellite Attitude vs Time (Recovery to Zero)');
xlabel('Time (s)');
ylabel('Euler Angles (deg)');
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)');

% Plot Thruster Commands (Duty Cycle)
subplot(2, 1, 2);
colors = lines(num_thrusters);
hold on;
for i = 1:num_thrusters
    % Only plot if the thruster was actually fired during the simulation
    if sum(history_cmd(i,:)) > 0.01
        plot(time, history_cmd(i,:), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('T%d', i));
    end
end
grid on;
title('PWM Thruster Commands (0 = Off, 1 = 100% On)');
xlabel('Time (s)');
ylabel('Duty Cycle');
legend('NumColumns', 2, 'Location', 'best');

disp('Simulation plots generated.');
