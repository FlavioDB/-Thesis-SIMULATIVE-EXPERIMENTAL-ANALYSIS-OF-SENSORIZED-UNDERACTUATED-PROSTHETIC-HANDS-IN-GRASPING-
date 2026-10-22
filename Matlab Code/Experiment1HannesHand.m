%Experiment 1 Hannes Hand

clc; clear; close all;

disp('Building HannesHand...');

%% 1. THUMB (1 DoF)
% URDF joint: 'thumb_bottom_wheel' (distance from palm to base of thumb)
xyz_thumb = [0.0404, -0.0313, 0.0309] * 1000; % Convert m to mm
rpy_thumb = [-0.0002, -1.0978, -2.8786];
T_base_thumb = URDF_T(xyz_thumb, rpy_thumb);

% DH Table: [alpha, a, d, theta]
% The thumb URDF doesn't have a top joint, so we estimate the link 
% length measured on Solid Edge Cad 
DH_thumb = [0, 60, 0, 0]; 
q_thumb  = [0];
F_thumb  = SGmakeFinger(DH_thumb, T_base_thumb, q_thumb);

%% 2. INDEX FINGER (2 DoF)
% Base Joint: 'index_bottom_wheel'
xyz_idx_base = [0.0988, 0.0028, 0.0238] * 1000; %Joint from palm to index_base
rpy_idx_base = [0.0004, -0.2858, 1.9981];
T_base_idx = URDF_T(xyz_idx_base, rpy_idx_base);

% Link 1 (Distance to 'index_top_wheel')
xyz_idx_top = [-0.0132, -0.0275, 0.0007] * 1000;
% Calculate length (Pythagorean theorem on X and Y)
L1_idx = sqrt(xyz_idx_top(1)^2 + xyz_idx_top(2)^2); 
L2_idx = 38; % Estimated top phalanx length

% DH Table
DH_idx = [0, L1_idx, 0, 0; 
          0, L2_idx, 0, 0];
% To make the DH X-axis point towards the next joint, we add the angle
angle_idx = atan2(xyz_idx_top(2), xyz_idx_top(1)); %arctg(y/x) per avere l'inclinazione del dito
T_base_idx = T_base_idx * T_rotz(angle_idx); % Align axis

F_index = SGmakeFinger(DH_idx, T_base_idx, [0;0]);

%% 3. MIDDLE FINGER (2 DoF)
% Base Joint: 'middle_bottom_wheel'
xyz_mid_base = [0.0999, 0.0083, 0.0014] * 1000;
rpy_mid_base = [0.0002, -0.1704, 2.3608];
T_base_mid = URDF_T(xyz_mid_base, rpy_mid_base);

% Link 1 (Distance to 'middle_top_wheel')
xyz_mid_top = [-0.0265, -0.0205, 0.0006] * 1000;
L1_mid = sqrt(xyz_mid_top(1)^2 + xyz_mid_top(2)^2);
L2_mid = 45; 

DH_mid = [0, L1_mid, 0, 0; 0, L2_mid, 0, 0];
angle_mid = atan2(xyz_mid_top(2), xyz_mid_top(1));
T_base_mid = T_base_mid * T_rotz(angle_mid);

F_middle = SGmakeFinger(DH_mid, T_base_mid, [0;0]);

%% 4. RING FINGER (2 DoF)
% Base Joint: 'ring_bottom_wheel'
xyz_rng_base = [0.0971, 0.0059, -0.0213] * 1000;
rpy_rng_base = [-0.0000, -0.1222, -3.1222];
T_base_rng = URDF_T(xyz_rng_base, rpy_rng_base);

% Link 1 (Distance to 'ring_top_wheel')
xyz_rng_top = [-0.0302, 0.0042, 0.0006] * 1000;
L1_rng = sqrt(xyz_rng_top(1)^2 + xyz_rng_top(2)^2);
L2_rng = 40; 

DH_rng = [0, L1_rng, 0, 0; 0, L2_rng, 0, 0];
angle_rng = atan2(xyz_rng_top(2), xyz_rng_top(1));
T_base_rng = T_base_rng * T_rotz(angle_rng);

F_ring = SGmakeFinger(DH_rng, T_base_rng, [0;0]);

%% 5. LITTLE FINGER (2 DoF)
% Base Joint: 'little_bottom_wheel'
xyz_lit_base = [0.0835, -0.0017, -0.0411] * 1000;
rpy_lit_base = [-0.0359, -0.2727, -1.8907];
T_base_lit = URDF_T(xyz_lit_base, rpy_lit_base);

% Link 1 (Distance to 'little_top_wheel')
xyz_lit_top = [-0.0151, 0.0217, 0.0004] * 1000;
L1_lit = sqrt(xyz_lit_top(1)^2 + xyz_lit_top(2)^2);
L2_lit = 30;

DH_lit = [0, L1_lit, 0, 0; 0, L2_lit, 0, 0];
angle_lit = atan2(xyz_lit_top(2), xyz_lit_top(1));
T_base_lit = T_base_lit * T_rotz(angle_lit);

F_little = SGmakeFinger(DH_lit, T_base_lit, [0;0]);

%% 6. ASSEMBLE THE HAND
F_array = {F_little, F_ring, F_middle, F_index, F_thumb}; %mano sinistra con spigolo dietro
%F_array = {F_index, F_middle, F_ring, F_little, F_thumb};

%F_array = {F_thumb, F_little, F_ring, F_middle, F_index}; 
%F_array = {F_thumb, F_index, F_middle, F_ring, F_little};
custom_hand = SGmakeHand(F_array);

%% 7. DEFINE THE "POWER GRASP" SYNERGY (S)
%power grasp -> large portions of the hand used, not only fingertips
% The hand has 9 joints. We want 1 Synergy (z). 
% Therefore, S must be a 9x1 vector.
S_power = zeros(custom_hand.m, 1);

% --- TUNING THE FINGERS (Joints 1 to 8) ---
% Each finger has 2 joints (Base and Top).
% We want them to curl inwards.

% Little Finger (Joints 1, 2)
S_power(1:2) = -1.0; 

% Ring Finger (Joints 3, 4)
S_power(3:4) = -1.0; 

% Middle Finger (Joints 5, 6)
S_power(5:6) = -1.0; 

% Index Finger (Joints 7, 8)
S_power(7:8) = -1.0; 

% --- TUNING THE THUMB (Joint 9) ---
% The Thumb has 1 joint (Base).
% Because of its URDF orientation, closing it might require 
% a negative or positive value, and it might need to move slower.
% Let's start with a slower closing speed.
S_power(9) = 1.8; 

% Define the resting pose (straight fingers)
q_ref = zeros(custom_hand.m, 1);

% Attach the Synergy Matrix to the Hand
custom_hand = SGdefineSynergies(custom_hand, S_power, q_ref);

%% 8. ANIMATE THE GRASP
% Set simulation limits
z_start = 0.0; % Open
z_end   = 1.25; % Closed (1.2 radians ~ 70 degrees)
n_steps = 20;  % Smoothness (number frames = animation lenght)
az = 45;
el = 15;
% Setup Figure
figure('Name', 'Custom Hand Power Grasp', 'Color', 'w');
axis equal; grid on; view(3);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis([-100 200 -100 100 -100 150]); % Lock camera bounds
view(az, el); % Looking from -Y axis, slightly elevated by 15 degrees
%set(gca, 'ZDir', 'reverse'); % Reverses the Z-axis, perfectly mirroring the plot
hold on;

disp('Animating Custom Hand Grasp...');

% The Animation Loop
for i = 1:n_steps
    % Linear interpolation of z
    alpha = i / n_steps; 
    z_current = (1 - alpha)*z_start + (alpha)*z_end;
    
    % Calculate new joint angles: q = q_ref + S * z
    hand_moved = SGactivateSynergies(custom_hand, z_current);
    
    % Render Frame
    cla; % Clear previous frame
    SGplotHand(hand_moved);
    
    % Re-apply camera settings (cla deletes them)
    axis equal; grid on; view(-140, 30);
    axis([-100 200 -100 100 -100 150]);
    view(az, el); % Looking from -Y axis, slightly elevated by 15 degrees
    %set(gca, 'ZDir', 'reverse'); % Reverses the Z-axis, perfectly mirroring the plot
    title(sprintf('Close Fist Hannes (z = %.2f rad)', z_current));
    
    drawnow limitrate;
end

disp('Grasp Simulation Complete!');

%% --- HELPER FUNCTIONS ---
function T = URDF_T(xyz, rpy)
    % Converts URDF xyz and rpy vectors into a 4x4 Homogeneous Matrix
    % URDF rpy applies rotations in order: X, then Y, then Z
    % (n.b.:rotations wrt global fixed frame)
    
    % Create individual rotation matrices
    Rx = SGrotx(rpy(1)); %argument is the rotation angle to create a rotation about X, Y, or Z
    Ry = SGroty(rpy(2));
    Rz = SGrotz(rpy(3));
    
    % Combine rotations (URDF standard is Rz * Ry * Rx)(cfr. global fixed f)
    R_total = Rz * Ry * Rx;
    
    % Build 4x4 homogeneous matrix matrix
    T = eye(4);
    T(1:3, 1:3) = R_total;
    T(1:3, 4)   = xyz'; % Add translation
end

function T = T_rotz(angle)
    % Converts a 3x3 rotation into a 4x4 homogeneous matrix, necessary for
    % multiplications
    T = eye(4);
    T(1:3, 1:3) = SGrotz(angle);
end