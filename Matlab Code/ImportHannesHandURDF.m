%% Building Hannes Hand from URDF
% Translating CAD URDF data into SynGrasp Denavit-Hartenberg kinematics

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
angle_idx = atan2(xyz_idx_top(2), xyz_idx_top(1)); 
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
%F_array = {F_thumb, F_index, F_middle,  F_ring, F_little}; %mano sinistra con spigolo dove non deve essere

custom_hand = SGmakeHand(F_array);

% Define a simple 1:1 Synergy (1 motor moves all 9 joints equally)
S_matrix = ones(custom_hand.m, 1);
custom_hand = SGdefineSynergies(custom_hand, S_matrix, zeros(9,1));

%% 7. VISUALIZE
figure('Name', 'URDF Imported Hand', 'Color', 'w');
SGplotHand(custom_hand);
hold on;
SGhandFrames(custom_hand, 10); % Plot the frames

axis equal; grid on; view(3);
%set(gca, 'ZDir', 'reverse'); % Reverses the Z-axis, mirroring the plot ->
%makes a right hand from a left one
title('Hannes Hand imported from URDF');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
disp('Figure complete');

%% --- HELPER FUNCTIONS ---
function T = URDF_T(xyz, rpy)
    % Converts URDF xyz and rpy vectors into a 4x4 Homogeneous Matrix
    % URDF rpy applies rotations in order: X, then Y, then Z, rotations wrt global fixed frame
    
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