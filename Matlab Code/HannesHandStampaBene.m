%% Hannes Hand Clean Photoshoot Script
% Generates a clean, axis-free image of the hand mid-grasp.

clc; clear; close all;

% Set your preferred camera angle here!
az = 155;
el = 20;

%% 1. BUILD THE HANNES HAND (From URDF data)
T_base_thumb = URDF_T([40.4, -31.3, 30.9], [-0.0002, -1.0978, -2.8786]);
F_thumb = SGmakeFinger([0, 60, 0, 0], T_base_thumb, 0);

T_base_idx = URDF_T([98.8, 2.7, 23.7], [0.0003, -0.2857, 1.9981]);
angle_idx = atan2(-27.5, -13.2);
T_base_idx = T_base_idx * T_rotz(angle_idx);
F_index = SGmakeFinger([0, 30.5, 0, 0; 0, 38, 0, 0], T_base_idx, [0;0]);

T_base_mid = URDF_T([99.9, 8.3, 1.4], [0.0002, -0.1704, 2.3608]);
angle_mid = atan2(-20.5, -26.5);
T_base_mid = T_base_mid * T_rotz(angle_mid);
F_middle = SGmakeFinger([0, 33.5, 0, 0; 0, 45, 0, 0], T_base_mid, [0;0]);

T_base_rng = URDF_T([97.1, 5.9, -21.3], [0, -0.1222, -3.1222]);
angle_rng = atan2(4.2, -30.2);
T_base_rng = T_base_rng * T_rotz(angle_rng);
F_ring = SGmakeFinger([0, 30.5, 0, 0; 0, 40, 0, 0], T_base_rng, [0;0]);

T_base_lit = URDF_T([83.5, -1.7, -41.1], [-0.0359, -0.2727, -1.8907]);
angle_lit = atan2(21.7, -15.1);
T_base_lit = T_base_lit * T_rotz(angle_lit);
F_little = SGmakeFinger([0, 26.4, 0, 0; 0, 30, 0, 0], T_base_lit, [0;0]);

% Assemble
F_array = {F_thumb, F_index, F_middle, F_ring, F_little}; 
hannes_hand = SGmakeHand(F_array);

%% 2. DEFINE THE HANNES S-MATRIX
S_hannes = [-1.3; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];

%% 3. SIMULATE THE MID-GRASP
fig = figure('Name', 'Hannes Photoshoot', 'Color', 'w', 'Position', [200, 200, 800, 600]);

% --- Tuning the pose ---
step_size = 0.05; 
max_steps = 18; % <-- CHANGE THIS NUMBER to curl the fingers more or less!
q_current = zeros(9,1);

disp('Posing the hand...');

for i = 1:max_steps
    delta_q = zeros(9,1);
    for f = 1:5
        if f == 1 
            j_idx = 1; 
        else
            j_idx = [ (f-1)*2, (f-1)*2 + 1 ]; 
        end
        delta_q(j_idx) = S_hannes(j_idx) * step_size;
    end
    
    q_current = q_current + delta_q;
    hannes_hand = SGmoveHand(hannes_hand, q_current);
    
    % --- CLEAN PLOTTING ---
    cla;
    SGplotHand(hannes_hand); 
    
    % Keep proportions and camera steady
    axis equal; 
    axis([-50 150 -100 100 -80 80]);
    view(az, el);
    
    % MAGIC COMMANDS: Turn off all grids, text, and axes boxes
    axis off; 
    
    drawnow;
end

disp('Photoshoot ready!');

%% --- HELPER FUNCTIONS ---
function T = URDF_T(xyz, rpy)
    Rx = SGrotx(rpy(1)); Ry = SGroty(rpy(2)); Rz = SGrotz(rpy(3));
    T = eye(4); T(1:3, 1:3) = Rz * Ry * Rx; T(1:3, 4) = xyz';
end

function T = T_rotz(angle)
    T = eye(4); T(1:3, 1:3) = SGrotz(angle);
end