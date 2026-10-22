%% Thesis Hannes Differential Adaptive Grasping
% Simulating the cable-driven differential mechanism on a Sphere.

clc; clear; close all;
az = 155;
el = 20;

%% 1. BUILD THE HANNES HAND (From URDF data)
%same data as previous experiments taken from URDF
% --- THUMB ---
T_base_thumb = URDF_T([40.4, -31.3, 30.9], [-0.0002, -1.0978, -2.8786]);
F_thumb = SGmakeFinger([0, 60, 0, 0], T_base_thumb, 0);
% --- INDEX ---
T_base_idx = URDF_T([98.8, 2.7, 23.7], [0.0003, -0.2857, 1.9981]);
angle_idx = atan2(-27.5, -13.2);
T_base_idx = T_base_idx * T_rotz(angle_idx);
F_index = SGmakeFinger([0, 30.5, 0, 0; 0, 38, 0, 0], T_base_idx, [0;0]);
% --- MIDDLE ---
T_base_mid = URDF_T([99.9, 8.3, 1.4], [0.0002, -0.1704, 2.3608]);
angle_mid = atan2(-20.5, -26.5);
T_base_mid = T_base_mid * T_rotz(angle_mid);
F_middle = SGmakeFinger([0, 33.5, 0, 0; 0, 45, 0, 0], T_base_mid, [0;0]);
% --- RING ---
T_base_rng = URDF_T([97.1, 5.9, -21.3], [0, -0.1222, -3.1222]);
angle_rng = atan2(4.2, -30.2);
T_base_rng = T_base_rng * T_rotz(angle_rng);
F_ring = SGmakeFinger([0, 30.5, 0, 0; 0, 40, 0, 0], T_base_rng, [0;0]);
% --- LITTLE ---
T_base_lit = URDF_T([83.5, -1.7, -41.1], [-0.0359, -0.2727, -1.8907]);
angle_lit = atan2(21.7, -15.1);
T_base_lit = T_base_lit * T_rotz(angle_lit);
F_little = SGmakeFinger([0, 26.4, 0, 0; 0, 30, 0, 0], T_base_lit, [0;0]);

% Assemble
%F_array = {F_little, F_ring, F_middle, F_index, F_thumb}; %mano destra con angolo sotto il pollice bleah
%F_array = {F_thumb, F_little, F_ring, F_middle, F_index}; %dx con angolo sopra indice
F_array = {F_thumb, F_index, F_middle, F_ring, F_little}; %dx con angolo sopra il mignolo

hannes_hand = SGmakeHand(F_array);

%% 2. DEFINE THE HANNES S-MATRIX (Differential Transmission)
% The paper states MCP torque is roughly twice the PIP torque.
% In our kinematic closing loop, we will set the closing speeds.
% Order: Thumb(1), Index(2), Middle(2), Ring(2), Little(2). Total = 9.

S_hannes = zeros(9, 1);
% MCP Joints (Base) close slightly faster/stronger
% PIP Joints (Top) close normally
% --- THUMB ---
% Joint 1: Thumb Base
S_hannes(1) = -1.0; 
% --- INDEX ---
S_hannes(2) = 1.8; % MCP (Base)
S_hannes(3) = 0.9; % PIP (Top)
% --- MIDDLE ---
S_hannes(4) = 1.8; % MCP (Base)
S_hannes(5) = 0.9; % PIP (Top)
% --- RING ---
S_hannes(6) = 1.8; % MCP (Base)
S_hannes(7) = 0.9; % PIP (Top)
% --- LITTLE ---
S_hannes(8) = 1.8; % MCP (Base)
S_hannes(9) = 0.9; % PIP (Top)

%% 3. CREATE THE OBJECT (Sphere)
% Let's put a sphere in the palm
obj_radius = 30;
H_obj = eye(4);
H_obj(1:3, 4) = [50,  15, 5]; % Center of the palm workspace
object = SGsphere(H_obj, obj_radius, 40);

%% 4. SIMULATE THE DIFFERENTIAL GRASP
fingers_contact = zeros(5, 1);
figure('Name', 'Hannes Differential Grasp', 'Color', 'w');
axis equal; grid on;
axis([-50 150 -100 100 -80 80]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view (az, el);
disp('Starting Adaptive Differential Closure...');

% Simulation variables
step_size = 0.01; % How much 'z' increases per frame
max_steps = 200;
q_current = zeros(9,1);

% Keep track of which fingers are blocked by the object
% 1 = Moving, 0 = Blocked
finger_active = ones(5, 1); 

for i = 1:max_steps
    % Build the movement vector for this step
    delta_q = zeros(9,1);
    
   % Check each finger individually (The Differential Logic)
    for f = 1:5
        if finger_active(f) == 1
            
            % Find which joints belong to this finger mathematically
            % (Thumb is f=1, Index is f=2, etc.)
            if f == 1 
                j_idx = 1; % Thumb only has 1 joint
            else
                % Math to find the 2 joints for fingers 2 to 5
                j_idx = [ (f-1)*2, (f-1)*2 + 1 ]; %matematica PAZZESCA
            end
            
            % Add the movement for this finger's joints
            delta_q(j_idx) = S_hannes(j_idx) * step_size;
        end
    end
    
    % Update joint angles
    q_current = q_current + delta_q;
    hannes_hand = SGmoveHand(hannes_hand, q_current);

    % --- MECHANICAL JOINT LIMITS (Stop crossing into the palm) ---
    max_angle = 2.7; % about 150°
    
    for f = 1:5
        if finger_active(f) == 1
            % Find indices
            if f == 1 
                j_idx = 1;
            else
                j_idx = [ (f-1)*2, (f-1)*2 + 1 ];
            end
            
            % Check if this finger has curled too much
            if any(q_current(j_idx) > max_angle)
                finger_active(f) = 0; % Lock this finger!
                 finger_names = {'Thumb', 'Index', 'Middle', 'Ring', 'Little'};
                 disp([finger_names{f}, ' finger reached mechanical limit (stopped).']);
            end
        end
    end
    
    % --- COLLISION DETECTION ---
    % We must check one finger at a time
    for f = 1:5
        if finger_active(f) == 1 % Only check fingers that are still moving
            
            % SGcontactDetection(hand, object, finger_index)
            cp_mat = SGcontactDetection(hannes_hand, object, f);
            
            % If the returned matrix is NOT empty, a collision happened!
            if ~isempty(cp_mat)
                finger_active(f) = 0; % Lock this finger!
                fingers_contact(f) = 1;
                % Print a message to the command window
                finger_names = {'Thumb', 'Index', 'Middle', 'Ring', 'Little'};
                disp([finger_names{f}, ' finger made contact.']);
            end
        end
    end
    
    % --- PLOTTING ---
    cla;
    SGplotHand(hannes_hand); hold on;
    SGplotSolid(object);
    axis equal; grid on;
    axis([-50 150 -100 100 -80 80]);
    view (az, el);
    title(sprintf('Differential Grasping (Step %d)', i));
    
    drawnow;
    
    % Stop the whole simulation if ALL fingers are blocked
    if sum(finger_active) == 0
        disp('All fingers contacted the object! Grasp secure.');
        break;
    end
end

%% 5. GRASP QUALITY ANALYSIS
disp(' ');
disp('--- Starting Grasp Analysis ---');
contacting_fingers = find(fingers_contact)'; % Returns an array like [3, 4, 5]

% Step A: Define the Contact Points
% The loop stopped the fingers on the object. Now we formally tell the 
% mathematical model to place contact points at the tips of all 5 fingers.
hannes_hand = SGaddFtipContact(hannes_hand, 2, contacting_fingers); % 2: soft finger (2D or 3D)
%third input "1:5" is the number of number fingers to work on

% Step A.2: Build the "Mathematical Object"
% This extracts the contact points from the hand to build the object struct
grasped_obj = SGmakeObject(hannes_hand);

% Step B: Calculate Grasp Matrix (G) and Jacobian (J)
% G: Maps fingertip forces to object motion
% J: Maps joint speeds to fingertip speeds
G = SGgraspMatrix(grasped_obj);
J = SGjacobianMatrix(hannes_hand);

% Step C: Calculate Quality Metrics
MEV  = SGmanipEllipsoidVolume(G, J);
MSVG = SGminSVG(G);
DTSC = SGdistSingularConfiguration(G, J);
GII  = SGgraspIsotropyIndex(G);

% Print Results neatly to the Command Window
disp('====================================');
disp('   GRASP QUALITY METRICS Sphere     ');
disp('====================================');
fprintf('MEV  (Dexterity Volume) : %.4f\n', MEV);
fprintf('MSVG (Weakest Direction): %.4f\n', MSVG);
fprintf('DTSC (Distance to Sing.): %.4f\n', DTSC);
fprintf('GII  (Force Isotropy)   : %.4f\n', GII);
disp('====================================');

%% --- HELPER FUNCTIONS ---
function T = URDF_T(xyz, rpy)
    Rx = SGrotx(rpy(1)); Ry = SGroty(rpy(2)); Rz = SGrotz(rpy(3));
    T = eye(4); T(1:3, 1:3) = Rz * Ry * Rx; T(1:3, 4) = xyz';
end

function T = T_rotz(angle)
    T = eye(4); T(1:3, 1:3) = SGrotz(angle);
end