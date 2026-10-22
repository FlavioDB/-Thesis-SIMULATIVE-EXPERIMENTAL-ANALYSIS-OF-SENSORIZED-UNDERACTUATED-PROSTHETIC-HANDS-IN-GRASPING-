%% simulate_hannes_grasp.m
% Function for Thesis on Hannes Hand

%Flavio Davide Borriello
%May 2026
%Università di Bologna - Ingegneria dell'automazione

%inputs: shape_type string ('prism', 'cylinder' or 'sphere')
%obj_dim 1x3 vector, the meaning of the entries change based on object type
%pos_xyz 1x3 vector, position of the object in 3D space
%S_matrix 9x1 vector, synergy matrix of the Hannes Hand
%eneble plot bit, if 1 plot is enabled, else no plot will be drawn
%use_differential bit, 1=true

function [MEV, MSVG, DTSC, GII, Contact_Forces] = simulate_hannes_grasp(shape_type, obj_dim, pos_xyz, S_matrix, enable_plot, use_differential)

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
F_array = {F_thumb, F_index, F_middle, F_ring, F_little}; %dx con angolo sopra il mignolo
hannes_hand = SGmakeHand(F_array);

%% 2. DEFINE THE HANNES S-MATRIX (Differential Transmission)
%S-Matrix will be fed along with the inputs
S_hannes = S_matrix;

%% 3. CREATE THE OBJECT (if statement)
%obj_dim [1x3] depending on shape_type
%pos_xyz is location in 3D space
H_obj = eye(4);
H_obj(1:3, 4) = pos_xyz;

switch shape_type
    case 'prism'
        %obj_dim [a ,b ,c]
        R_z = SGrotz(0.19*pi);
        H_obj(1:3,1:3) = R_z; %rotate cube by about 30°
        object = SGcube(H_obj, obj_dim(1), obj_dim(2), obj_dim(3));
    
    case 'cylinder'
        
        %obj_dim [obj_radius , // , height] (height is last even if its the
        %first input for SGcylinder so that radius and res are in the same
        %position for both cylinder and sphere cases
        object = SGcylinder(H_obj, obj_dim(3), obj_dim(1), 35); %35 is the resolution

    case 'sphere'
   
        %obj_dim [obj_radius, // ,//]
        object = SGsphere(H_obj, obj_dim(1), 35);

    otherwise
        
        error('Shape type not recognized! Use prism, cylinder or sphere');
end

%% 4. SIMULATE THE DIFFERENTIAL GRASP

fingers_contact = zeros(5, 1); %used for debugging to count how many fingers touch and how many don't
fingers_limit_reached = 0;

%1/2 graphics code that are disabled for majority of simulation %check!
% to save computational power and time
if enable_plot
    az = 155; el = 20;
    figure('Name', sprintf('Hannes Grasp: %s', shape_type), 'Color', 'w');
    axis equal; grid on; view(az, el);
    axis([-50 150 -100 100 -80 80]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view (az, el);
    disp('Starting Adaptive Differential Closure...');
end

% Simulation variables
% step_size = 0.05; % How much 'z' increases per frame
% max_steps = 50; %increase to 50 to ensure all objects will be grasped
step_size = 0.05; % How much 'z' increases per frame
max_steps = 200; %increase to 50 to ensure all objects will be grasped
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
                j_idx = [ (f-1)*2, (f-1)*2 + 1 ]; %matematica PAZZESCA per individuare le entrate di S-matrix giuste per il dito in questione
            end
            
            % Add the movement for this finger's joints
            delta_q(j_idx) = S_hannes(j_idx) * step_size;
        end
    end
    
    % Update joint angles
    q_current = q_current + delta_q;
    hannes_hand = SGmoveHand(hannes_hand, q_current);

     % --- MECHANICAL JOINT LIMITS (Stop crossing into the palm) ---
    max_angle = 2.7; % Approx 150 degrees in radians
    
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
                fingers_limit_reached = fingers_limit_reached + 1;
                if enable_plot
                    finger_names = {'Thumb', 'Index', 'Middle', 'Ring', 'Little'};
                    disp([finger_names{f}, ' finger reached mechanical limit (stopped).']);
                end
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
                fingers_contact(f) = 1; %finger f made contact

           if enable_plot %check!    
                % Print a message to the command window
                finger_names = {'Thumb', 'Index', 'Middle', 'Ring', 'Little'};
                disp([finger_names{f}, ' finger made contact.']);
           end
            end
        end
    end
    
        %2/2 graphics code that are disabled for majority of simulation
        if enable_plot
            % --- PLOTTING ---
            cla;
            SGplotHand(hannes_hand); hold on;
            SGplotSolid(object);
            axis equal; grid on; view(180, 15);
            axis([-50 150 -100 100 -80 80]);
            view (az, el);
            title(sprintf('Differential Grasping (Step %d)', i));
    
            drawnow;
        end

    total_finger_contact = sum(fingers_contact);

% --- THE DIFFERENTIAL VS. RIGID LOGIC ---
    if use_differential == 1
        % HANNES MODE: Wait until ALL fingers are blocked (Sum == 0)
        if sum(finger_active) == 0
            fprintf('Grasp completed: %d fingers made contact | %d fingers reached joint limit\n', total_finger_contact, fingers_limit_reached);
            break;
        end
    else
        % RIGID MODE: Stop the entire hand if ANY finger touches (Sum < 5)
        if sum(finger_active) < 5
            fprintf('Grasp completed: %d fingers made contact | %d fingers reached joint limit\n', total_finger_contact, fingers_limit_reached);
            break;
        end
    end
end

%% 5. GRASP QUALITY ANALYSIS
if enable_plot, disp('--- Starting Grasp Analysis ---'); end

% A. Find out which fingers ACTUALLY touched the object
% The loop stopped the fingers on the object. Now we formally tell the 
% mathematical model to place contact points at the tips of contact fingers.
contacting_fingers = find(fingers_contact)'; % Returns an array like [3, 4, 5]
%find(X) returns a vector containing the linear indices of each nonzero element in array X

% Failsafe: If no fingers touched (shouldn't happen, but just in case)
if isempty(contacting_fingers)
    MEV = 0; MSVG = 0; DTSC = 0; GII = 0; Contact_Forces = zeros(5,1);
    return;
end

% B. Define Contact Points ONLY for the fingers that touched
hannes_hand = SGaddFtipContact(hannes_hand, 1, contacting_fingers); 
%2° input: 1= hard finger, 2 = soft finger
%3°input: fingers to apply it on (usually all 5)

% This extracts the contact points from the hand to build the object struct
grasped_obj = SGmakeObject(hannes_hand);

% Calculate Grasp Matrix (G) and Jacobian (J)
% G: Maps fingertip forces to object motion
% J: Maps joint speeds to fingertip speeds
G = SGgraspMatrix(grasped_obj);
J = SGjacobianMatrix(hannes_hand);

% C. Calculate Quality Metrics
% SANITY CHECK: A stable 3D grasp requires at least 3 contact points.
% If fewer than 3 fingers touched, the grasp is mathematically unstable 
% (the object would fall), so all quality metrics must be 0.
% We use real() to eliminate floating-point complex artifacts caused by 
% taking the square root of ill-conditioned, near-singular matrices.

if length(contacting_fingers) >= 3
    try MEV  = real(SGmanipEllipsoidVolume(G, J)); catch, MEV = 0; end
    try MSVG = real(SGminSVG(G)); catch, MSVG = 0; end
    try DTSC = real(SGdistSingularConfiguration(G, J)); catch, DTSC = 0; end
    try GII  = real(SGgraspIsotropyIndex(G)); catch, GII = 0; end
else
    MEV = 0; MSVG = 0; DTSC = 0; GII = 0; 
    if enable_plot, disp('WARNING: Less than 3 contacts. Grasp unstable!'); end
end

% Step D: CALCULATE CONTACT FORCES (STATIC EQUILIBRIUM)
% tau = Jtranspose * lambda
%lambda = J_pseudoinverse * tau
%lambda = contact forces, tau = torques at the joints (known with matrix S)

% 1. Assume the main motor pulls with 1 Unit of Tension
motor_tension = 1.0; 
% 2. The joint torques are proportional to our S-Matrix (pulley radii)
tau_joints = S_hannes * motor_tension; 
% 3. Calculate forces using the Pseudoinverse of the Jacobian Transpose
% Formula: lambda = pinv(J') * tau
%only calculate for the active contact points
lambda = pinv(J') * tau_joints;
% lambda is a vertical list of forces [Fx1; Fy1; Fz1; Fx2; Fy2; Fz2; ...]
% Let's calculate the overall magnitude (norm) of the force for each finger
Contact_Forces = zeros(5, 1);
% Carefully map the calculated forces back to the correct fingers
for idx = 1:length(contacting_fingers)
    f = contacting_fingers(idx); % The actual finger ID (1 to 5)
    % Each hard contact provides 3 force components (X, Y, Z)
    force_idx = (idx-1)*3 + 1 : (idx-1)*3 + 3;
    finger_force_vector = lambda(force_idx);
    % Save the magnitude of the force for this specific finger
    Contact_Forces(f) = norm(finger_force_vector);
end

if enable_plot %check! 
    % Print Results neatly to the Command Window
    disp('====================================');
    fprintf ('   GRASP QUALITY METRICS & Forces (%s)\n', shape_type);
    disp('====================================');
    fprintf('MEV  (Dexterity Volume) : %.4f\n', MEV);
    fprintf('MSVG (Weakest Direction): %.4f\n', MSVG);
    fprintf('DTSC (Distance to Sing.): %.4f\n', DTSC);
    fprintf('GII  (Force Isotropy)   : %.4f\n', GII);
    disp('--- Fingertip Contact Forces (Relative) ---');
    finger_names = {'Thumb', 'Index', 'Middle', 'Ring', 'Little'};
    for f = 1:5
        fprintf('%s : %.4f Units\n', finger_names{f}, Contact_Forces(f));
    end
    disp('====================================');
end

%Ho appliacto il principio dei lavori virtuali e ho calcolato l'equilibrio
%statico da tau = J_transpose*lambda con la moore-penrose inverse. 



%% --- HELPER FUNCTIONS ---
function T = URDF_T(xyz, rpy)
    Rx = SGrotx(rpy(1)); Ry = SGroty(rpy(2)); Rz = SGrotz(rpy(3));
    T = eye(4); T(1:3, 1:3) = Rz * Ry * Rx; T(1:3, 4) = xyz';
end

function T = T_rotz(angle)
    T = eye(4); T(1:3, 1:3) = SGrotz(angle);
end

end %end function