%Sphere_Master_Thesis_Runner.m
%Master wrt slave, yet bachelor thesis

%function [MEV, MSVG, DTSC, GII] = simulate_hannes_grasp(shape_type, obj_dim, pos_xyz, S_matrix, enable_plot, use_differential)
%Prism: obj_dim [a ,b ,c]
%Cylinder: obj_dim [obj_radius , (res=35) , height]
%Sphere:obj_dim [obj_radius, (res=35) , //]

%S_Matrix general S_Matrix = [thumb; I_MCP; I_PIP; M_MCP; M_PIP; R_MCP; R_PIP; L_MCP; L_PIP];
%MCP is joint from palm to finger base
%PIP is joint from finger base to fingertip

%In a human hand, MCP_speed>PIP_speed, cfr. page 4 Hannes Hand paper

clc; clear; close all;

disp('--- Starting Automated Data Collection ---');

% Nominal Hannes S-Matrix, MCP = 2PIP
S_nominal = [-1.0; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];

%Uniform S-Matrix
S_uniform = [-0.8; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0];

%UnbalncedIM (index and middle curling much quicker than ring and little)
% -33% performance on ring and little wrt index and middle
S_unbalanceIM = [-0.8; 1.5; 1.0; 1.5; 1.0; 1.0; 0.7; 1.0; 0.7];

%UnbalncedRL (ring and little curling much quicker than middle and index)
% -33% performance on ring and little wrt index and middle
S_unbalanceRL = [-0.8; 1.0; 0.7; 1.0; 0.7; 1.5; 1.0; 1.5; 1.0];

% =============
% Sphere tests
% =============

%% 1. Dimensions and Positions 
% 4 radii, 3 positions each (S, M, L, XL)

radii = [30, 40, 50, 60];
pos_names = {'Power Grasp (Centered)', 'Fingertip Grasp', 'Off-Center Grasp (Higher)'};
pos_coords = [
    50,  15, 27;     %Power - Small
    42,  27, 20;    % Fingertip - S
    42,  27, 35;    % Top - S
    52, 20, -3;     %P - Medium
    32, 27, 3;      %FT - M
    42, 10, 45;     %Top - M
    45, 27, -5;     %P - Large
    28, 34, -5;     %FT - L
    45, 27, 60;     %Top - L
    35, 37, -5;     %P - XL
    25, 40, 4;      %FT - XL
    40, 32, 65;     %Top - XL
];

%% 2. Prepare Data Storage
% We have 4 sizes * 3 positions = 12 total simulations.
% We will store: [Radius, Position_Name, MEV, MSVG, DTSC, GII]
num_sims = length(radii) * length(pos_names); %4*3=12
results_data = cell(num_sims, 12);  %num_sims righe, 7 + 5 (forze) colonne
row = 1; % Counter for our table rows

%% 3. The Nested Automation Loop
tic; % Start a timer to see how fast MATLAB is

for r = 1:length(radii) %1 to 4
    current_radius = radii(r);
    
    for p = 1:3 %1:3 coordinates per radius
        current_pos_name = pos_names{p};
        current_index_coord = 3*(r-1)+p;
        current_pos_xyz  = pos_coords(current_index_coord, :)'; % Transpose to 3x1 column vector to better read coordinates
        
        % Print status to console so we know it's working
        fprintf('Simulating: Sphere R=%d | Position: %s | Coordinates: [%d, %d, %d]\n', ... %... significa che continua l'istruzione
            current_radius, current_pos_name, current_pos_xyz(1), current_pos_xyz(2), current_pos_xyz(3)); %modifications
        
        % --- RUN THE SIMULATION ---
        % shape='sphere', obj_dim=[radius, 30_res, 0], enable_plot=false (0)
        [MEV, MSVG, DTSC, GII, Forces] = simulate_hannes_grasp('sphere', [current_radius, 0, 0], current_pos_xyz, S_nominal, 0, 1);
        
        % Store the results in our cell array
        results_data{row, 1} = current_radius;
        results_data{row, 2} = current_pos_name;
        results_data{row, 3} = mat2str(current_pos_xyz'); %apex to transpose back to 1x3 vector (single row)
        results_data{row, 4} = MEV;
        results_data{row, 5} = MSVG;
        results_data{row, 6} = DTSC;
        results_data{row, 7} = GII;

        results_data{row, 8}  = Forces(1); % Thumb
        results_data{row, 9}  = Forces(2); % Index
        results_data{row, 10} = Forces(3); % Middle
        results_data{row, 11} = Forces(4); % Ring
        results_data{row, 12} = Forces(5); % Little
        
        row = row + 1; % Move to next row
    end
end

time_taken = toc; % Stop timer

%% 4. Convert to Table and Export
% Convert the raw data into a nice MATLAB table with headers
Headers = {'Sphere_Radius_mm', 'Grasp_Position', 'Coordinates','MEV', 'MSVG', 'DTSC', 'GII'...
           'Force_Thumb', 'Force_Index', 'Force_Middle', 'Force_Ring', 'Force_Little'};
ResultsTable = cell2table(results_data, 'VariableNames', Headers);

% Display the table in the Command Window
disp(' ');
disp('--- SIMULATION COMPLETE ---');
fprintf('Calculated %d grasps in %.2f seconds.\n', num_sims, time_taken);
disp(ResultsTable);

%______________________________________________________________________________

%% --- DATA VISUALIZATION (POWERPOINT MODE - LINE CHARTS) ---
disp('Generating Presentation Line Graphs...');

% Extract the unique radii and position names
% (Since we know the order is perfectly sequential: 3 positions per radius)
radii_unique = unique(ResultsTable.Sphere_Radius_mm);
num_r = length(radii_unique);

% Reshape the data columns into 4x3 matrices (Rows = Radii, Columns = Positions)
% The apostrophe ' transposes it so rows correspond to radii
MEV_mat  = reshape(ResultsTable.MEV, 3, num_r)';
DTSC_mat = reshape(ResultsTable.DTSC, 3, num_r)';
GII_mat  = reshape(ResultsTable.GII, 3, num_r)';

% Define colors and markers
colors = {'#0072BD', '#D95319', '#EDB120'}; % Blue, Orange, Yellow
markers = {'o', 's', '^'}; % Circle, Square, Triangle
pos_labels = {'Power Grasp (Centered)', 'Fingertip Grasp', 'Off-Center Grasp'};

% Create a large, wide figure
fig = figure('Name', 'Grasp Quality Metrics', 'Color', 'w', 'Position', [100, 100, 1400, 600]);

% --- Subplot 1: MEV ---
subplot(1, 3, 1);
set(gca, 'FontSize', 16); % Scales the axis numbers
hold on; grid on;
for p = 1:3
    % Increased LineWidth to 3 and MarkerSize to 12!
    plot(radii_unique, MEV_mat(:, p), '-k', 'LineWidth', 3, ...
        'Color', colors{p}, 'Marker', markers{p}, 'MarkerSize', 12, 'MarkerFaceColor', colors{p});
end
title('MEV', 'FontSize', 24, 'FontWeight', 'bold'); % Short title, Huge font
ylabel('Metric', 'FontSize', 16, 'FontWeight', 'bold'); % Single word, bold
xlabel('Radius (mm)', 'FontSize', 16, 'FontWeight', 'bold'); % Make sure to change 'Radius' to 'Radius' or 'Prism side a'

xticks(radii_unique);

% --- Subplot 2: DTSC ---
subplot(1, 3, 2);
set(gca, 'FontSize', 16);
hold on; grid on;
for p = 1:3
    plot(radii_unique, DTSC_mat(:, p), '-k', 'LineWidth', 3, ...
        'Color', colors{p}, 'Marker', markers{p}, 'MarkerSize', 12, 'MarkerFaceColor', colors{p});
end
title('DTSC', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Radius (mm)', 'FontSize', 16, 'FontWeight', 'bold');
xticks(radii_unique);

% --- Subplot 3: GII ---
subplot(1, 3, 3);
set(gca, 'FontSize', 16);
hold on; grid on;
for p = 1:3
    plot(radii_unique, GII_mat(:, p), '-k', 'LineWidth', 3, ...
        'Color', colors{p}, 'Marker', markers{p}, 'MarkerSize', 12, 'MarkerFaceColor', colors{p});
end
title('GII', 'FontSize', 24, 'FontWeight', 'bold');
xlabel('Radius (mm)', 'FontSize', 16, 'FontWeight', 'bold')
xticks(radii_unique);

% Auto-Export for PowerPoint (PNG with transparent background is great for slides)
exportgraphics(fig, 'LineChart_Presentation.png', 'Resolution', 600, 'BackgroundColor', 'none');
disp('Figure saved as LineChart_Presentation.png');