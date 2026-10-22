%Prism_Master_Thesis_Runner.m
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
% Prism tests
% =============

%% 1. Dimensions and Positions 
% 4 sizes (S, M, L, XL), 3 positions each

a_length = [25, 35, 45, 55];
b_length = [20, 30, 40 , 50];
c_length  = [ 80, 90, 130, 140];
pos_names = {'Power Grasp (Centered)', 'Fingertip Grasp', 'Off-Center Grasp (Higher)'};
pos_coords = [
    59,  -2, -5;    %Power - Small
    42,  15, -5;    % Fingertip - S
    59,  -2, 30;    % Top - S
    54,   3, -5;    %P - Medium
    40, 28, -5;     %FT - M
    54, 3, 40;      %Top - M
    46, 2, -5;      %P - Large
    22, 37, 10;     %FT - L
    52, 5, 60;      %Top - L
    44, 8, 10;      %P - XL
    32, 32, 10;     %FT - XL
    57, 11, 70;     %Top - XL
];

%% 2. Prepare Data Storage
% We have 4 sizes * 3 positions = 12 total simulations.
% We will store: [(a,b,c), Position_Name, MEV, MSVG, DTSC, GII]
num_sims = length(a_length ) * length(pos_names); %4*3=12
results_data = cell(num_sims, 12);  %num_sims righe, 7 colonne + 5 forze

row = 1; % Counter for our table rows

%% 3. The Nested Automation Loop
tic; % Start a timer to see how fast MATLAB is

for r = 1:length(a_length ) %1 to 4
    current_a_length = a_length(r);
    current_b_length = b_length(r);
    current_c_length = c_length(r);

    for p = 1:3 %1:3 coordinates per radius
        current_pos_name = pos_names{p};
        current_index_coord = 3*(r-1)+p; %modifications
        current_pos_xyz  = pos_coords(current_index_coord, :)'; % Transpose to 3x1 column vector to better read coordinates
        
        % Print status to console so we know it's working
        fprintf('Simulating: Prism a=%d b=%d c=%d | Position: %s | Coordinates: [%d, %d, %d]\n', ... %... significa che continua l'istruzione
            current_a_length, current_b_length, current_c_length, current_pos_name, current_pos_xyz(1), current_pos_xyz(2), current_pos_xyz(3)); %modifications
        
        % --- RUN THE SIMULATION ---
        % shape='prism', obj_dim=[a,b, c], enable_plot=false (0)
        [MEV, MSVG, DTSC, GII, Forces] = simulate_hannes_grasp('prism', [current_a_length, current_b_length, current_c_length], current_pos_xyz, S_nominal, 0, 1);
        current_size = [current_a_length, current_b_length, current_c_length];

        % Store the results in our cell array
        results_data{row, 1} = (current_a_length);
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
Headers = {'Prism_side_a_mm', 'Grasp_Position', 'Coordinates','MEV', 'MSVG', 'DTSC', 'GII'...
           'Force_Thumb', 'Force_Index', 'Force_Middle', 'Force_Ring', 'Force_Little'};
ResultsTable = cell2table(results_data, 'VariableNames', Headers);

% Display the table in the Command Window
disp(' ');
disp('--- SIMULATION COMPLETE ---');
fprintf('Calculated %d grasps in %.2f seconds.\n', num_sims, time_taken);
disp(ResultsTable);

%Export to Excel! (Keep the file closed, else MATLAB can't edit it when you run the script a second time)
filename = 'Prism_Grasp_Results.xlsx';
writetable(ResultsTable, filename);
disp(['Results successfully saved to: ', filename]);

%% --- DATA VISUALIZATION ---
disp('Generating Thesis Graphs...');

% Extract the unique radii and position names
% (Since we know the order is perfectly sequential: 3 positions per radius)
radii_unique = unique(ResultsTable.Prism_side_a_mm);
num_r = length(radii_unique);

% Reshape the data columns into 4x3 matrices (Rows = Radii, Columns = Positions)
% The apostrophe ' transposes it so rows correspond to radii
MEV_mat  = reshape(ResultsTable.MEV, 3, num_r)';
DTSC_mat = reshape(ResultsTable.DTSC, 3, num_r)';
GII_mat  = reshape(ResultsTable.GII, 3, num_r)';

% Define colors and markers for the 3 positions to make it look professional
colors = {'#0072BD', '#D95319', '#EDB120'}; % Blue, Orange, Yellow
markers = {'o', 's', '^'}; % Circle, Square, Triangle
pos_labels = {'Power Grasp (Centered)', 'Fingertip Grasp', 'Off-Center Grasp'};

% Create a large, wide figure
figure('Name', 'Grasp Quality Metrics', 'Color', 'w', 'Position', [100, 100, 1200, 400]);

% --- Subplot 1: MEV ---
subplot(1, 3, 1);
hold on; grid on;
for p = 1:3
    plot(radii_unique, MEV_mat(:, p), '-k', 'LineWidth', 2, ...
        'Color', colors{p}, 'Marker', markers{p}, 'MarkerSize', 8, 'MarkerFaceColor', colors{p});
end
title('Dexterity (MEV)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Prism side a (mm)', 'FontSize', 11);
ylabel('Ellipsoid Volume', 'FontSize', 11);
xticks(radii_unique);

% --- Subplot 2: DTSC ---
subplot(1, 3, 2);
hold on; grid on;
for p = 1:3
    plot(radii_unique, DTSC_mat(:, p), '-k', 'LineWidth', 2, ...
        'Color', colors{p}, 'Marker', markers{p}, 'MarkerSize', 8, 'MarkerFaceColor', colors{p});
end
title('Distance to Singularity (DTSC)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Prism side a (mm)', 'FontSize', 11);
xticks(radii_unique);

% --- Subplot 3: GII ---
subplot(1, 3, 3);
hold on; grid on;
for p = 1:3
    plot(radii_unique, GII_mat(:, p), '-k', 'LineWidth', 2, ...
        'Color', colors{p}, 'Marker', markers{p}, 'MarkerSize', 8, 'MarkerFaceColor', colors{p});
end
title('Force Isotropy (GII)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Prism side a (mm)', 'FontSize', 11);
xticks(radii_unique);

% Add a single master legend at the bottom
legend(pos_labels, 'Location', 'southoutside', 'Orientation', 'horizontal', ...
       'FontSize', 11, 'Box', 'off');

% Adjust layout so the legend fits nicely
set(gcf, 'Position', [100, 100, 1200, 450]);