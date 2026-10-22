% Cylinder_Master_Thesis_Runner.m

clc; clear; close all;

disp('--- Starting Automated Data Collection ---');

% Nominal Hannes S-Matrix
S_nominal = [-1.0; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];

% =============
% Cylinder Custom Position Tests
% =============

%% 1. Dimensions and Positions 
% We are testing 1 Cylinder size across 6 custom positions
cylinder_radius = 25;
cylinder_height = 130;

pos_names = {
    'Power Grasp 1', 
    'Power Grasp 2', 
    'Power Grasp 3', 
    'Fingertip 1', 
    'Fingertip 2', 
    'Top Grasp'
};

pos_coords = [
    50, 11,  7;  % Power - Small
    55, 15,  7;  % Power2 - Small
    45,  8,  7;  % Power3 - Small
    42, 24,  7;  % Fingertip1 - S
    45, 29,  7;  % Fingertip2 - S
    50, 11, 55   % Top - S
];

num_sims = length(pos_names);

%% 2. Prepare Data Storage
% Columns: [Radius, Position, Coords, MEV, MSVG, DTSC, GII, F_Thumb, F_Index, F_Mid, F_Ring, F_Lit]
results_data = cell(num_sims, 12);  
row = 1; 

%% 3. The Automation Loop
tic; % Start timer

for p = 1:num_sims
    current_pos_name = pos_names{p};
    current_pos_xyz  = pos_coords(p, :)'; % Transpose to 3x1 column vector
    
    % Print status
    fprintf('Simulating: %s | Coordinates: [%d, %d, %d]\n', ... 
        current_pos_name, current_pos_xyz(1), current_pos_xyz(2), current_pos_xyz(3));
    
    % --- RUN THE SIMULATION ---
    % shape='cylinder', obj_dim=[radius, 0, height], enable_plot=0
    [MEV, MSVG, DTSC, GII, Forces] = simulate_hannes_grasp('cylinder', [cylinder_radius, 0, cylinder_height], current_pos_xyz, S_nominal, 1, 1);
    
    % Store the results
    results_data{row, 1} = cylinder_radius;
    results_data{row, 2} = current_pos_name;
    results_data{row, 3} = mat2str(current_pos_xyz'); 
    results_data{row, 4} = MEV;
    results_data{row, 5} = MSVG;
    results_data{row, 6} = DTSC;
    results_data{row, 7} = GII;
    results_data{row, 8}  = Forces(1); % Thumb
    results_data{row, 9}  = Forces(2); % Index
    results_data{row, 10} = Forces(3); % Middle
    results_data{row, 11} = Forces(4); % Ring
    results_data{row, 12} = Forces(5); % Little
    
    row = row + 1;
end

time_taken = toc; 

%% 4. Convert to Table and Export
% Calculate the Force Ratios (Thumb is Col 8, Index is Col 9, Middle is Col 10)
for r = 1:num_sims
    thumb_force  = results_data{r, 8};
    index_force  = results_data{r, 9};
    middle_force = results_data{r, 10};
    
    % Prevent division by zero if thumb force is 0 (failed grasp)
    if thumb_force > 0
        results_data{r, 13} = index_force / thumb_force;   % Index/Thumb Ratio
        results_data{r, 14} = middle_force / thumb_force;  % Middle/Thumb Ratio
    else
        results_data{r, 13} = 0;
        results_data{r, 14} = 0;
    end
end

% Create the Headers for the new 14-column table
Headers = {'Cylinder_Radius_mm', 'Grasp_Position', 'Coordinates', 'MEV', 'MSVG', 'DTSC', 'GII', ...
           'Force_Thumb', 'Force_Index', 'Force_Middle', 'Force_Ring', 'Force_Little', ...
           'Ratio_Idx_Thumb', 'Ratio_Mid_Thumb'};
           
ResultsTable = cell2table(results_data, 'VariableNames', Headers);

disp(' ');
disp('--- SIMULATION COMPLETE ---');
fprintf('Calculated %d grasps in %.2f seconds.\n', num_sims, time_taken);
disp(ResultsTable);

% Export to Excel with the requested filename
filename = 'LAR_Cylinder_Experiment.xlsx';
writetable(ResultsTable, filename);
disp(['Results successfully saved to: ', filename]);

%% --- DATA VISUALIZATION (BAR CHARTS WITH FORCES) ---
disp('Generating Thesis Graphs...');

% Extract data for plotting
MEV_data  = cell2mat(results_data(:, 4));
DTSC_data = cell2mat(results_data(:, 6));
GII_data  = cell2mat(results_data(:, 7));
Force_data = cell2mat(results_data(:, 8:12)); % 5xN matrix for grouped bars

% Create categorical X-axis based on grasp names
X_cats = categorical(pos_names);
X_cats = reordercats(X_cats, pos_names); 

% Create a larger figure to accommodate 4 subplots
fig = figure('Name', 'Grasp Quality and Forces: Cylinder Positions', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% --- Subplot 1: MEV ---
subplot(2, 2, 1);
b1 = bar(X_cats, MEV_data);
b1.FaceColor = '#0072BD'; 
title('Dexterity (MEV)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Ellipsoid Volume', 'FontSize', 11);
grid on; xtickangle(30); 

% --- Subplot 2: DTSC ---
subplot(2, 2, 2);
b2 = bar(X_cats, DTSC_data);
b2.FaceColor = '#D95319'; 
title('Distance to Singularity (DTSC)', 'FontSize', 12, 'FontWeight', 'bold');
grid on; xtickangle(30);

% --- Subplot 3: GII ---
subplot(2, 2, 3);
b3 = bar(X_cats, GII_data);
b3.FaceColor = '#EDB120'; 
title('Force Isotropy (GII)', 'FontSize', 12, 'FontWeight', 'bold');
grid on; xtickangle(30);

% --- Subplot 4: CONTACT FORCES ---
subplot(2, 2, 4);
b4 = bar(X_cats, Force_data, 'grouped'); 
title('Contact Force Distribution', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Relative Force (Units)'); grid on; xtickangle(30);
legend({'Thumb', 'Index', 'Middle', 'Ring', 'Little'}, 'Location', 'northeast', 'FontSize', 9);

% Adjust layout
set(gcf, 'Position', [100, 100, 1200, 750]);

% Optional: Auto-export the figure for your thesis
%exportgraphics(fig, 'LAR_Cylinder_Metrics_and_Forces.pdf', 'ContentType', 'vector');

%end