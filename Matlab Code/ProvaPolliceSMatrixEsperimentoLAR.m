% ProvaPolliceSMatrixEsperimentoLAR.m
clc; clear; close all;

disp('--- Starting Thumb Synergy Parameter Sweep ---');

% 1. Define the 5 S-Matrices Numerically (Varying only the Thumb)
S_nominal1 = [-1.0; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];
S_nominal2 = [-0.8; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9]; % Slower thumb
S_nominal3 = [-0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9]; % Faster thumb
S_nominal4 = [-1.1; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9]; % Very fast thumb
S_nominal5 = [-1.2; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9]; % Extremely fast thumb

% 2. Store them in a Cell Array 
S_matrices = {S_nominal1, S_nominal2, S_nominal3, S_nominal4, S_nominal5};
S_names    = {'Thumb -1.0', 'Thumb -0.8', 'Thumb -0.9', 'Thumb -1.1', 'Thumb -1.2'};

% =============
% Constant Parameters
% =============
current_radius = 25;
current_height = 130;
% Need to transpose coords to a 3x1 column vector for the function
current_pos_xyz = [45,  8,  7]'; % Power3 - Small

%% 3. Prepare Data Storage
num_sims = length(S_matrices); 
results_data = cell(num_sims, 14); % 7 base + 5 forces + 2 ratios
row = 1; 

%% 4. The Automation Loop
tic; 
for r = 1:num_sims
    current_S_matrix = S_matrices{r}; % Get the actual numbers
    current_S_name   = S_names{r};    % Get the text name
    
    fprintf('Simulating: Matrix = %s\n', current_S_name);

    % --- RUN THE SIMULATION ---
    % shape='cylinder', obj_dim=[radius, res, height], enable_plot=false,
    % enable differential = true
    [MEV, MSVG, DTSC, GII, Forces] = simulate_hannes_grasp('cylinder', [current_radius, 35, current_height], current_pos_xyz, current_S_matrix, 0, 1);
    
    % Store the results
    results_data{row, 1} = current_radius;
    results_data{row, 2} = current_S_name;
    results_data{row, 3} = mat2str(current_pos_xyz'); 
    results_data{row, 4} = MEV;
    results_data{row, 5} = MSVG;
    results_data{row, 6} = DTSC;
    results_data{row, 7} = GII;
    
    % --- ADD THE FORCE DATA ---
    results_data{row, 8}  = Forces(1); % Thumb
    results_data{row, 9}  = Forces(2); % Index
    results_data{row, 10} = Forces(3); % Middle
    results_data{row, 11} = Forces(4); % Ring
    results_data{row, 12} = Forces(5); % Little

    row = row + 1; 
end

time_taken = toc; 

%% 4.5 Calculate Ratios
% Loop through the table we just built and calculate Index/Thumb and Middle/Thumb
for r = 1:num_sims
    thumb_force  = results_data{r, 8};
    index_force  = results_data{r, 9};
    middle_force = results_data{r, 10};
    
    % Prevent division by zero if grasp failed
    if thumb_force > 0
        results_data{r, 13} = index_force / thumb_force;
        results_data{r, 14} = middle_force / thumb_force;
    else
        results_data{r, 13} = 0;
        results_data{r, 14} = 0;
    end
end

%% 5. Convert to Table and Export
% Added the two ratio columns at the end!
Headers = {'Cyl_Radius_mm', 'Synergy_Matrix', 'Coordinates', 'MEV', 'MSVG', 'DTSC', 'GII',...
            'Force_Thumb', 'Force_Index', 'Force_Middle', 'Force_Ring', 'Force_Little', 'Ratio_Idx_Thumb', 'Ratio_Mid_Thumb'};
ResultsTable = cell2table(results_data, 'VariableNames', Headers);

disp(' ');
disp('--- SIMULATION COMPLETE ---');
fprintf('Calculated %d grasps in %.2f seconds.\n', num_sims, time_taken);
disp(ResultsTable);

filename = 'ProvaPolliceSMatrixEsperimentoLAR.xlsx';
writetable(ResultsTable, filename);
disp(['Results successfully saved to: ', filename]);


%% --- DATA VISUALIZATION (BAR CHARTS) ---
disp('Generating Thesis Graphs...');

MEV_data  = cell2mat(results_data(:, 4));
DTSC_data = cell2mat(results_data(:, 6));
GII_data  = cell2mat(results_data(:, 7));
Force_data = cell2mat(results_data(:, 8:12)); 

X_cats = categorical(S_names);
X_cats = reordercats(X_cats, S_names);

fig = figure('Name', 'Sensitivity Analysis: Thumb Speed', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% --- Subplot 1: MEV ---
subplot(2, 2, 1);
b1 = bar(X_cats, MEV_data);
b1.FaceColor = '#0072BD'; 
title('Dexterity (MEV)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Ellipsoid Volume'); grid on;

% --- Subplot 2: DTSC ---
subplot(2, 2, 2);
b2 = bar(X_cats, DTSC_data);
b2.FaceColor = '#D95319'; 
title('Distance to Singularity (DTSC)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% --- Subplot 3: GII ---
subplot(2, 2, 3);
b3 = bar(X_cats, GII_data);
b3.FaceColor = '#EDB120'; 
title('Force Isotropy (GII)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% --- Subplot 4: CONTACT FORCES ---
subplot(2, 2, 4);
b4 = bar(X_cats, Force_data, 'grouped'); 
title('Contact Force Distribution', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Relative Force (Units)'); grid on;
legend({'Thumb', 'Index', 'Middle', 'Ring', 'Little'}, 'Location', 'northeast', 'FontSize', 9);

set(gcf, 'Position', [100, 100, 1200, 700]);

% Export the figure automatically
%exportgraphics(fig, 'Thumb_Sensitivity_Analysis.pdf', 'ContentType', 'vector');
%disp('Figure saved as Thumb_Sensitivity_Analysis.pdf');