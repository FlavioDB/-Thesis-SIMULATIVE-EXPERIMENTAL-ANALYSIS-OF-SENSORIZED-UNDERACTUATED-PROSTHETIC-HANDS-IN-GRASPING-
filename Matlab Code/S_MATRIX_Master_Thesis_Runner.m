clc; clear; close all;

disp('--- Starting S-Matrix Sensitivity Analysis ---');

% 1. Define the 4 S-Matrices Numerically
S_nominal     = [-1.0; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];
S_uniform     = [-1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0];
S_unbalanceIM = [-1.0; 1.8; 0.9; 1.8; 0.9; 1.0; 0.5; 1.0; 0.5];
S_unbalanceRL = [-1.0; 1.0; 0.5; 1.0; 0.5; 1.8; 0.9; 1.8; 0.9];

% S_nominal     = [-2.0; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];
% S_uniform     = [-0.5; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];
% S_unbalanceIM = [-2.5; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];
% S_unbalanceRL = [-3.0; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9; 1.8; 0.9];
%matrici per le prove del pollice

% 2. Store them in a Cell Array so we can loop through them
% S_matrices holds the math. S_names holds the text for Excel.
S_matrices = {S_nominal, S_uniform, S_unbalanceIM, S_unbalanceRL, S_nominal};
S_names    = {'Nominal (Biomimetic)', 'Uniform', 'Unbalanced (Index/Mid)', 'Unbalanced (Ring/Lit)', 'Nominal (NO DIFF)'};

% =============
% Constant Parameters
% =============
current_radius = 30;
current_height = 150;
% Need to transpose coords to a 3x1 column vector for the function
current_pos_xyz = [49, 17, 0]'; 

%% 3. Prepare Data Storage
num_sims = length(S_matrices); %+1 = nominal again but without diff
results_data = cell(num_sims, 12); %7 old + 5 new for the forces  
row = 1; 

%% 4. The Automation Loop
tic; 
for r = 1:num_sims
    current_S_matrix = S_matrices{r}; % Get the actual numbers
    current_S_name   = S_names{r};    % Get the text name
    
    fprintf('Simulating: Matrix = %s\n', current_S_name);

     % Decide if we use the differential based on the loop index
    if r == 5
        use_diff_flag = 0; % Turn off for the 5th test
    else
        use_diff_flag = 1; % On for tests 1-4
    end     
    % --- RUN THE SIMULATION ---
    % shape='sphere', obj_dim=[radius, res_35, 0], enable_plot=0 (false) for speed
    [MEV, MSVG, DTSC, GII, Forces] = simulate_hannes_grasp('cylinder', [current_radius, 35, current_height], current_pos_xyz, current_S_matrix, 0, use_diff_flag); %line 44
    
    % Store the results
    results_data{row, 1} = current_radius;
    results_data{row, 2} = current_S_name;
    results_data{row, 3} = mat2str(current_pos_xyz'); % Transpose back to row for text
    results_data{row, 4} = MEV;
    results_data{row, 5} = MSVG;
    results_data{row, 6} = DTSC;
    results_data{row, 7} = GII;
    
    % --- ADD THE NEW FORCE DATA ---
    results_data{row, 8}  = Forces(1); % Thumb
    results_data{row, 9}  = Forces(2); % Index
    results_data{row, 10} = Forces(3); % Middle
    results_data{row, 11} = Forces(4); % Ring
    results_data{row, 12} = Forces(5); % Little

    row = row + 1; 
end

time_taken = toc; 

%% 5. Convert to Table and Export
Headers = {'Sphere_Radius_mm', 'Synergy_Matrix', 'Coordinates', 'MEV', 'MSVG', 'DTSC', 'GII',...
            'Force_Thumb', 'Force_Index', 'Force_Middle', 'Force_Ring', 'Force_Little'};
ResultsTable = cell2table(results_data, 'VariableNames', Headers);

disp(' ');
disp('--- SIMULATION COMPLETE ---');
fprintf('Calculated %d grasps in %.2f seconds.\n', num_sims, time_taken);
disp(ResultsTable);

filename = 'S_Matrix_Grasp_Results.xlsx';
writetable(ResultsTable, filename);
disp(['Results successfully saved to: ', filename]);


%% --- DATA VISUALIZATION (BAR CHARTS) ---
disp('Generating Thesis Graphs...');

% Extract data into simple arrays for plotting
MEV_data  = cell2mat(results_data(:, 4));
MSVG_data = cell2mat(results_data(:, 5));
DTSC_data = cell2mat(results_data(:, 6));
GII_data  = cell2mat(results_data(:, 7));

% Extract the Forces (Rows = S-Matrix, Columns = 5 Fingers)
Force_data = cell2mat(results_data(:, 8:12)); 

% Create a categorical X-axis
X_cats = categorical(S_names);
X_cats = reordercats(X_cats, S_names);

% Create figure (Made it slightly taller to fit the 4th subplot nicely)
fig = figure('Name', 'Sensitivity Analysis: Synergy Matrices', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% --- Subplot 1: MEV ---
subplot(2, 2, 1);
b1 = bar(X_cats, MEV_data);
b1.FaceColor = '#0072BD'; 
title('MEV', 'FontSize', 18, 'FontWeight', 'bold');
ylabel('Ellipsoid Volume'); grid on;

% --- Subplot 2: DTSC ---
subplot(2, 2, 2);
b2 = bar(X_cats, DTSC_data);
b2.FaceColor = '#D95319'; 
title('DTSC', 'FontSize', 18, 'FontWeight', 'bold');
grid on;

% --- Subplot 3: GII ---
subplot(2, 2, 3);
b3 = bar(X_cats, GII_data);
b3.FaceColor = '#EDB120'; 
title('GII', 'FontSize', 18, 'FontWeight', 'bold');
grid on;

% --- Subplot 4: CONTACT FORCES (Grouped Bar Chart) ---
subplot(2, 2, 4);
% Plotting the 4x5 Force matrix creates a grouped bar chart automatically!
b4 = bar(X_cats, Force_data, 'grouped'); 
title('Forces', 'FontSize', 18, 'FontWeight', 'bold');
ylabel('Relative Force (Units)'); grid on;

% Add a legend for the 5 fingers
legend({'Thumb', 'Index', 'Middle', 'Ring', 'Little'}, 'Location', 'northeast', 'FontSize', 9);

% Adjust overall layout
set(gcf, 'Position', [100, 100, 1200, 700]);
