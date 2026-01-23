clear all
close all
clc
run("C:\Users\edo\acados\examples\acados_matlab_octave\acados_env_variables_windows.m")
import casadi.*
addpath('tracks', 'BRT data', 'build', 'c_generated_code', 'helper_functions_3egos'); 
track_file = 'rettilineo.txt';

%% Simulation parameters and prediction horizon
N = 60;
T = 3; %prediction horizon
dt = T / N;
Tf = 25.0; 
Nsim = round(Tf / dt);

%% Vehicle initialization
lane_width = 8.0; 
num_egos = 3; 
ego = cell(1, num_egos);

% %IC
% ego{1}.x0 = [24, 2, 0, 0, 6];      %scenario 1 
% ego{2}.x0 = [22, -2, 0, 0, 6];
% ego{3}.x0 = [15, -2, 0, 0, 6]; 
% hdv_x0 = [0, 2, 0, 8];
% scen = 1;

% ego{1}.x0 = [29.5, 2, 0, 0, 6];      %scenario 4  
% ego{2}.x0 = [25, -2, 0, 0, 6];
% ego{3}.x0 = [21, 2, 0, 0, 6]; 
% hdv_x0 = [0, 2, 0, 8];
% scen = 4;

% ego{1}.x0 = [24.5, 2, 0, 0, 6];    %scenario 5  
% ego{2}.x0 = [28, -2, 0, 0, 6];
% ego{3}.x0 = [21, -2, 0, 0, 6]; 
% hdv_x0 = [0, 2, 0, 8];
% scen = 5;

% ego{1}.x0 = [21, 2, 0, 0, 6];      %scenario 6 
% ego{2}.x0 = [29, -2, 0, 0, 6];
% ego{3}.x0 = [23, -2, 0, 0, 6]; 
% hdv_x0 = [0, 2, 0, 8];
% scen = 6;

% ego{1}.x0 = [20, 2, 0, 0, 6];      %scenario 7 
% ego{2}.x0 = [27, 2, 0, 0, 6];
% ego{3}.x0 = [29, -2, 0, 0, 6]; 
% hdv_x0 = [0, 2, 0, 8];
% scen = 7;


% EGO 1
ego{1}.x_current = ego{1}.x0;
ego{1}.u_current = [0; 0];
ego{1}.V_ref = ego{1}.x0(end);    
ego{1}.y_ref = ego{1}.x0(2);      

% EGO 2
ego{2}.x_current = ego{2}.x0;
ego{2}.u_current = [0; 0];
ego{2}.V_ref = ego{2}.x0(end);    
ego{2}.y_ref = ego{2}.x0(2);       

% EGO 3 
ego{3}.x_current = ego{3}.x0;
ego{3}.u_current = [0; 0];
ego{3}.V_ref = ego{3}.x0(end);     
ego{3}.y_ref = ego{3}.x0(2);      

% HDV  
N_first_step = 3;
human = {hdv(hdv_x0, Nsim, false)};
brt_hp = load('BRT data/BRT_H_temp14.mat'); 
brt_hp = brt_hp.BRT;
brt_lp = load('BRT data/BRT_L_temp42.mat'); 
brt_lp1 = brt_lp.BRT;
brt_lp = load('BRT data/BRT_L_temp46.mat'); 
brt_lp2 = brt_lp.BRT;

%Simulation data
sim_data.ddelta_min = -0.087; 
sim_data.ddelta_max = 0.087;
sim_data.dv_min = -2; 
sim_data.dv_max = 2;   
sim_data.v_min = brt_lp1.g.min(end);
sim_data.v_max = brt_lp1.g.max(end);
sim_data.delta_min = brt_lp1.g.min(end-1);
sim_data.delta_max = brt_lp1.g.max(end-1);

% Constraint selection
brt_constr.ego = true;
brt_constr.hum = true;
constr_list.eucl = false;
constr_list.left_lane = true;
constr_list.right_lane = true;
constr_list.get_overtaken = false;
constr_list.decelerate = false;
constr_list.center_lane = false;
constr_list.lane_keeping = true;
constr_list.brt_constr = brt_constr;

%% OCP solvers creation
ocp_solver = cell(1, num_egos); 

for i_solver = 1:num_egos
    [model, constraint] = bicycle_model2(constr_list, sim_data);

    
    % Parameters assignment
    model.x0 = ego{i_solver}.x0;
    model.x_ref(end) = ego{i_solver}.V_ref;
    model.x_ref(2) = ego{i_solver}.y_ref;   

    ocp_model = acados_ocp_model();
    ocp_model.set('name', [model.name '_' num2str(i_solver)]);
    ocp_model.set('T', T);

    % Symbolics
    ocp_model.set('sym_x', model.x); 
    ocp_model.set('sym_u', model.u);
    ocp_model.set('sym_xdot', model.xdot); 
    ocp_model.set('sym_p', model.p);

    % Dynamics
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.f_expl_expr);
    
    % Set constraint on initial state in the model
    ocp_model.set('constr_x0', model.x0);
    
    ocp_model.set('constr_lbx', [model.y_min, model.delta_min, model.v_min]);
    ocp_model.set('constr_ubx', [model.y_max, model.delta_max, model.v_max]);
    ocp_model.set('constr_lbu', [model.ddelta_min, model.dv_min]);
    ocp_model.set('constr_ubu', [model.ddelta_max, model.dv_max]);

    %Constraints weights
    nh = length(constraint.nonli_expr);
    uh = ones(nh, 1); 
    lh = zeros(nh, 1);

    %Soft constraint lower bound
    lh(1:2)    = 0;   % Limit for C_left, C_right
    lh(3:10)   = 0;   % Limit for 8 distance constraints 
    lh(11:12)  = 0;   % Limit for C_overtaken, C_decelerate
    lh(13)     = 0;   % Limit for Center lane
    lh(14)     = 0;   % Limit for Lane Keeping
    lh(15)     = 0;   % Limit BRT vs ego 
    lh(16)     = 0;   % Limit BRT vs hum

    %Soft constraint upper bound
    uh(1:2)    = 1;      % Limit for C_left, C_right
    uh(3:10)   = 1e-3;   % Limit for 8 distance constraints
    uh(11:12)  = 1;      % Limit for C_overtaken, C_decelerate
    uh(13)     = 1;      % Limit for Center lane
    uh(14)     = 1;      % Limit for Lane keeping
    uh(15)     = 1000;   % Limit BRT vs ego
    uh(16)     = 1000;   % Limit BRT vs hum

    %Lower and Upper slack linear terms 
    soft_l_lin = [0, 0, ...                % 2x lanes
                   ones(1, 8) * 1e-3, ...  % 8x obs dist
                   1e-5, 1e-3, ...         % 2x overtake
                   0, 0, ...               % Center lane, Lane Keeping  
                   6000, 300]';            % 2x BRT constraints (ego, hum) 

    soft_u_lin = [10000, 10000, ...        % 2x lanes
                   ones(1, 8) * 0.01, ...  % 8x obs dist
                   1e-9, 1e-9, ...         % 2x overtake, decelerate
                   10, 100000, ...         % Center Lane, Lane Keeping
                   0, 0]';                 % 2x BRT constraints (ego, hum)

    %Lower and Upper slack quadratic terms 
    soft_l_quad = [0, 0, ...                % 2x lanes
                   ones(1, 8) * 0, ...      % 8x obs dist
                   0, 0, ...                % 2x overtake, decelerate  questi erano mille
                   0, 0, ...                % Center lane, Lane Keeping  
                   5, 0]';                  % 2x BRT constraints (ego, hum) 1000
 
    soft_u_quad = [10, 10, ...              % 2x lanes
                   ones(1, 8) * 0, ...      % 8x obs dist
                   0, 0, ...                % 2x overtake, decelerate 
                   0, 10000, ...            % Center Lane, Lane Keeping
                   0, 0]';                  % 2x BRT constraints (ego, hum)

    ocp_model = setConstraints(ocp_model, model, constr_list, constraint, uh, lh,...
                               soft_l_lin, soft_u_lin, soft_l_quad, soft_u_quad);


    % States and inputs weights
    Q = diag([0*1e-2, 1e-2, 1e-1, 1e1, 4.3e3]); 
    R = diag([ 1e1,  1e0]);  
    ocp_model.set('cost_W', blkdiag(Q, R));
    ocp_model.set('cost_W_e', Q/2);
    % Cost function (linear_ls)
    nx = length(model.x);
    nu = length(model.u);
    ocp_model.set('cost_type', 'linear_ls');
    ocp_model.set('cost_type_e', 'linear_ls');
    ny = nx + nu; 
    ny_e = nx;
    Vx = zeros(ny, nx); 
    Vx(1:nx,:) = eye(nx);
    Vu = zeros(ny, nu); 
    Vu(nx+1:end,:) = eye(nu);
    Vx_e = eye(ny_e, nx);
    ocp_model.set('cost_Vx', Vx); 
    ocp_model.set('cost_Vu', Vu); 
    ocp_model.set('cost_Vx_e', Vx_e);
    
    y_ref = [model.x_ref'; 0; 0]; 
    ocp_model.set('cost_y_ref', y_ref);
    ocp_model.set('cost_y_ref_e', model.x_ref');

    % Acados OCP options
    ocp_opts = acados_ocp_opts();
    ocp_opts.set('compile_interface', 'auto');
    ocp_opts.set('codgen_model', 'true');
    ocp_opts.set('param_scheme_N', N);
    ocp_opts.set('nlp_solver', 'sqp_rti');
    ocp_opts.set('qp_solver', 'partial_condensing_hpipm');
    ocp_opts.set('nlp_solver_exact_hessian', 'false');
    ocp_opts.set('qp_solver_cond_N', 50);
    ocp_opts.set('regularize_method', 'no_regularize');
    ocp_opts.set('sim_method', 'erk');
    ocp_opts.set('print_level', 1);    
    ocp_opts.set('sim_method_num_stages', 4);
    ocp_opts.set('sim_method_num_steps', 3);
    ocp_opts.set('nlp_solver_tol_stat', 1e-4);
    ocp_opts.set('nlp_solver_tol_eq', 1e-4);
    ocp_opts.set('nlp_solver_tol_ineq', 1e-4);
    ocp_opts.set('nlp_solver_tol_comp', 1e-4);
    % Create solver
    ocp_solver{i_solver} = acados_ocp(ocp_model, ocp_opts);
end

%% Simulation
simX = cell(1, num_egos); % vehicles' states over time
simU = cell(1, num_egos); % vehicles' control over time

for i = 1:num_egos
    simX{i} = zeros(Nsim, length(model.x)); 
    simU{i} = zeros(Nsim, length(model.u));
end

simX_hdv = zeros(Nsim, length(model.x));

brt_activated = cell(1, num_egos); 
for i = 1:num_egos
    brt_activated{i} = zeros(Nsim, 2); 
end

V_ego_vs_human = zeros(Nsim, num_egos);   %Value function ego vs human
V_ego_vs_ego = zeros(Nsim, num_egos);     %Value function ego vs ego
bhj_eh = zeros(Nsim, 3, num_egos);        %Contraint parameters ego vs human 
bhj_ee = zeros(Nsim, 3, num_egos);        %Contraint parameters ego vs ego 
cost_f = zeros(Nsim, num_egos);           %Cost functional value for each ego
time_ocp = cost_f;
time_tot = time_ocp;

for i = 1:num_egos
    ocp_solver{i}.set('constr_x0', ego{i}.x0);
    simX{i}(1, :) = ego{i}.x0; 
end

simX_hdv(1, :) = [human{1}.simX_0, 0]; 
u_egos = zeros(num_egos,2);
for i=1:num_egos
    u_egos(i,:) = ego{i}.u_current';
end

% 
y_ref_j = cell(1, num_egos);
y_ref_e = cell(1, num_egos);
for i = 1:num_egos
    y_ref_j{i} = [ego{i}.x0, 0, 0];
    y_ref_e{i} = ego{i}.x0;
end

SL = cell(1, num_egos);      % Lower slacks
SU = cell(1, num_egos);      % Upper slacks
ham_ego = cell(1, num_egos); % Hamiltonian values for each ego

for i = 1:num_egos
    SL{i} = zeros(Nsim, ocp_solver{i}.ocp.dims.nsh);
    SU{i} = zeros(Nsim, ocp_solver{i}.ocp.dims.nsh);
    ham_ego{i} = zeros(Nsim, 2);     % Column 1: vs human, Column 2: vs ego
end

states_mpc{1} = zeros(5,N,Nsim);
states_mpc{2} = zeros(5,N,Nsim);
states_mpc{3} = zeros(5,N,Nsim);

V_save{1} = zeros(1,Nsim);
V_save{2} = zeros(1,Nsim);
V_save{3} = zeros(1,Nsim);

%Fake ego, it is always outside of any grid
ego{num_egos+1}.x0 = [inf, inf, inf, inf, inf]; 
ego{num_egos+1}.x_current = ego{num_egos+1}.x0;
ego{num_egos+1}.u_current = [inf; inf];
u_egos = [u_egos; inf, inf];

% tic
for i = 1:Nsim

    ego_obs = setEgoFront3(ego);
    for ego_idx = 1:num_egos
        
        tic;

        p1 = [];

        obstacles = [ego{ego_obs(1,ego_idx)}.x_current([1:3,5])'; human{1}.simX_0'];
        p1 = [p1; obstacles]; 
        
        HP = 1;
       
        [V_h, onoff_h, p5_h, p6_h, bhj_h]=evaluateVPBhj(ego{ego_idx}.x_current, u_egos(ego_idx,:), human{1}.simX_0, human{1}.simU_0, model, brt_hp);
        brt_activated{ego_idx}(i,1) = onoff_h; % 0=off, 1=HP, 2=LP

        V_ego_vs_human(i, ego_idx) = V_h;
        bhj_eh(i,1,ego_idx) = p5_h;
        bhj_eh(i,2,ego_idx) = p6_h;
        bhj_eh(i,3,ego_idx) = bhj_h;
        

        [V_e1, onoff_e1, p5_e1, p6_e1, bhj_e1]=evaluateVPBhj(ego{ego_idx}.x_current, u_egos(ego_idx,:), ego{ego_obs(1,ego_idx)}.x_current, u_egos(ego_obs(1,ego_idx),:), model, brt_lp1);
        [V_e2, onoff_e2, p5_e2, p6_e2, bhj_e2]=evaluateVPBhj(ego{ego_idx}.x_current, u_egos(ego_idx,:), ego{ego_obs(2,ego_idx)}.x_current, u_egos(ego_obs(2,ego_idx),:), model, brt_lp2);
        
        [~, v_small] = find([V_e1, V_e2] == min([V_e1, V_e2]));
        switch v_small(1)
            case 1
                V_e = V_e1;
                onoff_e = onoff_e1;
                p5_e = p5_e1;
                p6_e = p6_e1;
                bhj_e = bhj_e1;
                id = 1;
            case 2 
                V_e = V_e2;
                onoff_e = onoff_e2;
                p5_e = p5_e2;
                p6_e = p6_e2;
                bhj_e = bhj_e2;
                id = 2;
        end
        V_save{ego_idx}(i) = V_e; 
        if onoff_h && ego{ego_idx}.x0(2) == human{1}.simX_0(2) && ... 
            abs(ego{ego_idx}.x0(2)-ego{ego_idx}.x_current(2)) <= 1 
            V_e = 0;
            onoff_e = 0;
            p5_e = 0;
            p6_e = 0;
            bhj_e = 0;
        end
        bhj_ee(i,1,ego_idx) = p5_e;
        bhj_ee(i,2,ego_idx) = p6_e;
        bhj_ee(i,3,ego_idx) = bhj_e; 
        V_ego_vs_ego(i,ego_idx) = V_e;

        if ego_obs(id,ego_idx) ~= 4
            brt_activated{ego_idx}(i,2) = onoff_e*ego_obs(id,ego_idx); 
        else
            brt_activated{ego_idx}(i,2) = onoff_e; 
        end

        % Determine lane centring constraint parameter
        if  (abs(ego{ego_idx}.x0(2)-ego{ego_idx}.x_current(2)) <= 0.5 && ...
            ego{ego_idx}.x0(2) ~= human{1}.simX_0(2)) || ...
            (abs(ego{ego_idx}.x0(2)-ego{ego_idx}.x_current(2)) >= 3.5 && onoff_h)
            onoff_lane = 1;
            if sign(ego{ego_idx}.x_current(2)) ~= sign(ego{ego_idx}.x0(2))
                lane_pos = ego{ego_idx}.x0(2);
            else
                lane_pos = -ego{ego_idx}.x0(2);
            end
        else
            onoff_lane = 0;
            lane_pos = -ego{ego_idx}.x0(2);
        end

       
        p1 = vertcat(p1,  onoff_e, onoff_h, onoff_lane, p5_e, p6_e, p5_h, p6_h, bhj_e, bhj_h, ...
            HP, lane_pos);

        for jj = 0:N
             if jj < N_first_step
                p1(9:10)=1;
                ocp_solver{ego_idx}.set('p', p1, jj);
             else
                p1(9:10)=0;
                ocp_solver{ego_idx}.set('p', p1, jj);
             end
        end

        for j=0:N-1
           ocp_solver{ego_idx}.set('cost_y_ref', y_ref_j{ego_idx}, j);
        end
        ocp_solver{ego_idx}.set('cost_y_ref_e', y_ref_e{ego_idx});
        

        
        ocp_solver{ego_idx}.solve();
        if ocp_solver{ego_idx}.get('status') ~= 0 && ocp_solver{ego_idx}.get('status') ~= 2
            error('EGO %d: Acados error: %d', ego_idx, ocp_solver{ego_idx}.get('status'));
        end

        time_ocp(i,ego_idx) = ocp_solver{ego_idx}.get('time_tot');
        time_tot(i,ego_idx) = toc;

        simX{ego_idx}(i, :) = ocp_solver{ego_idx}.get('x', 0)'; 
        simU{ego_idx}(i, :) = ocp_solver{ego_idx}.get('u', 0)';
        u_egos(ego_idx,:) = simU{ego_idx}(i, :);
        SL{ego_idx}(i,:) = ocp_solver{ego_idx}.get('sl', 0);
        SU{ego_idx}(i,:) = ocp_solver{ego_idx}.get('su', 0);
        
        ham_ego{ego_idx}(i,1) = u_egos(ego_idx,:)*[bhj_eh(i,1,ego_idx); bhj_eh(i,2,ego_idx)] + bhj_eh(i,3,ego_idx);
               
        ham_ego{ego_idx}(i,2) = u_egos(ego_idx,:)*[bhj_ee(i,1,ego_idx); bhj_ee(i,2,ego_idx)] + bhj_ee(i,3,ego_idx);

        
        cost_f(i,ego_idx) = ocp_solver{ego_idx}.get_cost(); % Save cost function value
    end
    
    % Next iteration update
    for ego_idx = 1:num_egos
        ego{ego_idx}.x_current = ocp_solver{ego_idx}.get('x', 1)';
        ocp_solver{ego_idx}.set('constr_x0', ego{ego_idx}.x_current);
        ocp_solver{ego_idx}.set('constr_lbx', ego{ego_idx}.x_current, 0);
        ocp_solver{ego_idx}.set('constr_ubx', ego{ego_idx}.x_current, 0);
        for k=1:N
            states_mpc{ego_idx}(:,k,i) = ocp_solver{ego_idx}.get('x', k)';
        end
    end
    
    human{1} = human_control(human{1}, i, model, dt);
    human{1} = human_integrate(human{1},i,dt);
    if i < Nsim
        simX_hdv(i+1, :) = [human{1}.simX_0, 0]; 
    end

    clc
    fprintf('Iterazione: %d / %d\n', i, Nsim);
    
end
% toc


%% Plots 
t = linspace(0.0, Nsim * dt, Nsim);

% Settings for video saving
do_video = false; 
if do_video
    video_filename = 'scenario_4.mp4'; 
    writerObj = VideoWriter(video_filename, 'MPEG-4'); 
    writerObj.FrameRate = 20; 
    open(writerObj);
    disp(['Salvataggio video in corso... File: ' video_filename]);
end
% Create subplot and figure
fig = figure('Name', 'Animazione Traiettorie con Indicatori BRT (3 Egos)');
% Upper subplot for vehicles animation
ax1 = subplot(4, 1, 1:3);
hold(ax1, 'on'); 
grid(ax1, 'on'); 
axis(ax1, 'equal');
title(ax1, 'Animazione Traiettorie'); 
xlabel(ax1, 'Posizione s [m]'); 
ylabel(ax1, 'Posizione y [m]');

D1 = 4.3; 
D2 = 1.8;
vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];

ego_patches = cell(1, num_egos);
colors = {'b', 'r', 'y'}; 
for i = 1:num_egos
    ego_patches{i} = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', colors{i}, 'DisplayName', ['Ego ' num2str(i)]);
end
hdv_patch = patch(ax1, 'XData', [], 'YData', [], 'FaceColor', [0.5 0.5 0.5], 'DisplayName', 'HDV');
legend(ax1, 'show', 'Location', 'northeast');

% Lower subplot for BRT constraint activation
ax2 = subplot(4, 1, 4);
hold(ax2, 'on');
axis(ax2, 'off'); 
xlim(ax2, [0 10]);
ylim(ax2, [0 (num_egos * 1.5) + 2]); 
title(ax2, 'Stato Attivazione Vincoli BRT');

% Constraint status color definition
color_off = [0.8 0.8 0.8];    % Grey: no BRT
color_hp_human = [1 0.2 0.2]; % Red: HP BRT vs hdv
color_lp_human = [0.2 0.5 1]; % Blue: LP BRT vs hdv
color_lp_ego = [1 0.8 0.2];   % Yellow: LP BRT vs Ego


% Indicators for each ego
indicator_e_h = cell(1, num_egos);
indicator_e_e = cell(num_egos, num_egos); 

for i_ego = 1:num_egos
    base_y = (num_egos - i_ego + 1) * 1.5; 

    text(ax2, 0.5, base_y + 0.5, ['Ego ' num2str(i_ego) ':'], 'FontSize', 10, 'HorizontalAlignment', 'right');
    
    % vs Human
    indicator_e_h{i_ego} = rectangle(ax2, 'Position', [1 base_y 1.5 0.8], 'FaceColor', color_off, 'EdgeColor', 'k');
    text(ax2, 1.75, base_y - 0.2, 'vs Human', 'HorizontalAlignment', 'center', 'FontSize', 8);
    
    % vs other Egos
    current_x_pos = 3;
    for j_ego = 1:num_egos
        if i_ego ~= j_ego
            indicator_e_e{i_ego, j_ego} = rectangle(ax2, 'Position', [current_x_pos base_y 1.5 0.8], 'FaceColor', color_off, 'EdgeColor', 'k');
            text(ax2, current_x_pos + 0.75, base_y - 0.2, ['vs Ego ' num2str(j_ego)], 'HorizontalAlignment', 'center', 'FontSize', 8);
            current_x_pos = current_x_pos + 1.8; 
        end
    end
end

% Legend for colors
legend_y_start = 3; 
rectangle(ax2, 'Position', [7 legend_y_start + 1.6 0.5 0.5], 'FaceColor', color_hp_human, 'EdgeColor', 'k');
text(ax2, 7.7, legend_y_start + 1.85, 'Alta Priorità (Umano)', 'VerticalAlignment', 'middle');
rectangle(ax2, 'Position', [7 legend_y_start + 0.8 0.5 0.5], 'FaceColor', color_lp_human, 'EdgeColor', 'k');
text(ax2, 7.7, legend_y_start + 1.05, 'Bassa Priorità (Umano)', 'VerticalAlignment', 'middle');
rectangle(ax2, 'Position', [7 legend_y_start 0.5 0.5], 'FaceColor', color_lp_ego, 'EdgeColor', 'k');
text(ax2, 7.7, legend_y_start + 0.25, 'Bassa Priorità (Ego)', 'VerticalAlignment', 'middle');
rectangle(ax2, 'Position', [7 legend_y_start - 0.8 0.5 0.5], 'FaceColor', color_off, 'EdgeColor', 'k');
text(ax2, 7.7, legend_y_start - 0.55, 'Nessun Vincolo Attivo', 'VerticalAlignment', 'middle');

% Animation loop
for i = 1:Nsim
    % Vehicles update
    for ego_idx = 1:num_egos
        s = simX{ego_idx}(i, 1); y = simX{ego_idx}(i, 2); theta = simX{ego_idx}(i, 3);
        rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        verts = (rot * vehicle_verts_unrotated')' + [s, y];
        set(ego_patches{ego_idx}, 'XData', verts(:,1), 'YData', verts(:,2));
    end
    
    s_hdv = simX_hdv(i, 1); y_hdv = simX_hdv(i, 2); theta_hdv = simX_hdv(i, 3);
    rot_hdv = [cos(theta_hdv), -sin(theta_hdv); sin(theta_hdv), cos(theta_hdv)];
    verts_hdv = (rot_hdv * vehicle_verts_unrotated')' + [s_hdv, y_hdv];
    set(hdv_patch, 'XData', verts_hdv(:,1), 'YData', verts_hdv(:,2));
    
    % title and axis update 
    title(ax1, sprintf('Animazione Traiettorie (Tempo: %.2f s)', t(i)));
    axis(ax1, [simX{1}(i,1)-20 simX{1}(i,1)+40 -lane_width/2-1 lane_width/2+1]); 

     % Indicators update
    for ego_idx = 1:num_egos
        % Ego vs Human
        status_h = brt_activated{ego_idx}(i, 1);
        switch status_h
            case 0 % No BRT
                set(indicator_e_h{ego_idx}, 'FaceColor', color_off);
            case 1 % HP
                set(indicator_e_h{ego_idx}, 'FaceColor', color_hp_human);
            case 2 % LP
                set(indicator_e_h{ego_idx}, 'FaceColor', color_lp_human);
        end
        
        % Ego vs other Egos
   
        status_e = brt_activated{ego_idx}(i, 2);
        if status_e == 1 
            set(indicator_e_e{ego_idx, 1}, 'FaceColor', color_lp_ego);
            set(indicator_e_e{ego_idx, 2}, 'FaceColor', color_off);
            set(indicator_e_e{ego_idx, 3}, 'FaceColor', color_off);
        elseif status_e == 2 % No BRT
            set(indicator_e_e{ego_idx, 1}, 'FaceColor', color_off);
            set(indicator_e_e{ego_idx, 2}, 'FaceColor', color_lp_ego);
            set(indicator_e_e{ego_idx, 3}, 'FaceColor', color_off);
        elseif status_e == 3 % No BRT
            set(indicator_e_e{ego_idx, 1}, 'FaceColor', color_off);
            set(indicator_e_e{ego_idx, 2}, 'FaceColor', color_off);
            set(indicator_e_e{ego_idx, 3}, 'FaceColor', color_lp_ego);
        else % No BRT
            set(indicator_e_e{ego_idx, 1}, 'FaceColor', color_off);
            set(indicator_e_e{ego_idx, 2}, 'FaceColor', color_off);
            set(indicator_e_e{ego_idx, 3}, 'FaceColor', color_off);
        end     
    end
    
    drawnow;
    
    % Frame capture
    if do_video
        frame = getframe(fig); 
        writeVideo(writerObj, frame);
    end
end
% Finalize video
if do_video
    close(writerObj);
    fprintf('Video salvato correttamente come: %s\n', video_filename);
end
hold(ax1, 'off');
% ttc_dyn_data_7.ev_states_dyn = ev_states_dyn;
% ttc_dyn_data_7.time_future_dyn = time_future_dyn;

% figure

% plot(t, V_save{1})
