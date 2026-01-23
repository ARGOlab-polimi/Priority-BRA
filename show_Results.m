clc
clear all
close all 

addpath('Results', 'helper_functions_2egos')          % i suggest to put all the datas inside a folder called "Results". In this way this script should run without any change.

plot_value_functions = 0;   % change to 1 to plot the value functions of all scenarios

ttc_kin = 1;                % 0 to show dynamic ttc, 1 to show kinematic ttc

plot_ttc_diagrams = 1;      % change to 1 to plot the ttc diagrams of all scenarios 

plot_ttc_statistics = 1;    % change to 1 to plot the mean and std deviation for all scenarios (separetly and all together)

play_scenario = 0;          % change to 1-9 (2 excluded) to play the corresponding scenario's video

play_ttc_frame = 0;         % change to 1-9 (2 excluded) to select the corresponding scenario to check visually the ttc value between two vehicles 
id_vehicles = [1 0];        % change to 0-3 to select the two vehicles, 1 for ego1, 2 for ego2, 3 for ego3, 0 for hdv
target_time = 10.6266;      % time of the simulation of which i want to see the ttc configuration and evolution

plot_full_vf = 0;

%Load all results
res1 = load("Results\scenario1_new.mat");
res1 = res1.res;
results{1} = res1;

res = load("Results\scenario3.mat");
res3 = res.res;
results{2} = res3;

res = load("Results\scenario4_new.mat");
res4 = res.res;
results{3} = res4;

res = load("Results\scenario5.mat");
res5 = res.res;
results{4} = res5;

res = load("Results\scenario6.mat");
res6 = res.res;
results{5} = res6;

res = load("Results\scenario7.mat");
res7 = res.res;
results{6} = res7;

% res = load("Results\scenario8.mat");
% res8 = res.res;
% results{7} = res8;

res = load("Results\scenario9.mat");
res9 = res.res;
results{7} = res9;

V = load("Results\V_ego_scenario1.mat");
V_ego_full{1} = V.V_save;
V = load("Results\V_ego_scenario4.mat");
V_ego_full{2} = V.V_save;

ttc = load("Results\ttc1.mat");
ttc_standard{1} = ttc.ttcs;
ttc = load("Results\ttc3.mat");
ttc_standard{2} = ttc.ttcs;
ttc = load("Results\ttc4.mat");
ttc_standard{3} = ttc.ttcs;
ttc = load("Results\ttc5.mat");
ttc_standard{4} = ttc.ttcs;
ttc = load("Results\ttc6.mat");
ttc_standard{5} = ttc.ttcs;
ttc = load("Results\ttc7.mat");
ttc_standard{6} = ttc.ttcs;
ttc = load("Results\ttc9.mat");
ttc_standard{7} = ttc.ttcs;

clear res V ttc

% Value function plot
if plot_value_functions 
for i=1:7
    
    V_ego_vs_human = results{i}.V_ego_vs_human;
    V_ego_vs_ego = results{i}.V_ego_vs_ego;
    t = results{i}.t;
    brt_activated = results{i}.brt_activated;

    figure
    subplot(2,2,1)
    plot(t,V_ego_vs_human(:,1), '.', MarkerSize=12);
    hold on
    plot(t,V_ego_vs_human(:,2), '.', MarkerSize=12);
    hold on
    plot(t,V_ego_vs_human(:,3), '.', MarkerSize=12);
    legend('LP1-HP', 'LP2-HP', 'LP3-HP')
    title('Human-Ego Value Function')
    grid on
    xlabel('$t$ [s]', 'Interpreter', 'latex')
    ylabel('$V$', 'Interpreter', 'latex')
    
    for ego_i = 1:3
        % figure;
        subplot(2,2,ego_i+1)
        hold on; 
        current_figure_legends = {};
        plot_handles = [];
    
        for ii = 1:3
            if any(brt_activated{ego_i}(:,2)==ii)
                h = plot(t(brt_activated{ego_i}(:,2)==ii), V_ego_vs_ego((brt_activated{ego_i}(:,2)==ii),ego_i), '.', MarkerSize=12);
                plot_handles = [plot_handles, h];
                current_figure_legends = [current_figure_legends, ['with LP' num2str(ii)]];
            end
        end
    
        if ~isempty(plot_handles)
            legend(plot_handles, current_figure_legends);
        end
    
        title(['Value function for LP' num2str(ego_i)]);
        hold off; % Rilascia il plot corrente
        xlabel('$t$ [s]', 'Interpreter', 'latex')
        ylabel('$V$', 'Interpreter', 'latex')
        grid on
    end
    if i == 1
        sgtitle(['Scenario ' num2str(i) ' Value Functions'])
    elseif i == 7
        sgtitle(['Scenario ' num2str(i+2) ' Value Functions'])
    else
        sgtitle(['Scenario ' num2str(i+1) ' Value Functions'])
    end
end
end

% plot ttc
if plot_ttc_diagrams && ttc_kin == 0
    for i=1:7
        ttcs = results{i}.ttc_dyn_data.ttcs_dyn;
        t = results{i}.t;
        figure
        subplot(3,2,1)
        plot(t, ttcs(:,1), '.')
        ylim([0 10])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego1 and ego2')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,3)
        plot(t, ttcs(:,2),  '.')
        ylim([0 10])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego1 and ego3')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,5)
        plot(t, ttcs(:,3), '.')
        ylim([0 10])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego2 and ego3')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,2)
        plot(t, ttcs(:,4), '.')
        ylim([0 10])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego1 and hdv')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,4)
        plot(t, ttcs(:,5), '.')
        ylim([0 10])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego2 and hdv')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,6)
        plot(t, ttcs(:,6), '.')
        ylim([0 10])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego3 and hdv')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        if i == 1
            sgtitle(['Scenario ' num2str(i) ' TTCs'])
        elseif i == 7
            sgtitle(['Scenario ' num2str(i+2) ' TTCs'])
        else
            sgtitle(['Scenario ' num2str(i+1) ' TTCs'])
        end
    end
end

%% plot ttc statistics
if plot_ttc_statistics && ttc_kin == 0

    ttc_all_ego_ego = [];
    ttc_all_ego_hum = [];

    for i = 1:7
        ttc_ego_ego = results{i}.ttc_dyn_data.ttcs_dyn(:,1:3);
        ttc_ego_hum = results{i}.ttc_dyn_data.ttcs_dyn(:,4:6);
    
        ttc_ego_ego(ttc_ego_ego >= 10) = [];
        ttc_ego_hum(ttc_ego_hum >= 10) = [];
    
        mean_ego_ego = mean(ttc_ego_ego);
        mean_ego_hum = mean(ttc_ego_hum);
    
        std_dev_ego_ego = std(ttc_ego_ego);
        std_dev_ego_hum = std(ttc_ego_hum);
    
        min_value_ego_ego = min(ttc_ego_ego);
        min_value_ego_hum = min(ttc_ego_hum);
    
        groups = {'Av vs Av', 'Av vs Hum'};
        means = [mean_ego_ego, mean_ego_hum];    
        stds = [std_dev_ego_ego, std_dev_ego_hum]; 
        min_vals = [min_value_ego_ego, min_value_ego_hum];
    
        ttc_all_ego_ego = [ttc_all_ego_ego, ttc_ego_ego];
        ttc_all_ego_hum = [ttc_all_ego_hum, ttc_ego_hum];
    
        figure;
        b = bar(means);
        b.FaceColor = [0 0.5 0.7];
        
        hold on;  
        
        ngroups = length(groups);
        errorbar(1:ngroups, means, stds, 'k', 'linestyle', 'none', 'LineWidth', 2);
        
        if isempty(min_value_ego_ego) == 0 
            label_str = sprintf('---Min: %.2f s---', min_value_ego_ego);
            
            text(1, min_value_ego_ego, label_str, ...
                 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom', ...
                 'FontSize', 10, 'FontWeight', 'bold');
        end
    
        if isempty(min_value_ego_hum) == 0 
            label_str = sprintf('---Min: %.2f s---', min_value_ego_hum);
            
            text(2, min_value_ego_hum, label_str, ...
                 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom', ...
                 'FontSize', 10, 'FontWeight', 'bold');
        end
    
        xticks(1:ngroups);
        xticklabels(groups);
        ylabel('Value');
        if i == 1
            title(['Scenario ' num2str(i) ' TTCs statistics'])
        elseif i == 7
            title(['Scenario ' num2str(i+2) ' TTCs statistics'])
        else
            title(['Scenario ' num2str(i+1) ' TTCs statistics'])
        end
        hold off;
    end

    mean_ego_ego = mean(ttc_all_ego_ego);
    mean_ego_hum = mean(ttc_all_ego_hum);

    std_dev_ego_ego = std(ttc_all_ego_ego);
    std_dev_ego_hum = std(ttc_all_ego_hum);

    min_value_ego_ego = min(ttc_all_ego_ego);
    min_value_ego_hum = min(ttc_all_ego_hum);

    groups = {'Av vs Av', 'Av vs Hum'};
    means = [mean_ego_ego, mean_ego_hum];    
    stds = [std_dev_ego_ego, std_dev_ego_hum]; 
    min_vals = [min_value_ego_ego, min_value_ego_hum];

    figure;
    b = bar(means);
    b.FaceColor = [0 0.5 0.7];
    
    hold on;  
    
    ngroups = length(groups);
    errorbar(1:ngroups, means, stds, 'k', 'linestyle', 'none', 'LineWidth', 2);
    
    if isempty(min_value_ego_ego) == 0 
        label_str = sprintf('---Min: %.2f s---', min_value_ego_ego);
        
        text(1, min_value_ego_ego, label_str, ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', ...
             'FontSize', 10, 'FontWeight', 'bold');
    end

    if isempty(min_value_ego_hum) == 0 
        label_str = sprintf('---Min: %.2f s---', min_value_ego_hum);
        
        text(2, min_value_ego_hum, label_str, ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', ...
             'FontSize', 10, 'FontWeight', 'bold');
    end

    xticks(1:ngroups);
    xticklabels(groups);
    ylabel('Value');
    
    title('All scenarios TTCs statistics')
end

if plot_ttc_diagrams && ttc_kin 
    for i=1:7
        ttcs = ttc_standard{i};
        t = results{i}.t;
        figure
        subplot(3,2,1)
        plot(t, ttcs(:,1), '.')
        ylim([0 20])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego1 and ego2')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,3)
        plot(t, ttcs(:,2),  '.')
        ylim([0 20])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego1 and ego3')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,5)
        plot(t, ttcs(:,3), '.')
        ylim([0 20])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego2 and ego3')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,2)
        plot(t, ttcs(:,4), '.')
        ylim([0 20])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego1 and hdv')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,4)
        plot(t, ttcs(:,5), '.')
        ylim([0 20])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego2 and hdv')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        subplot(3,2,6)
        plot(t, ttcs(:,6), '.')
        ylim([0 20])
        xlim([0 t(end)])
        grid on
        title('Time to collision between ego3 and hdv')
        xlabel('Simulation Time [s]')
        ylabel('TTC [s]')
        if i == 1
            sgtitle(['Scenario ' num2str(i) ' TTCs'])
        elseif i == 7
            sgtitle(['Scenario ' num2str(i+2) ' TTCs'])
        else
            sgtitle(['Scenario ' num2str(i+1) ' TTCs'])
        end
    end
end

%% plot ttc statistics
if plot_ttc_statistics && ttc_kin

    ttc_all_ego_ego = [];
    ttc_all_ego_hum = [];

    for i = 1:7
        ttc_ego_ego = ttc_standard{i}(:,1:3);
        ttc_ego_hum = ttc_standard{i}(:,4:6);
    
        ttc_ego_ego(ttc_ego_ego >= 20) = [];  %con 20 alzerei la media
        ttc_ego_hum(ttc_ego_hum >= 20) = [];
    
        mean_ego_ego = mean(ttc_ego_ego);
        mean_ego_hum = mean(ttc_ego_hum);
    
        std_dev_ego_ego = std(ttc_ego_ego);
        std_dev_ego_hum = std(ttc_ego_hum);
    
        min_value_ego_ego = min(ttc_ego_ego);
        min_value_ego_hum = min(ttc_ego_hum);
    
        groups = {'Av vs Av', 'Av vs Hum'};
        means = [mean_ego_ego, mean_ego_hum];    
        stds = [std_dev_ego_ego, std_dev_ego_hum]; 
        min_vals = [min_value_ego_ego, min_value_ego_hum];
    
        ttc_all_ego_ego = [ttc_all_ego_ego, ttc_ego_ego];
        ttc_all_ego_hum = [ttc_all_ego_hum, ttc_ego_hum];
    
        figure;
        b = bar(means);
        b.FaceColor = [0 0.5 0.7];
        
        hold on;  
        
        ngroups = length(groups);
        errorbar(1:ngroups, means, stds, 'k', 'linestyle', 'none', 'LineWidth', 2);
        
        if isempty(min_value_ego_ego) == 0 
            label_str = sprintf('---Min: %.2f s---', min_value_ego_ego);
            
            text(1, min_value_ego_ego, label_str, ...
                 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom', ...
                 'FontSize', 10, 'FontWeight', 'bold');
        end
    
        if isempty(min_value_ego_hum) == 0 
            label_str = sprintf('---Min: %.2f s---', min_value_ego_hum);
            
            text(2, min_value_ego_hum, label_str, ...
                 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom', ...
                 'FontSize', 10, 'FontWeight', 'bold');
        end
    
        xticks(1:ngroups);
        xticklabels(groups);
        ylabel('Value');
        if i == 1
            title(['Scenario ' num2str(i) ' TTCs statistics'])
        elseif i == 7
            title(['Scenario ' num2str(i+2) ' TTCs statistics'])
        else
            title(['Scenario ' num2str(i+1) ' TTCs statistics'])
        end
        hold off;
    end

    mean_ego_ego = mean(ttc_all_ego_ego);
    mean_ego_hum = mean(ttc_all_ego_hum);

    std_dev_ego_ego = std(ttc_all_ego_ego);
    std_dev_ego_hum = std(ttc_all_ego_hum);

    min_value_ego_ego = min(ttc_all_ego_ego);
    min_value_ego_hum = min(ttc_all_ego_hum);

    groups = {'Av vs Av', 'Av vs Hum'};
    means = [mean_ego_ego, mean_ego_hum];    
    stds = [std_dev_ego_ego, std_dev_ego_hum]; 
    min_vals = [min_value_ego_ego, min_value_ego_hum];

    figure;
    b = bar(means);
    b.FaceColor = [0 0.5 0.7];
    
    hold on;  
    
    ngroups = length(groups);
    errorbar(1:ngroups, means, stds, 'k', 'linestyle', 'none', 'LineWidth', 2);
    
    if isempty(min_value_ego_ego) == 0 
        label_str = sprintf('---Min: %.2f s---', min_value_ego_ego);
        
        text(1, min_value_ego_ego, label_str, ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', ...
             'FontSize', 10, 'FontWeight', 'bold');
    end

    if isempty(min_value_ego_hum) == 0 
        label_str = sprintf('---Min: %.2f s---', min_value_ego_hum);
        
        text(2, min_value_ego_hum, label_str, ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', ...
             'FontSize', 10, 'FontWeight', 'bold');
    end

    xticks(1:ngroups);
    xticklabels(groups);
    ylabel('Value');
    
    title('All scenarios TTCs statistics')
end


%% play scenario 
if play_scenario ~= 0 
    if play_scenario == 2 || play_scenario > 9
        error('The scenarios indexes are from 1 to 9, 2 is neglected.')
    end
    if play_scenario > 1
        play_scenario = play_scenario-1;
    end
    if play_scenario > 7
        play_scenario = play_scenario-1;
    end
    simX = results{play_scenario}.simX;
    simX_hdv = results{play_scenario}.simX_hdv;
    num_egos = 3;
    brt_activated = results{play_scenario}.brt_activated;
    Nsim = length(results{play_scenario}.t);
    t = results{play_scenario}.t;
    lane_width = 8;

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
    end
    hold(ax1, 'off');
end


if play_ttc_frame ~= 0
    if play_ttc_frame == 2 || play_ttc_frame > 9
        error('The scenarios indexes are from 1 to 9, 2 and 8 is neglected.')
    end
    if play_ttc_frame > 1
        play_ttc_frame = play_ttc_frame-1;
    end
    if play_ttc_frame > 7
        play_ttc_frame = play_ttc_frame-1;
    end
    ttcs = results{play_ttc_frame}.ttc_dyn_data.ttcs_dyn;
    ev_states = results{play_ttc_frame}.ttc_dyn_data.ev_states_dyn;
    time_future = results{play_ttc_frame}.ttc_dyn_data.time_future_dyn;
    Nsim = length(results{play_ttc_frame}.t);
    simX{1} = results{play_ttc_frame}.simX{1};
    dt = 3/60;
    model.params.D1 = 4.3;
    model.params.D2 = 1.8;
    t = results{play_ttc_frame}.t;

    [~,ttc] = create_image_ttc2(simX, target_time, dt, id_vehicles, ttcs);
    checkTTC_dyn(ttcs, ev_states, dt, target_time, id_vehicles, simX, time_future)
    plot_frames_ttc(ev_states, target_time, ttcs, id_vehicles, model, simX, dt, time_future)
end

if plot_full_vf
    [m, ~] = find(results{1}.brt_activated{1}(:,2) == 0);
    time_off1 = results{1}.t(m(1)-1);
    time_on1 = results{1}.t(m(end)+1);

    [m, ind] = find(results{3}.brt_activated{1}(:,2) == 0);
    time_off4 = results{1}.t(m(1)-1);
    time_on4 = results{1}.t(m(end)+1);

    figure 
    plot(results{1}.t, V_ego_full{1}{1})
    hold on
    xline(time_off1)
    hold on
    xline(time_on1)
    title('Scenario 1 full value function for vehicle 1')
    figure 
    plot(results{3}.t, V_ego_full{2}{1})
    hold on
    xline(time_off4)
    hold on
    xline(time_on4)
    title('Scenario 4 full value function for vehicle 1')
end