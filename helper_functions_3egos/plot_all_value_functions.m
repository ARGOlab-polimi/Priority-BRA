function handles = plot_all_value_functions(V_ego_vs_human, V_ego_vs_ego, brt_activated, Tf, Nsim, num_egos)

% Time vector
t = linspace(0, Tf, Nsim);

default_colors = get(groot, 'defaultAxesColorOrder');


axis_font_settings = struct(...
    'FontSize', 16, ... 
    'FontName', 'Times New Roman', ...
    'TickLabelInterpreter', 'latex');

    
% Output struct initialization
handles = struct();
handles.ego_figs = cell(num_egos, 1);
handles.ego_axes = cell(num_egos, 1);


% plot hdv vs egos
fig_h = figure('Name', 'Human-Ego Value Function', ...
               'Units', 'centimeters');
ax_h = axes('Parent', fig_h);
set(ax_h, axis_font_settings);
hold(ax_h, 'on');

plot(ax_h, t, V_ego_vs_human(:, 1:num_egos), 'LineWidth', 2.0);

xlabel(ax_h, '$t$ [s]', 'Interpreter', 'latex', 'FontSize', 18); %erano 16
ylabel(ax_h, '$V$', 'Interpreter', 'latex', 'FontSize', 18);
% title(ax_h, 'Human-Ego Value Function', 'Interpreter', 'latex', 'FontSize', 16);

legend_labels = arrayfun(@(i) sprintf('LP%d-HP', i), 1:num_egos, 'UniformOutput', false);
legend(ax_h, legend_labels, 'Interpreter', 'latex', 'FontSize', 16, ...  %era 14
       'Location', 'best', 'Box', 'on');
       
grid(ax_h, 'on');
box(ax_h, 'on');
hold(ax_h, 'off');

handles.human_fig = fig_h;
handles.human_ax = ax_h;


% Plot egos vs egos
for ego_i = 1:num_egos
    fig_name = sprintf('Value Function for LP%d', ego_i);
    fig_e = figure('Name', fig_name, ...
                   'Units', 'centimeters');
    ax_e = axes('Parent', fig_e);
    set(ax_e, axis_font_settings);
    hold(ax_e, 'on');
    
    plot_handles = [];
    current_figure_legends = {};
    
    for ii = 1:num_egos
        try
            idx_active = (brt_activated{ego_i}(:, 2) == ii);
        catch E
            warning('Errore nel processare brt_activated per ego_i=%d, ii=%d. Errore: %s', ego_i, ii, E.message);
            continue; 
        end
        
        if any(idx_active)
            t_active = t(idx_active);
            V_active = V_ego_vs_ego(idx_active, ego_i);
            
            h = plot(ax_e, t_active, V_active, 'LineWidth', 2.0, ...
                     'Color', default_colors(ii, :));
            
            plot_handles = [plot_handles, h];
            current_figure_legends = [current_figure_legends, sprintf('with LP%d', ii)];
        end
    end
    
    xlabel(ax_e, '$t$ [s]', 'Interpreter', 'latex', 'FontSize', 18); 
    ylabel(ax_e, '$V$', 'Interpreter', 'latex', 'FontSize', 18);
    % title(ax_e, sprintf('Value Function for LP%d', ego_i), 'Interpreter', 'latex', 'FontSize', 16);

    if ~isempty(plot_handles)
        legend(ax_e, plot_handles, current_figure_legends, ...
               'Interpreter', 'latex', 'FontSize', 16, 'Location', 'best', 'Box', 'on'); %era 14
    end
           
    grid(ax_e, 'on');
    box(ax_e, 'on');
    hold(ax_e, 'off');
    
    handles.ego_figs{ego_i} = fig_e;
    handles.ego_axes{ego_i} = ax_e;
end

fprintf('Creazione di %d grafici completata.\n', 1 + num_egos);

end