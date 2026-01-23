function [handles, ttc] = create_image_ttc2(simX, target_time, dt, id_vehicles, ttcs)

axis_font_settings = struct(...
    'FontSize', 14, ...
    'FontName', 'Times New Roman', ...
    'TickLabelInterpreter', 'latex');

Nsim = size(simX{1}, 1);
t_max = (Nsim - 1) * dt;
if target_time > t_max
    target_time = t_max;
    warning('Target time is beyond simulation duration. Showing the last frame at t=%.2f s.', t_max);
elseif target_time < 0
    target_time = 0;
    warning('Target time is negative. Showing the first frame at t=0 s.');
end
frame_idx = round(target_time / dt) + 1; 
frame_idx = max(1, min(frame_idx, Nsim));
current_time = (frame_idx - 1) * dt;

fig_ttc = figure('Name', 'TTC Time Series', ...
                 'Units', 'centimeters');
ax_ttc = axes('Parent', fig_ttc);
set(ax_ttc, axis_font_settings);
hold(ax_ttc, 'on');

ids_sorted = sort(id_vehicles); 
if isequal(ids_sorted, [1 2])
    col_idx = 1; plot_title = 'TTC between Ego 1 \& Ego 2';
elseif isequal(ids_sorted, [1 3])
    col_idx = 2; plot_title = 'TTC between Ego 1 \& Ego 3';
elseif isequal(ids_sorted, [2 3])
    col_idx = 3; plot_title = 'TTC between Ego 2 \& Ego 3';
elseif isequal(ids_sorted, [0 1])
    col_idx = 4; plot_title = 'TTC between Ego 1 \& HDV';
elseif isequal(ids_sorted, [0 2])
    col_idx = 5; plot_title = 'TTC between Ego 2 \& HDV';
elseif isequal(ids_sorted, [0 3])
    col_idx = 6; plot_title = 'TTC between Ego 3 \& HDV';
else
    warning('Combinazione id_vehicles non riconosciuta. Uso Colonna 1.');
    col_idx = 1; plot_title = 'TTC (Unknown Pair)';
end

t_vec = (0:Nsim-1)' * dt; 
data_to_plot = ttcs(:, col_idx);

plot(ax_ttc, t_vec, data_to_plot, '.', 'MarkerSize', 10, ...
     'MarkerEdgeColor', [0 0.4470 0.7410], ...
     'DisplayName', 'TTC History');

current_value = data_to_plot(frame_idx);
plot(ax_ttc, current_time, current_value, 'ro', 'MarkerFaceColor', 'r', ...
     'MarkerSize', 10, 'HandleVisibility', 'off');

text_str = sprintf('TTC = %.2f s', current_value);
text_offset = 0.02 * t_max; 
text(ax_ttc, current_time + text_offset, current_value, text_str, ...
     'HorizontalAlignment', 'left', ...
     'VerticalAlignment', 'bottom', ...
     'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');

xlabel(ax_ttc, 'Simulation time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel(ax_ttc, 'TTC [s]', 'Interpreter', 'latex', 'FontSize', 16);
% title(ax_ttc, plot_title, 'Interpreter', 'latex', 'FontSize', 16);
grid(ax_ttc, 'on');
box(ax_ttc, 'on');
% axis equal
xlim(ax_ttc, [0 t_max]);
ylim(ax_ttc, [0 10]); 

hold(ax_ttc, 'off');

handles.fig_ttc = fig_ttc;
handles.ax_ttc = ax_ttc;

ttc = current_value;
end