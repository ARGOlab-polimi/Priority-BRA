function [handles, ttc] = create_image_ttc(simX, simX_hdv, target_time, dt, lane_width, id_vehicles, ttcs)
%CREATE_PUBLICATION_PLOTS_TTC
axis_font_settings = struct(...
    'FontSize', 14, ...
    'FontName', 'Times New Roman', ...
    'TickLabelInterpreter', 'latex');
line_width_road = 2.0;
line_width_trail = 1.2;

% 1. Compute time and index
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

% 2.1 Setup figure 1
fig_sim = figure('Name', sprintf('Simulation at t = %.2f s', current_time), ...
                 'Units', 'centimeters'); 
ax_sim = axes('Parent', fig_sim);
set(ax_sim, axis_font_settings);
hold(ax_sim, 'on');
axis(ax_sim, 'equal');
box(ax_sim, 'on');

% 2.2 Plot Road Boundaries
x_lims = [simX{1}(frame_idx, 1) - 20, simX{1}(frame_idx, 1) + 20];
y_boundary = lane_width / 2;
plot(ax_sim, x_lims, [y_boundary, y_boundary], 'k-', 'LineWidth', line_width_road, 'HandleVisibility', 'off');
plot(ax_sim, x_lims, [-y_boundary, -y_boundary], 'k-', 'LineWidth', line_width_road, 'HandleVisibility', 'off');
plot(ax_sim, x_lims, [0, 0], 'k-.', 'LineWidth', line_width_trail, 'HandleVisibility', 'off');

% 2.3 Define vehicle properties 
num_egos = length(simX);
ego_colors = {[0 0 1], [1 0 0], [1 1 0]}; 
hdv_color = [0.5 0.5 0.5]; 
alpha_visible = 1.0;
alpha_transparent = 0.25;
D1 = 4.3; D2 = 1.8;
% D1 = 4.68; D2 = 2.2; % for thesis plot 
vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
tri_height = 0.3;
tri_base_width = D2 * 0.3;

% 2.4 Plot Ego Vehicles
for i_ego = 1:num_egos
    if ismember(i_ego, id_vehicles)
        current_alpha = alpha_visible;
    else
        current_alpha = alpha_transparent;
    end
    current_color_rgb = ego_colors{i_ego};
    
    plot(ax_sim, simX{i_ego}(1:frame_idx, 1), simX{i_ego}(1:frame_idx, 2), ...
         'Color', [current_color_rgb, current_alpha], ...
         'LineStyle', '--', 'LineWidth', line_width_trail, 'HandleVisibility','off');
    
    state = simX{i_ego}(frame_idx, :);
    s = state(1); y = state(2); theta = state(3);
    rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    verts = (rot_matrix * vehicle_verts_unrotated')' + [s, y];
    
    patch(ax_sim, 'XData', verts(:,1), 'YData', verts(:,2), ...
          'FaceColor', current_color_rgb, 'EdgeColor', 'k', ...
          'FaceAlpha', current_alpha, 'EdgeAlpha', current_alpha, ...
          'DisplayName', sprintf('Ego %d', i_ego)); 

    theta_normalized = atan2(sin(theta), cos(theta)); 
    if abs(theta_normalized) < pi/2
        x_base = D1/2; x_tip = D1/2 + tri_height;
    else
        x_base = -D1/2; x_tip = -D1/2 - tri_height;
    end
    tri_verts_unrotated = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
    tri_verts = (rot_matrix * tri_verts_unrotated')' + [s, y];
        
    patch(ax_sim, 'XData', tri_verts(:,1), 'YData', tri_verts(:,2), ...
              'FaceColor', 'k','FaceAlpha', current_alpha, ...
              'EdgeAlpha', current_alpha,'EdgeColor', 'k', 'HandleVisibility', 'off');
end

% 2.5 Plot HDV
if ismember(0, id_vehicles)
    current_alpha_hdv = alpha_visible;
else
    current_alpha_hdv = alpha_transparent;
end

plot(ax_sim, simX_hdv(1:frame_idx, 1), simX_hdv(1:frame_idx, 2), ...
     'Color', [hdv_color, current_alpha_hdv], ...
     'LineStyle', '--', 'LineWidth', line_width_trail, 'HandleVisibility','off');
state_hdv = simX_hdv(frame_idx, :);
s_hdv = state_hdv(1); y_hdv = state_hdv(2); theta_hdv = state_hdv(3);
rot_hdv = [cos(theta_hdv), -sin(theta_hdv); sin(theta_hdv), cos(theta_hdv)];
verts_hdv = (rot_hdv * vehicle_verts_unrotated')' + [s_hdv, y_hdv];
patch(ax_sim, 'XData', verts_hdv(:,1), 'YData', verts_hdv(:,2), ...
      'FaceColor', hdv_color, 'EdgeColor', 'k', ...
      'FaceAlpha', current_alpha_hdv, 'EdgeAlpha', current_alpha_hdv, ...
      'DisplayName', 'HDV');

theta_hdv_normalized = atan2(sin(theta_hdv), cos(theta_hdv));
if abs(theta_hdv_normalized) < pi/2 
    x_base = D1/2; x_tip = D1/2 + tri_height;
else
    x_base = -D1/2; x_tip = -D1/2 - tri_height;
end
tri_verts_unrotated_hdv = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
tri_verts_hdv = (rot_hdv * tri_verts_unrotated_hdv')' + [s_hdv, y_hdv];
patch(ax_sim, 'XData', tri_verts_hdv(:,1), 'YData', tri_verts_hdv(:,2), ...
      'FaceColor', 'k', 'FaceAlpha', current_alpha_hdv, 'EdgeColor', 'k',...
      'EdgeAlpha', current_alpha_hdv,'HandleVisibility', 'off');
      
% 2.6 Finalize Sim Plot
xlabel(ax_sim, '$s$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel(ax_sim, '$y$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
% title(ax_sim, sprintf('Simulation Snapshot at $t = %.2f$ s', current_time), 'Interpreter', 'latex', 'FontSize', 16);
axis(ax_sim, [x_lims, -y_boundary-1, y_boundary+1]);
% legend(ax_sim, 'show', 'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 14);
hold(ax_sim, 'off');

% Save handle
handles.fig_sim = fig_sim;
handles.ax_sim = ax_sim;


% 3.1 Setup figure 2 (ttc)
fig_ttc = figure('Name', 'TTC Time Series', ...
                 'Units', 'centimeters');
ax_ttc = axes('Parent', fig_ttc);
set(ax_ttc, axis_font_settings);
hold(ax_ttc, 'on');

% 3.2 Select title and matrix column
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

% 3.3 Data preparation
t_vec = (0:Nsim-1)' * dt; 
data_to_plot = ttcs(:, col_idx);

% 3.4 Plot past trajectory
plot(ax_ttc, t_vec, data_to_plot, '.', 'MarkerSize', 10, ...
     'MarkerEdgeColor', [0 0.4470 0.7410], ...
     'DisplayName', 'TTC History');

% 3.5 find current value
current_value = data_to_plot(frame_idx);
plot(ax_ttc, current_time, current_value, 'ro', 'MarkerFaceColor', 'r', ...
     'MarkerSize', 10, 'HandleVisibility', 'off');

% 3.6 Add text
text_str = sprintf('TTC = %.2f s', current_value);
text_offset = 0.02 * t_max; 
text(ax_ttc, current_time + text_offset, current_value, text_str, ...
     'HorizontalAlignment', 'left', ...
     'VerticalAlignment', 'bottom', ...
     'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');

% 3.7 Finalize TTC Plot
xlabel(ax_ttc, 'Simulation time $t$ [s]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel(ax_ttc, 'TTC [s]', 'Interpreter', 'latex', 'FontSize', 16);
% title(ax_ttc, plot_title, 'Interpreter', 'latex', 'FontSize', 16);
grid(ax_ttc, 'on');
box(ax_ttc, 'on');
% axis equal
xlim(ax_ttc, [0 t_max]);
ylim(ax_ttc, [0 20]); 

hold(ax_ttc, 'off');

% Save handle
handles.fig_ttc = fig_ttc;
handles.ax_ttc = ax_ttc;

ttc = current_value;
end