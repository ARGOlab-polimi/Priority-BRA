function fig = check_ttc(simX, simX_hdv, target_time, dt, lane_width, id_vehicles, ttc)

% Standard settings for pubblication
axis_font_settings = struct(...
    'FontSize', 14, ...
    'FontName', 'Times New Roman', ...
    'TickLabelInterpreter', 'latex');
line_width_road = 2.0;
line_width_prediction = 1.2;


% Compute time and index

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


% Initial states extraction
s1_0 = simX{1}(frame_idx, 1); 
y1_0 = simX{1}(frame_idx, 2); 
theta1 = simX{1}(frame_idx, 3); 
v1 = simX{1}(frame_idx, 5);

s2_0 = simX{2}(frame_idx, 1); 
y2_0 = simX{2}(frame_idx, 2); 
theta2 = simX{2}(frame_idx, 3); 
v2 = simX{2}(frame_idx, 5);

s3_0 = simX{3}(frame_idx, 1); 
y3_0 = simX{3}(frame_idx, 2); 
theta3 = simX{3}(frame_idx, 3); 
v3 = simX{3}(frame_idx, 5);

sh_0 = simX_hdv(frame_idx, 1); 
yh_0 = simX_hdv(frame_idx, 2); 
thetah = simX_hdv(frame_idx, 3); 
vh = simX_hdv(frame_idx, 4);

% States propagation
x1_end = s1_0 + v1 * cos(theta1) * ttc; 
y1_end = y1_0 + v1 * sin(theta1) * ttc;
x2_end = s2_0 + v2 * cos(theta2) * ttc; 
y2_end = y2_0 + v2 * sin(theta2) * ttc;
x3_end = s3_0 + v3 * cos(theta3) * ttc; 
y3_end = y3_0 + v3 * sin(theta3) * ttc;
xh_end = sh_0 + vh * cos(thetah) * ttc; 
yh_end = yh_0 + vh * sin(thetah) * ttc;

% Trajectories creation
x1_series = [s1_0, x1_end]; 
y1_series = [y1_0, y1_end];
x2_series = [s2_0, x2_end]; 
y2_series = [y2_0, y2_end];
x3_series = [s3_0, x3_end]; 
y3_series = [y3_0, y3_end];
xh_series = [sh_0, xh_end]; 
yh_series = [yh_0, yh_end];

% Plot settings
fig = figure('Name', sprintf('Predicted Configuration at TTC = %.2f s', ttc), ...
             'Units', 'centimeters');
ax = axes('Parent', fig);
set(ax, axis_font_settings);
hold(ax, 'on');
axis(ax, 'equal');
box(ax, 'on');

ego_colors = {'b', 'r', 'y'}; 
hdv_color = [0.5 0.5 0.5]; 

% Data selection
ids_sorted = sort(id_vehicles);
if isequal(ids_sorted, [1 2])
    x_v1 = x1_series; y_v1 = y1_series; theta_v1 = theta1; color_v1 = ego_colors{1}; name_v1 = 'Ego 1';
    x_v2 = x2_series; y_v2 = y2_series; theta_v2 = theta2; color_v2 = ego_colors{2}; name_v2 = 'Ego 2';
elseif isequal(ids_sorted, [1 3])
    x_v1 = x1_series; y_v1 = y1_series; theta_v1 = theta1; color_v1 = ego_colors{1}; name_v1 = 'Ego 1';
    x_v2 = x3_series; y_v2 = y3_series; theta_v2 = theta3; color_v2 = ego_colors{3}; name_v2 = 'Ego 3';
elseif isequal(ids_sorted, [2 3])
    x_v1 = x2_series; y_v1 = y2_series; theta_v1 = theta2; color_v1 = ego_colors{2}; name_v1 = 'Ego 2';
    x_v2 = x3_series; y_v2 = y3_series; theta_v2 = theta3; color_v2 = ego_colors{3}; name_v2 = 'Ego 3';
elseif isequal(ids_sorted, [0 1])
    x_v1 = x1_series; y_v1 = y1_series; theta_v1 = theta1; color_v1 = ego_colors{1}; name_v1 = 'Ego 1';
    x_v2 = xh_series; y_v2 = yh_series; theta_v2 = thetah; color_v2 = hdv_color; name_v2 = 'HDV';
elseif isequal(ids_sorted, [0 2])
    x_v1 = x2_series; y_v1 = y2_series; theta_v1 = theta2; color_v1 = ego_colors{2}; name_v1 = 'Ego 2';
    x_v2 = xh_series; y_v2 = yh_series; theta_v2 = thetah; color_v2 = hdv_color; name_v2 = 'HDV';
elseif isequal(ids_sorted, [0 3])
    x_v1 = x3_series; y_v1 = y3_series; theta_v1 = theta3; color_v1 = ego_colors{3}; name_v1 = 'Ego 3';
    x_v2 = xh_series; y_v2 = yh_series; theta_v2 = thetah; color_v2 = hdv_color; name_v2 = 'HDV';
else
    warning('Combinazione id_vehicles non riconosciuta.');
    return;
end


% 5.1 Plot Road Boundaries
x_lims = [mean([x_v1(2), x_v2(2)]) - 20, mean([x_v1(2), x_v2(2)]) + 20];
y_boundary = lane_width / 2;
plot(ax, x_lims, [y_boundary, y_boundary], 'k-', 'LineWidth', line_width_road, 'HandleVisibility', 'off');
plot(ax, x_lims, [-y_boundary, -y_boundary], 'k-', 'LineWidth', line_width_road, 'HandleVisibility', 'off');
plot(ax, x_lims, [0, 0], 'k-.', 'LineWidth', line_width_prediction, 'HandleVisibility', 'off');

% 5.2 Vehicle definition
D1 = 4.3; D2 = 1.8;
vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
tri_height = 0.3;
tri_base_width = D2 * 0.3;

% 5.3 Plot Vehicle 1 
plot(ax, x_v1, y_v1, ...
     'Color', color_v1, 'LineStyle', '--', 'LineWidth', line_width_prediction, 'HandleVisibility','off');
s = x_v1(end);
y = y_v1(end);
theta = theta_v1;
rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
verts = (rot_matrix * vehicle_verts_unrotated')' + [s, y];
patch(ax, 'XData', verts(:,1), 'YData', verts(:,2), ...
      'FaceColor', color_v1, 'EdgeColor', 'k', 'DisplayName', name_v1);
theta_normalized = atan2(sin(theta), cos(theta));
if abs(theta_normalized) < pi/2 
    x_base = D1/2; x_tip = D1/2 + tri_height;
else
    x_base = -D1/2; x_tip = -D1/2 - tri_height;
end
tri_verts_unrotated = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
tri_verts = (rot_matrix * tri_verts_unrotated')' + [s, y];
patch(ax, 'XData', tri_verts(:,1), 'YData', tri_verts(:,2), ...
      'FaceColor', 'k', 'EdgeColor', 'k', 'HandleVisibility', 'off');
      
% 5.4 Plot Vehicle 2 ---
plot(ax, x_v2, y_v2, ...
     'Color', color_v2, 'LineStyle', '--', 'LineWidth', line_width_prediction, 'HandleVisibility','off');
s_v2 = x_v2(end);
y_v2 = y_v2(end);
theta_v2_val = theta_v2;
rot_v2 = [cos(theta_v2_val), -sin(theta_v2_val); sin(theta_v2_val), cos(theta_v2_val)];
verts_v2 = (rot_v2 * vehicle_verts_unrotated')' + [s_v2, y_v2];
patch(ax, 'XData', verts_v2(:,1), 'YData', verts_v2(:,2), ...
      'FaceColor', color_v2, 'EdgeColor', 'k', 'DisplayName', name_v2);
theta_normalized = atan2(sin(theta_v2_val), cos(theta_v2_val));
if abs(theta_normalized) < pi/2 
    x_base = D1/2; x_tip = D1/2 + tri_height;
else
    x_base = -D1/2; x_tip = -D1/2 - tri_height;
end
tri_verts_unrotated_v2 = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
tri_verts_v2 = (rot_v2 * tri_verts_unrotated_v2')' + [s_v2, y_v2];
patch(ax, 'XData', tri_verts_v2(:,1), 'YData', tri_verts_v2(:,2), ...
      'FaceColor', 'k', 'EdgeColor', 'k','HandleVisibility', 'off');

% 6 Final plot settings
xlabel(ax, '$s$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
ylabel(ax, '$y$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
% title(ax, sprintf('Predicted Configuration at TTC = %.2f s', ttc), 'Interpreter', 'latex', 'FontSize', 16);
axis(ax, [x_lims, -y_boundary-1, y_boundary+1]);
% legend(ax, 'show', 'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 14);
hold(ax, 'off');
end