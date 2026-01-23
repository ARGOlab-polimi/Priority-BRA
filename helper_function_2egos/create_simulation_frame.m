function fig = create_simulation_frame(simX, simX_hdv, target_time, dt, lane_width)
% 1. Calculate the corresponding simulation index
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

% 2. Setup the figure 
fig = figure('Name', sprintf('Simulation at t = %.2f s', (frame_idx-1)*dt), ...
             'Units', 'centimeters', ...
             'Visible', 'on'); 
         
ax = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
box(ax, 'on'); 

% Set publication-ready fonts and interpreters 
set(ax, 'FontSize', 14, ... 
        'FontName', 'Times New Roman', ... 
        'TickLabelInterpreter', 'latex');

% 3. Plot Road Boundaries
x_lims = [simX{1}(frame_idx, 1) - 20, simX{1}(frame_idx, 1) + 20];
y_boundary = lane_width / 2;

% --- Plot continuous road boundaries and dotted center line 
plot(ax, [x_lims(1)-20 x_lims(2)+20] , [y_boundary, y_boundary], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
plot(ax, [x_lims(1)-20 x_lims(2)+20], [-y_boundary, -y_boundary], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off'); 
plot(ax, [x_lims(1)-20 x_lims(2)+20], [0, 0], 'k-.', 'LineWidth', 1.2, 'HandleVisibility', 'off');

% 4. Define Vehicle Properties
num_egos = length(simX);

ego_colors = {'b', 'r', 'y'}; 
hdv_color = [0.5 0.5 0.5]; 

D1 = 4.3; 
D2 = 1.8; 
vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];

tri_height = 0.3; 
tri_base_width = D2 * 0.3;

% 5. Plot ego vehicles and their trajectories
for i_ego = [1 2 3]   % to change the order of the plotting of the lines
    plot(ax, simX{i_ego}(1:frame_idx, 1), simX{i_ego}(1:frame_idx, 2), ...
         'Color', ego_colors{i_ego}, 'LineStyle', '--', 'LineWidth', 1.5, 'HandleVisibility','off');
end

% Plot HDV trajectory 
plot(ax, simX_hdv(1:frame_idx, 1), simX_hdv(1:frame_idx, 2), ...
     'Color', hdv_color, 'LineStyle', '--', 'LineWidth', 1.5, 'HandleVisibility','off');

% Plot Ego vehicle bodies and direction triangles
for i_ego = 1:num_egos
    state = simX{i_ego}(frame_idx, :); 
    s = state(1);
    y = state(2);
    theta = state(3);
    
    rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    verts = (rot_matrix * vehicle_verts_unrotated')' + [s, y];
    
    patch(ax, 'XData', verts(:,1), 'YData', verts(:,2), ...
          'FaceColor', ego_colors{i_ego}, 'EdgeColor', 'k', ...
          'DisplayName', ['Ego ' num2str(i_ego)]);
          
    x_base = D1/2; 
    x_tip = D1/2 + tri_height; 
    tri_verts_unrotated = [x_base, -tri_base_width/2;
                           x_base,  tri_base_width/2;
                           x_tip,   0];                  
    tri_verts = (rot_matrix * tri_verts_unrotated')' + [s, y];

    patch(ax, 'XData', tri_verts(:,1), 'YData', tri_verts(:,2), ...
          'FaceColor', 'k', 'EdgeColor', 'k', 'HandleVisibility', 'off');
end

% 6. Plot HDV
state_hdv = simX_hdv(frame_idx, :); 
s_hdv = state_hdv(1);
y_hdv = state_hdv(2);
theta_hdv = state_hdv(3);
rot_hdv = [cos(theta_hdv), -sin(theta_hdv); sin(theta_hdv), cos(theta_hdv)];
verts_hdv = (rot_hdv * vehicle_verts_unrotated')' + [s_hdv, y_hdv];
patch(ax, 'XData', verts_hdv(:,1), 'YData', verts_hdv(:,2), ...
      'FaceColor', hdv_color, 'EdgeColor', 'k', 'DisplayName', 'HDV');
      
x_base = D1/2;
x_tip = D1/2 + tri_height;
tri_verts_unrotated_hdv = [x_base, -tri_base_width/2;
                           x_base,  tri_base_width/2;
                           x_tip,   0];
tri_verts_hdv = (rot_hdv * tri_verts_unrotated_hdv')' + [s_hdv, y_hdv];

patch(ax, 'XData', tri_verts_hdv(:,1), 'YData', tri_verts_hdv(:,2), ...
      'FaceColor', 'k', 'EdgeColor', 'k', 'HandleVisibility', 'off');

% 7. Finalize plot settings
xlabel(ax, '$s$ [m]', 'FontSize', 16, 'Interpreter', 'latex');
ylabel(ax, '$y$ [m]', 'FontSize', 16, 'Interpreter', 'latex');

% Set axis limits
axis(ax, [x_lims, -y_boundary-1, y_boundary+1]);

% legend(ax, 'show', 'Location', 'northeast', 'FontSize', 14, 'Interpreter', 'latex', 'Box', 'on');

% set(ax, 'LooseInset', get(ax, 'TightInset'));
hold(ax, 'off');
end