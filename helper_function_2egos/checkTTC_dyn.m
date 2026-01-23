function fig = checkTTC_dyn(ttcs, ev_states, dt, target_time, id_vehicle, simX, time_future)

    % Plot settings
    axis_font_settings = struct(...
        'FontSize', 14, ...
        'FontName', 'Times New Roman', ...
        'TickLabelInterpreter', 'latex');
    line_width_road = 2.0;
    line_width_prediction = 1.2;
    
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
    
    % TTC selection
    ids_sorted = sort(id_vehicle);
    if isequal(ids_sorted, [1 2])
        ttc = ttcs(frame_idx, 1);
    elseif isequal(ids_sorted, [1 3])
        ttc = ttcs(frame_idx, 2);
    elseif isequal(ids_sorted, [2 3])
        ttc = ttcs(frame_idx, 3);
    elseif isequal(ids_sorted, [0 1])
        ttc = ttcs(frame_idx, 4);
    elseif isequal(ids_sorted, [0 2])
        ttc = ttcs(frame_idx, 5);
    elseif isequal(ids_sorted, [0 3])
        ttc = ttcs(frame_idx, 6);
    else
        warning('Combinazione id_vehicles non riconosciuta.');
        ttc = 0; 
    end

    if ttc > 20
        ttc = 10;
    end
    
    % Data extraction 
    dt_future = time_future(2) - time_future(1);
    if dt_future <= 0, dt_future = 0.1; end 
    
    frame_state = round(ttc / dt_future);
    max_future_frames = size(ev_states.ev_states1, 2);
    frame_state = max(1, min(frame_state, max_future_frames)); 
    
    states1 = ev_states.ev_states1(:, frame_state, frame_idx);
    states2 = ev_states.ev_states2(:, frame_state, frame_idx);
    states3 = ev_states.ev_states3(:, frame_state, frame_idx);
    statesh = ev_states.ev_statesh(:, frame_state, frame_idx);
    
    % Plot setup
    fig = figure('Name', sprintf('Predicted Configuration at TTC = %.2f s', ttc), ...
                 'Units', 'centimeters');
    ax = axes('Parent', fig);
    set(ax, axis_font_settings);
    hold(ax, 'on');
    axis(ax, 'equal');
    box(ax, 'on');
    
    ego_colors = {'b', 'r', 'y'}; 
    hdv_color = [0.5 0.5 0.5]; 
    lane_width = 8;
       
    %5.1 Plot road boundaries
    all_s = [states1(1), states2(1), states3(1), statesh(1)];
    center_s = mean(all_s);
    range_view = 40;
    
    x_lims = [center_s - range_view, center_s + range_view];
    y_boundary = lane_width / 2;
    
    plot(ax, x_lims, [y_boundary, y_boundary], 'k-', 'LineWidth', line_width_road, 'HandleVisibility', 'off');
    plot(ax, x_lims, [-y_boundary, -y_boundary], 'k-', 'LineWidth', line_width_road, 'HandleVisibility', 'off');
    plot(ax, x_lims, [0, 0], 'k-.', 'LineWidth', line_width_prediction, 'HandleVisibility', 'off');
    
    % 5.2 Vehicle dimensions definition
    D1 = 4.3; D2 = 1.8;
    vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
    tri_height = 0.3;
    tri_base_width = D2 * 0.3;
    
    % 5.3 Plot Vheicle 1 
    s = states1(1);
    y = states1(2);
    theta = states1(3);
    rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    verts = (rot_matrix * vehicle_verts_unrotated')' + [s, y];
    patch(ax, 'XData', verts(:,1), 'YData', verts(:,2), ...
          'FaceColor', ego_colors{1}, 'EdgeColor', 'k', 'DisplayName', 'Ego1');
          
    theta_norm = atan2(sin(theta), cos(theta));
    if abs(theta_norm) < pi/2 
        x_base = D1/2; x_tip = D1/2 + tri_height;
    else
        x_base = -D1/2; x_tip = -D1/2 - tri_height;
    end
    tri_verts_unrot = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
    tri_verts = (rot_matrix * tri_verts_unrot')' + [s, y];
    patch(ax, 'XData', tri_verts(:,1), 'YData', tri_verts(:,2), ...
          'FaceColor', 'k', 'EdgeColor', 'k', 'HandleVisibility', 'off');
      
    % 5.4 Plot Vehicle 2 
    s_v2 = states2(1);
    y_v2 = states2(2);
    theta_v2 = states2(3);
    rot_v2 = [cos(theta_v2), -sin(theta_v2); sin(theta_v2), cos(theta_v2)];
    
    verts_v2 = (rot_v2 * vehicle_verts_unrotated')' + [s_v2, y_v2];
    patch(ax, 'XData', verts_v2(:,1), 'YData', verts_v2(:,2), ...
          'FaceColor', ego_colors{2}, 'EdgeColor', 'k', 'DisplayName', 'Ego2');
          
    theta_norm_v2 = atan2(sin(theta_v2), cos(theta_v2));
    if abs(theta_norm_v2) < pi/2 
        x_base = D1/2; x_tip = D1/2 + tri_height;
    else
        x_base = -D1/2; x_tip = -D1/2 - tri_height;
    end
    tri_verts_unrot_v2 = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
    tri_verts_v2 = (rot_v2 * tri_verts_unrot_v2')' + [s_v2, y_v2];
    patch(ax, 'XData', tri_verts_v2(:,1), 'YData', tri_verts_v2(:,2), ...
          'FaceColor', 'k', 'EdgeColor', 'k','HandleVisibility', 'off');

    % 5.5 Plot Vehicle 3 
    s_v3 = states3(1);
    y_v3 = states3(2);
    theta_v3 = states3(3);
    rot_v3 = [cos(theta_v3), -sin(theta_v3); sin(theta_v3), cos(theta_v3)];
    
    verts_v3 = (rot_v3 * vehicle_verts_unrotated')' + [s_v3, y_v3];
    patch(ax, 'XData', verts_v3(:,1), 'YData', verts_v3(:,2), ...
          'FaceColor', ego_colors{3}, 'EdgeColor', 'k', 'DisplayName', 'Ego3');
    
    theta_norm_v3 = atan2(sin(theta_v3), cos(theta_v3));
    if abs(theta_norm_v3) < pi/2 
        x_base = D1/2; x_tip = D1/2 + tri_height;
    else
        x_base = -D1/2; x_tip = -D1/2 - tri_height;
    end
    tri_verts_unrot_v3 = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
    tri_verts_v3 = (rot_v3 * tri_verts_unrot_v3')' + [s_v3, y_v3];
    patch(ax, 'XData', tri_verts_v3(:,1), 'YData', tri_verts_v3(:,2), ...
          'FaceColor', 'k', 'EdgeColor', 'k','HandleVisibility', 'off');

    % 5.6 Plot human vehicle
    s_vh = statesh(1);
    y_vh = statesh(2);
    theta_vh = statesh(3);
    rot_vh = [cos(theta_vh), -sin(theta_vh); sin(theta_vh), cos(theta_vh)];
    
    verts_vh = (rot_vh * vehicle_verts_unrotated')' + [s_vh, y_vh];
    patch(ax, 'XData', verts_vh(:,1), 'YData', verts_vh(:,2), ...
          'FaceColor', hdv_color, 'EdgeColor', 'k', 'DisplayName', 'Hum');
          
    theta_norm_vh = atan2(sin(theta_vh), cos(theta_vh));
    if abs(theta_norm_vh) < pi/2 
        x_base = D1/2; x_tip = D1/2 + tri_height;
    else
        x_base = -D1/2; x_tip = -D1/2 - tri_height;
    end
    tri_verts_unrot_vh = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
    tri_verts_vh = (rot_vh * tri_verts_unrot_vh')' + [s_vh, y_vh];
    patch(ax, 'XData', tri_verts_vh(:,1), 'YData', tri_verts_vh(:,2), ...
          'FaceColor', 'k', 'EdgeColor', 'k','HandleVisibility', 'off');

    % 6. Finalize plot settings
    xlabel(ax, '$s$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
    ylabel(ax, '$y$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
    axis(ax, [x_lims, -y_boundary-4, y_boundary+4]);   
    % legend(ax, 'show', 'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 14);
    hold(ax, 'off');
end