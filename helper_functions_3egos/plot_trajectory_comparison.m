function fig = plot_trajectory_comparison(simX, simX_hum, ev_states_mpc, id_vehicle, target_time)
    dt = 0.05;
    lane_width = 8;
    
    axis_font_settings = struct(...
        'FontSize', 14, ...
        'FontName', 'Times New Roman', ...
        'TickLabelInterpreter', 'latex');
    
    Nsim = size(simX{1}, 1);
    t_max = (Nsim - 1) * dt;
    
    if target_time > t_max
        target_time = t_max;
        warning('Target time oltre la durata della simulazione. Uso t=%.2f s.', t_max);
    elseif target_time < 0
        target_time = 0;
        warning('Target time negativo. Uso t=0 s.');
    end
    
    frame_idx = round(target_time / dt) + 1;
    frame_idx = max(1, min(frame_idx, Nsim));
    
    fig = figure('Name', sprintf('Trajectory Comparison ID %d vs %d', id_vehicle(1), id_vehicle(2)), ...
                 'Units', 'centimeters');
    ax = axes('Parent', fig);
    set(ax, axis_font_settings);
    hold(ax, 'on');
    axis(ax, 'equal');
    box(ax, 'on');
    
    plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', sprintf('Propagated\ntrajectory'));
    plot(ax, NaN, NaN, 'b--', 'LineWidth', 2.0, 'DisplayName', sprintf('Predicted\ntrajectory'));
    
    x_centers = zeros(1, length(id_vehicle));
    for i = 1:length(id_vehicle)
        id = id_vehicle(i);
        [state, ~] = get_kinematic_state(id, simX, simX_hum, frame_idx);
        x_centers(i) = state(1);
    end
    
    mean_s = mean(x_centers);
    x_lims = [mean_s - 8, mean_s + 25];
    y_boundary = lane_width / 2;
    
    plot(ax, x_lims, [y_boundary, y_boundary], 'k-', 'LineWidth', 2.0, 'HandleVisibility', 'off');
    plot(ax, x_lims, [-y_boundary, -y_boundary], 'k-', 'LineWidth', 2.0, 'HandleVisibility', 'off');
    plot(ax, x_lims, [0, 0], 'k-.', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    
    D1 = 4.3; D2 = 1.8;
    vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
    tri_height = 0.3;
    tri_base_width = D2 * 0.3;
    
    for i = 1:length(id_vehicle)
        id = id_vehicle(i);
        [state, ~] = get_kinematic_state(id, simX, simX_hum, frame_idx);
        s0 = state(1);
        y0 = state(2);
        th0 = state(3);
        
        c = get_v_color(id);
        
        t_vec = linspace(0, 3.0, round(3.0 / dt) + 1);
        get_x = @(s, t) s(1) + s(4) * cos(s(3)) * t;
        get_y = @(s, t) tan(s(3)) * (get_x(s, t) - s(1)) + s(2);
        
        x_kin = get_x(state, t_vec);
        y_kin = get_y(state, t_vec);
        
        plot(ax, x_kin, y_kin, '-', 'Color', c, 'LineWidth', 1.5, ...
             'HandleVisibility', 'off');
             
        if id ~= 0
            states_mpc = get_vehicle_data_mpc(id, ev_states_mpc);
            x_mpc = squeeze(states_mpc(1, 1:60, frame_idx));
            y_mpc = squeeze(states_mpc(2, 1:60, frame_idx));
            
            plot(ax, x_mpc, y_mpc, '--', 'Color', c, 'LineWidth', 2.0, ...
                 'HandleVisibility', 'off');
        end
        
        rot_matrix = [cos(th0), -sin(th0); sin(th0), cos(th0)];
        verts = (rot_matrix * vehicle_verts_unrotated')' + [s0, y0];
        patch(ax, 'XData', verts(:,1), 'YData', verts(:,2), ...
              'FaceColor', c, 'EdgeColor', 'k', 'HandleVisibility', 'off');
              
        theta_normalized = atan2(sin(th0), cos(th0));
        if abs(theta_normalized) < pi/2 
            x_base = D1/2; x_tip = D1/2 + tri_height;
        else
            x_base = -D1/2; x_tip = -D1/2 - tri_height;
        end
        tri_verts_unrotated = [x_base, -tri_base_width/2; x_base, tri_base_width/2; x_tip, 0];
        tri_verts = (rot_matrix * tri_verts_unrotated')' + [s0, y0];
        patch(ax, 'XData', tri_verts(:,1), 'YData', tri_verts(:,2), ...
              'FaceColor', 'k', 'EdgeColor', 'k', 'HandleVisibility', 'off');
    end
    
    axis(ax, [x_lims, -y_boundary-1, y_boundary+1]);
    
    xlabel(ax, '$s$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
    ylabel(ax, '$y$ [m]', 'Interpreter', 'latex', 'FontSize', 16);
    
    lgd = legend(ax, 'show');
    set(lgd, 'Location', 'eastoutside', 'Interpreter', 'tex', 'FontSize', 14);
    
    hold(ax, 'off');
end

function [state, dims] = get_kinematic_state(id, simX, simX_hum, idx)
    dims = [4.3, 1.8];
    if id == 0
        x0 = simX_hum(idx, 1);
        y0 = simX_hum(idx, 2);
        th = simX_hum(idx, 3);
        v  = simX_hum(idx, 4);
    else
        x0 = simX{id}(idx, 1);
        y0 = simX{id}(idx, 2);
        th = simX{id}(idx, 3);
        v  = simX{id}(idx, 5);
    end
    state = [x0, y0, th, v];
end

function states = get_vehicle_data_mpc(id, ev_states_mpc)
    switch id
        case 1, states = ev_states_mpc.ev_states1;
        case 2, states = ev_states_mpc.ev_states2;
        case 3, states = ev_states_mpc.ev_states3;
        otherwise, error('ID %d non valido per estrazione MPC', id);
    end
end

function c = get_v_color(id)
    if id == 0
        c = [0.5 0.5 0.5];
    elseif id == 1
        c = 'b';
    elseif id == 2
        c = 'r';
    elseif id == 3
        c = 'y';
    else
        c = 'k';
    end
end