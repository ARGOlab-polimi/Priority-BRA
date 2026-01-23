function play_mpc_traj(simX, simX_hdv, ttc_mpc_data1, dt, lane_width)
% Video of mpc predicted trajectories over time 

    preds_egos = {ttc_mpc_data1.ev_states_mpc.ev_states1, ...
                  ttc_mpc_data1.ev_states_mpc.ev_states2, ...
                  ttc_mpc_data1.ev_states_mpc.ev_states3};
    pred_hdv   = ttc_mpc_data1.ev_states_mpc.ev_statesh;
    
    num_steps_future = 60; % since there are 60 steps evaluated by the mpc
    Nsim = size(simX{1}, 1);


    fig = figure('Name', 'Simulazione Real-Time', ...
                 'Units', 'centimeters', ...
                 'Position', [2 2 25 15], ... 
                 'Color', 'w', ...
                 'Visible', 'on'); 

    for k = 1:Nsim

        if ~isvalid(fig)
            disp('Finestra chiusa dall''utente. Animazione terminata.');
            break;
        end

        clf(fig); 

        ax = axes('Parent', fig);
        hold(ax, 'on');
        axis(ax, 'equal');
        box(ax, 'on');

        set(ax, 'FontSize', 12, 'FontName', 'Times New Roman', 'TickLabelInterpreter', 'latex');

        current_x = simX{1}(k, 1);
        x_lims = [current_x - 20, current_x + 30]; 
        y_boundary = lane_width / 2;
        
        plot(ax, [x_lims(1)-50 x_lims(2)+50], [y_boundary, y_boundary], 'k-', 'LineWidth', 2);
        plot(ax, [x_lims(1)-50 x_lims(2)+50], [-y_boundary, -y_boundary], 'k-', 'LineWidth', 2); 
        plot(ax, [x_lims(1)-50 x_lims(2)+50], [0, 0], 'k-.', 'LineWidth', 1.2);

        ego_colors = {'b', 'r', [0.9290 0.6940 0.1250]};
        hdv_color = [0.5 0.5 0.5];
        D1 = 4.3; D2 = 1.8;
        tri_height = 0.3; tri_base = D2 * 0.3;

        v_rect = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
        v_tri  = [D1/2, -tri_base/2; D1/2, tri_base/2; D1/2+tri_height, 0];

        % Plot Ego vehicles 
        for i = 1:length(simX)
            plot(ax, simX{i}(1:k, 1), simX{i}(1:k, 2), ...
                 'Color', ego_colors{i}, 'LineStyle', '--', 'LineWidth', 1.5);
            try
                pred_xy = preds_egos{i}(1:2, 1:num_steps_future, k);
                plot(ax, pred_xy(1,:), pred_xy(2,:), ...
                     'Color', ego_colors{i}, 'LineStyle', '-', 'LineWidth', 0.5, ...
                     'Marker', '.', 'MarkerSize', 3);
            catch
            end
            
            state = simX{i}(k, :);
            pos = state(1:2); theta = state(3);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            verts_rot = (R * v_rect')' + pos;
            patch(ax, 'XData', verts_rot(:,1), 'YData', verts_rot(:,2), ...
                  'FaceColor', ego_colors{i}, 'EdgeColor', 'k');
            
            tri_rot = (R * v_tri')' + pos;
            patch(ax, 'XData', tri_rot(:,1), 'YData', tri_rot(:,2), ...
                  'FaceColor', 'k', 'EdgeColor', 'k');
        end
        
        % Plot HDV 
        plot(ax, simX_hdv(1:k, 1), simX_hdv(1:k, 2), ...
             'Color', hdv_color, 'LineStyle', '--', 'LineWidth', 1.5);

        try
            pred_h_xy = pred_hdv(1:2, 1:num_steps_future, k);
            plot(ax, pred_h_xy(1,:), pred_h_xy(2,:), ...
                 'Color', hdv_color, 'LineStyle', '-', 'LineWidth', 0.5, ...
                 'Marker', '.', 'MarkerSize', 3);
        catch
        end

        state_h = simX_hdv(k, :);
        pos_h = state_h(1:2); theta_h = state_h(3);
        R_h = [cos(theta_h), -sin(theta_h); sin(theta_h), cos(theta_h)];
        
        verts_h_rot = (R_h * v_rect')' + pos_h;
        patch(ax, 'XData', verts_h_rot(:,1), 'YData', verts_h_rot(:,2), ...
              'FaceColor', hdv_color, 'EdgeColor', 'k');
          
        tri_h_rot = (R_h * v_tri')' + pos_h;
        patch(ax, 'XData', tri_h_rot(:,1), 'YData', tri_h_rot(:,2), ...
              'FaceColor', 'k', 'EdgeColor', 'k');

        % Frame finalization
        xlabel(ax, '$s$ [m]', 'Interpreter', 'latex');
        ylabel(ax, '$y$ [m]', 'Interpreter', 'latex');
        title(ax, sprintf('Time: %.2f s', (k-1)*dt), 'FontSize', 14);
        axis(ax, [x_lims, -y_boundary-3, y_boundary+3]);
        
        drawnow; 

    end
end