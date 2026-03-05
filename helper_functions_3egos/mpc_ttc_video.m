function mpc_ttc_video(simX, simX_hdv, ttcs, ttc_mpc_data1, id_vehicles, dt, video_filename, save_video)
    num_egos = 3;
    lane_width = 8;
    
    preds_egos = {ttc_mpc_data1.ev_states_mpc.ev_states1, ...
                  ttc_mpc_data1.ev_states_mpc.ev_states2, ...
                  ttc_mpc_data1.ev_states_mpc.ev_states3};
    pred_hdv   = ttc_mpc_data1.ev_states_mpc.ev_statesh;
    num_steps_future = 60;
    
    pair = sort(id_vehicles); 
    ttc_col_idx = 1; 
    
    if isequal(pair, [1 2]), ttc_col_idx = 1; 
    elseif isequal(pair, [1 3]), ttc_col_idx = 2; 
    elseif isequal(pair, [2 3]), ttc_col_idx = 3; 
    elseif isequal(pair, [0 1]), ttc_col_idx = 4; 
    elseif isequal(pair, [0 2]), ttc_col_idx = 5; 
    elseif isequal(pair, [0 3]), ttc_col_idx = 6; 
    end

    writerObj = [];
    if save_video == 1
        writerObj = VideoWriter(video_filename, 'MPEG-4');
        writerObj.FrameRate = 1 / dt; 
        writerObj.Quality = 100;
        open(writerObj);
    end

    ss = get(0, 'ScreenSize'); 
    screen_w = ss(3);
    screen_h = ss(4);
    
    fig_w = 1080;
    fig_h = 700; 
    
    pos_x = (screen_w - fig_w) / 2;
    pos_y = (screen_h - fig_h) / 2;

    fig = figure('Name', 'MPC Trajectory & TTC Analysis', 'NumberTitle', 'off', ...
                 'Units', 'pixels', 'Position', [pos_x pos_y fig_w fig_h], ... 
                 'Color', 'w', 'Visible', 'on'); 
    
    movegui(fig, 'center'); 
    
    ax2 = subplot(2, 1, 2);
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    set(ax2, 'FontSize', 15);
    Nsim = size(ttcs, 1);
    t_total = Nsim * dt;
    xlim(ax2, [0, t_total]);
    ylim(ax2, [0, 20]); 
    xlabel(ax2, 'Simulation Time [s]', 'FontSize', 18);
    ylabel(ax2, 'TTC [s]', 'FontSize', 18);
    ttc_line = animatedline(ax2, 'Color', 'r', 'LineWidth', 2, 'Marker', '.', 'MarkerSize', 18);
    yline(ax2, 0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');
    
    ego_colors = {'b', 'r', [0.9290 0.6940 0.1250]}; 
    hdv_color = [0.5 0.5 0.5];
    D1 = 4.3; D2 = 1.8;
    v_rect = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
    tri_height = 0.3; tri_base = D2 * 0.3;
    v_tri  = [D1/2, -tri_base/2; D1/2, tri_base/2; D1/2+tri_height, 0];
    
    t_array = linspace(0.0, t_total, Nsim);
    
    for k = 1:Nsim
        if ~isvalid(fig), break; end
        
        ax1 = subplot(2, 1, 1);
        cla(ax1);
        hold(ax1, 'on'); axis(ax1, 'equal'); box(ax1, 'on');
        set(ax1, 'FontSize', 12, 'FontName', 'Times New Roman', 'TickLabelInterpreter', 'latex');
        
        current_x = simX{1}(k, 1);
        x_lims = [current_x - 20, current_x + 30];
        y_boundary = lane_width / 2;
        
        plot(ax1, [x_lims(1)-50 x_lims(2)+50], [y_boundary, y_boundary], 'k-', 'LineWidth', 2);
        plot(ax1, [x_lims(1)-50 x_lims(2)+50], [-y_boundary, -y_boundary], 'k-', 'LineWidth', 2); 
        plot(ax1, [x_lims(1)-50 x_lims(2)+50], [0, 0], 'k-.', 'LineWidth', 1.2);
        
        for i = 1:num_egos
            plot(ax1, simX{i}(1:k, 1), simX{i}(1:k, 2), ...
                 'Color', ego_colors{i}, 'LineStyle', '--', 'LineWidth', 1.5);
            try
                pred_xy = preds_egos{i}(1:2, 1:num_steps_future, k);
                plot(ax1, pred_xy(1,:), pred_xy(2,:), ...
                     'Color', ego_colors{i}, 'LineStyle', '-', 'LineWidth', 0.5, ...
                     'Marker', '.', 'MarkerSize', 3);
            catch
            end
            
            state = simX{i}(k, :);
            pos = state(1:2); theta = state(3);
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            verts_rot = (R * v_rect')' + pos;
            patch(ax1, 'XData', verts_rot(:,1), 'YData', verts_rot(:,2), ...
                  'FaceColor', ego_colors{i}, 'EdgeColor', 'k');
            
            tri_rot = (R * v_tri')' + pos;
            patch(ax1, 'XData', tri_rot(:,1), 'YData', tri_rot(:,2), ...
                  'FaceColor', 'k', 'EdgeColor', 'k');
        end
        
        plot(ax1, simX_hdv(1:k, 1), simX_hdv(1:k, 2), ...
             'Color', hdv_color, 'LineStyle', '--', 'LineWidth', 1.5);
        try
            pred_h_xy = pred_hdv(1:2, 1:num_steps_future, k);
            plot(ax1, pred_h_xy(1,:), pred_h_xy(2,:), ...
                 'Color', hdv_color, 'LineStyle', '-', 'LineWidth', 0.5, ...
                 'Marker', '.', 'MarkerSize', 3);
        catch
        end
        state_h = simX_hdv(k, :);
        pos_h = state_h(1:2); theta_h = state_h(3);
        R_h = [cos(theta_h), -sin(theta_h); sin(theta_h), cos(theta_h)];
        verts_h_rot = (R_h * v_rect')' + pos_h;
        patch(ax1, 'XData', verts_h_rot(:,1), 'YData', verts_h_rot(:,2), ...
              'FaceColor', hdv_color, 'EdgeColor', 'k');
        tri_h_rot = (R_h * v_tri')' + pos_h;
        patch(ax1, 'XData', tri_h_rot(:,1), 'YData', tri_h_rot(:,2), ...
              'FaceColor', 'k', 'EdgeColor', 'k');
          
        xlabel(ax1, '$s$ [m]', 'Interpreter', 'latex');
        ylabel(ax1, '$y$ [m]', 'Interpreter', 'latex');
        title(ax1, sprintf('Time: %.2f s', (k-1)*dt), 'FontSize', 14);
        axis(ax1, [x_lims, -y_boundary-3, y_boundary+3]);

        current_ttc = ttcs(k, ttc_col_idx);
        addpoints(ttc_line, t_array(k), current_ttc);
        
        drawnow limitrate;
        
        if save_video == 1
            frame = getframe(fig);
            writeVideo(writerObj, frame);
            if mod(k, 50) == 0
                fprintf('Rendering frame %d / %d\n', k, Nsim);
            end
        else
            pause(dt*0.4);
        end
    end
    
    if save_video == 1
        close(writerObj);
        fprintf('Video salvato: %s\n', video_filename);
        close(fig);
    end
end