function save_ttc_simulation_video(simX, simX_hdv, ttcs, id_vehicles, num_egos, lane_width, dt, video_filename)

    fprintf('Inizio rendering video TTC (Real-Time): %s ...\n', video_filename);

    % TTC identification
    pair = sort(id_vehicles); 
    
    ttc_col_idx = [];
    
    if isequal(pair, [1 2]), ttc_col_idx = 1; 
    elseif isequal(pair, [1 3]), ttc_col_idx = 2; 
    elseif isequal(pair, [2 3]), ttc_col_idx = 3; 
    elseif isequal(pair, [0 1]), ttc_col_idx = 4; 
    elseif isequal(pair, [0 2]), ttc_col_idx = 5; 
    elseif isequal(pair, [0 3]), ttc_col_idx = 6; 
    else
        warning('Coppia veicoli non riconosciuta per TTC. Uso colonna 1 di default.');
        ttc_col_idx = 1; 
    end

    writerObj = VideoWriter(video_filename, 'MPEG-4');
    writerObj.FrameRate = 1 / dt; % Real-time
    writerObj.Quality = 100;
    open(writerObj);

    fig = figure('Name', 'TTC Analysis Video', 'NumberTitle', 'off', ...
                 'Units', 'pixels', 'Position', [100 50 1080 1350], ... 
                 'Color', 'w', 'Visible', 'on'); 

    
    ax1 = subplot(2, 1, 1);
    hold(ax1, 'on');
    grid(ax1, 'off');
    axis(ax1, 'equal');
    xlabel(ax1, 'Longitudinal Position [m]', 'FontSize', 18);
    ylabel(ax1, 'Lateral Position [m]', 'FontSize', 18);
    set(ax1, 'FontSize', 15);

    road_length = 10000; road_start = -100;
    line(ax1, [road_start, road_length], [lane_width/2, lane_width/2], 'Color', 'k', 'LineWidth', 4, 'HandleVisibility', 'off');
    line(ax1, [road_start, road_length], [-lane_width/2, -lane_width/2], 'Color', 'k', 'LineWidth', 4, 'HandleVisibility', 'off');

    stripe_len = 1.5; gap_len = 2.5;
    dash_starts = road_start : (stripe_len + gap_len) : road_length;
    dash_ends = dash_starts + stripe_len;
    X_dashes = [dash_starts; dash_ends; NaN(size(dash_starts))];
    line(ax1, X_dashes(:), zeros(size(X_dashes(:))), 'Color', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');

    D1 = 4.3; D2 = 1.8;
    vehicle_verts = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];
    colors = {'b', 'r', 'y'}; 

    ego_patches = cell(1, num_egos);
    for i = 1:num_egos
        col = colors{mod(i-1, length(colors)) + 1};
        
        if ismember(i, id_vehicles)
            alpha_val = 1;     
            edge_col = 'k';
            l_width = 1.5;
        else
            alpha_val = 0.2;    
            edge_col = 'none'; 
            l_width = 0.5;
        end
        
        ego_patches{i} = patch(ax1, 'XData', [], 'YData', [], ...
                               'FaceColor', col, 'FaceAlpha', alpha_val, ...
                               'EdgeColor', edge_col, 'LineWidth', l_width, ...
                               'DisplayName', ['AV ' num2str(i)]);
    end

    if ismember(0, id_vehicles)
        alpha_val_h = 1;
        edge_col_h = 'k';
        l_width_h = 1.5;
    else
        alpha_val_h = 0.2;
        edge_col_h = 'none';
        l_width_h = 0.5;
    end
    hdv_patch = patch(ax1, 'XData', [], 'YData', [], ...
                      'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', alpha_val_h, ...
                      'EdgeColor', edge_col_h, 'LineWidth', l_width_h, ...
                      'DisplayName', 'HPV');

    legend(ax1, 'show', 'Location', 'northeast', 'FontSize', 15);

    ax2 = subplot(2, 1, 2);
    hold(ax2, 'on');
    grid(ax2, 'on');
    box(ax2, 'on');
    set(ax2, 'FontSize', 15);

    Nsim = size(ttcs, 1);
    t_total = Nsim * dt;
    xlim(ax2, [0, t_total]);
    
    ylim(ax2, [0, 20]); 
    
    xlabel(ax2, 'Simulation Time [s]', 'FontSize', 18);
    ylabel(ax2, 'TTC [s]', 'FontSize', 18);
    
    ttc_line = animatedline(ax2, 'Color', 'r', 'LineWidth', 2, ...
        'Marker', '.', 'MarkerSize', 18);
  
    yline(ax2, 0, 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');


    t_array = linspace(0.0, t_total, Nsim);
    
    for k = 1:Nsim
        for ego_idx = 1:num_egos
            s = simX{ego_idx}(k, 1); y = simX{ego_idx}(k, 2); theta = simX{ego_idx}(k, 3);
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            verts = (rot * vehicle_verts')' + [s, y];
            set(ego_patches{ego_idx}, 'XData', verts(:,1), 'YData', verts(:,2));
        end
        % HPV
        s_h = simX_hdv(k, 1); y_h = simX_hdv(k, 2); theta_h = simX_hdv(k, 3);
        rot_h = [cos(theta_h), -sin(theta_h); sin(theta_h), cos(theta_h)];
        verts_h = (rot_h * vehicle_verts')' + [s_h, y_h];
        set(hdv_patch, 'XData', verts_h(:,1), 'YData', verts_h(:,2));

        target_id = id_vehicles(1);
        if target_id == 0
            cam_s = s_h;
        else
            cam_s = simX{target_id}(k, 1);
        end
        xlim(ax1, [cam_s - 20, cam_s + 20]);
        ylim(ax1, [-5, 5]);
        title(ax1, sprintf('Simulation Time: %.2f s', t_array(k)), 'FontSize', 18);

        current_ttc = ttcs(k, ttc_col_idx);
        addpoints(ttc_line, t_array(k), current_ttc);

        drawnow limitrate;
        frame = getframe(fig);
        writeVideo(writerObj, frame);
        
        if mod(k, 50) == 0
            fprintf('Rendering frame %d / %d\n', k, Nsim);
        end
    end

    close(writerObj);
    close(fig);
    fprintf('Video TTC salvato: %s\n', video_filename);
end