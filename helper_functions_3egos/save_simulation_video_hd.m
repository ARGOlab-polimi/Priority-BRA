function save_simulation_video_hd(simX, simX_hdv, num_egos, lane_width, dt, video_filename)
% SAVE_SIMULATION_VIDEO_HD 

    fprintf('Inizio rendering video HD (Real-Time): %s ...\n', video_filename);

    writerObj = VideoWriter(video_filename, 'MPEG-4');

    real_time_fps = 1 / dt; 
    writerObj.FrameRate = real_time_fps; 
    writerObj.Quality = 100;
    open(writerObj);

    fig = figure('Name', 'Simulation HD', 'NumberTitle', 'off', ...
                 'Units', 'pixels', 'Position', [100 100 1920 1080], ...
                 'Color', 'w', 'Visible', 'on'); 
    
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'off'); 
    axis(ax, 'equal');
    
    xlabel(ax, 'Longitudinal Position [m]', 'FontSize', 16);
    ylabel(ax, 'Lateral Position [m]', 'FontSize', 16);
    set(ax, 'FontSize', 14);

    road_length = 10000; 
    road_start = -100;
    
    line(ax, [road_start, road_length], [lane_width/2, lane_width/2], ...
        'Color', 'k', 'LineWidth', 4, 'HandleVisibility', 'off');
    line(ax, [road_start, road_length], [-lane_width/2, -lane_width/2], ...
        'Color', 'k', 'LineWidth', 4, 'HandleVisibility', 'off');

    stripe_len = 1; 
    gap_len = 1;    
    
    dash_starts = road_start : (stripe_len + gap_len) : road_length;
    dash_ends = dash_starts + stripe_len;
    
    X_dashes = [dash_starts; dash_ends; NaN(size(dash_starts))];
    X_dashes = X_dashes(:); 
    Y_dashes = zeros(size(X_dashes)); 
    
    line(ax, X_dashes, Y_dashes, ...
        'Color', 'k', 'LineWidth', 3, 'HandleVisibility', 'off');

    D1 = 4.3; 
    D2 = 1.8; 
    vehicle_verts_unrotated = [D1/2, -D2/2; -D1/2, -D2/2; -D1/2, D2/2; D1/2, D2/2];

    colors = {'b', 'r', 'y'}; 

    % AVs (Autonomous Vehicles)
    ego_patches = cell(1, num_egos);
    for i = 1:num_egos
        col = colors{mod(i-1, length(colors)) + 1};
        ego_patches{i} = patch(ax, 'XData', [], 'YData', [], ...
                               'FaceColor', col, ...
                               'FaceAlpha', 1, ...     % Opaco
                               'EdgeColor', 'k', ...
                               'LineWidth', 1.5, ...
                               'DisplayName', ['AV ' num2str(i)]);
    end
    
    % HPV (Human Piloted Vehicle)
    hdv_patch = patch(ax, 'XData', [], 'YData', [], ...
                      'FaceColor', [0.5 0.5 0.5], ...
                      'FaceAlpha', 1, ...     
                      'EdgeColor', 'k', ...
                      'LineWidth', 1.5, ...
                      'DisplayName', 'HPV');

    lgd = legend(ax, 'show', 'Location', 'northeast');
    set(lgd, 'FontSize', 20); 

    Nsim = size(simX{1}, 1);
    t_array = linspace(0.0, Nsim * dt, Nsim);

    for k = 1:Nsim
        % A. update AVs
        for ego_idx = 1:num_egos
            s = simX{ego_idx}(k, 1);
            y = simX{ego_idx}(k, 2);
            theta = simX{ego_idx}(k, 3);
            
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            verts = (rot * vehicle_verts_unrotated')' + [s, y];
            
            set(ego_patches{ego_idx}, 'XData', verts(:,1), 'YData', verts(:,2));
        end

        % B. update HPV
        s_hdv = simX_hdv(k, 1);
        y_hdv = simX_hdv(k, 2);
        theta_hdv = simX_hdv(k, 3);
        
        rot_hdv = [cos(theta_hdv), -sin(theta_hdv); sin(theta_hdv), cos(theta_hdv)];
        verts_hdv = (rot_hdv * vehicle_verts_unrotated')' + [s_hdv, y_hdv];
        
        set(hdv_patch, 'XData', verts_hdv(:,1), 'YData', verts_hdv(:,2));

        % C. camera update 
        current_s = simX{2}(k, 1);
        xlim(ax, [current_s - 20, current_s + 20]); 
        
        ylim(ax, [-5, 5]); 
        
        title(ax, sprintf('Simulation Time: %.2f s', t_array(k)), 'FontSize', 18);

        % D. Frame capture
        drawnow limitrate; 
        frame = getframe(fig); 
        writeVideo(writerObj, frame);
        
        if mod(k, 50) == 0
            fprintf('Rendering frame %d / %d\n', k, Nsim);
        end
    end

    % 5 Video closing
    close(writerObj);
    close(fig);
    fprintf('Video salvato con successo: %s\n', video_filename);
end