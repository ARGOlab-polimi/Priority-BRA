function fig = plot_frames_ttc(ev_states, target_time, ttcs, id_vehicle, model, simX, dt, time_future)

    axis_font_settings = struct(...
        'FontSize', 14, ...
        'FontName', 'Times New Roman', ...
        'TickLabelInterpreter', 'latex');
    
    % time and frame managemnet 
    Nsim = size(simX{1}, 1);
    t_max = (Nsim - 1) * dt;
    
    if target_time > t_max
        target_time = t_max;
        warning('Target time oltre durata sim. Uso t=%.2f s.', t_max);
    elseif target_time < 0
        target_time = 0;
        warning('Target time negativo. Uso t=0 s.');
    end
    
    frame_idx = round(target_time / dt) + 1; 
    frame_idx = max(1, min(frame_idx, Nsim)); 
    
    % ttc selection
    ids_sorted = sort(id_vehicle);
    if isequal(ids_sorted, [1 2]), ttc_val = ttcs(frame_idx, 1);
    elseif isequal(ids_sorted, [1 3]), ttc_val = ttcs(frame_idx, 2);
    elseif isequal(ids_sorted, [2 3]), ttc_val = ttcs(frame_idx, 3);
    elseif isequal(ids_sorted, [0 1]), ttc_val = ttcs(frame_idx, 4);
    elseif isequal(ids_sorted, [0 2]), ttc_val = ttcs(frame_idx, 5);
    elseif isequal(ids_sorted, [0 3]), ttc_val = ttcs(frame_idx, 6);
    else, ttc_val = 0; warning('ID combinazione non valida'); end

    if ttc_val > 20
        ttc_val = 10;
    end
    
    % step evaluation
    dt_future = time_future(2) - time_future(1);
    if dt_future <= 0, dt_future = 0.1; end 
    
    idx_end = round(ttc_val / dt_future); 
    max_future_frames = size(ev_states.ev_states1, 2);
    idx_end = max(1, min(idx_end, max_future_frames)); 
    
    % 5 equal spaced frames
    pred_indices = unique(round(linspace(1, idx_end, 5)));
    
    [states_A_all, dims_A] = get_vehicle_data(id_vehicle(1), ev_states, model);
    [states_B_all, dims_B] = get_vehicle_data(id_vehicle(2), ev_states, model);


    fig = figure('Name', sprintf('Shape Evolution ID %d vs %d', id_vehicle(1), id_vehicle(2)));
    ax = axes('Parent', fig);
    set(ax, axis_font_settings);
    hold(ax, 'on');
    axis(ax, 'equal');
    box(ax, 'on');
    grid(ax, 'on');
    
    colors = parula(length(pred_indices)); 

    min_x = inf; max_x = -inf;
    min_y = inf; max_y = -inf;
    
    % Loop plotting 
    for k = 1:length(pred_indices)
        pidx = pred_indices(k);
    
        delta_t = pidx * dt_future; 
        
        sA = states_A_all(:, pidx, frame_idx); 
        sB = states_B_all(:, pidx, frame_idx);
        xA = sA(1); yA = sA(2); thetaA = sA(3);
        xB = sB(1); yB = sB(2); thetaB = sB(3);
        
        [x_poly_rel, y_poly_rel] = shapePolygonLevelRobust(dims_A, dims_B, thetaA, thetaB);

        x_poly = x_poly_rel + xA;
        y_poly = y_poly_rel + yA;

        x_poly = [x_poly; x_poly(1)];
        y_poly = [y_poly; y_poly(1)];
        
        lw = 1.2;
        if k == length(pred_indices), lw = 2.0; end 
        
        % Plot polygon
        plot(ax, x_poly, y_poly, 'LineStyle', '-', 'Color', colors(k,:), ...
             'LineWidth', lw, 'DisplayName', sprintf('+%.2f s', delta_t));
         
        % plot center A
        plot(ax, xA, yA, 'o', 'Color', colors(k,:), 'MarkerFaceColor', colors(k,:), ...
            'MarkerSize', 4, 'HandleVisibility', 'off');
            
        % Plot center B
        plot(ax, xB, yB, 'x', 'Color', colors(k,:), 'LineWidth', 2.0, ...
            'MarkerSize', 10, 'HandleVisibility', 'off'); 
        
        min_x = min([min_x; x_poly; xB]);
        max_x = max([max_x; x_poly; xB]);
        min_y = min([min_y; y_poly; yB]);
        max_y = max([max_y; y_poly; yB]);
    end

    xlabel(ax, '$x$ [m]', 'Interpreter', 'latex');
    ylabel(ax, '$y$ [m]', 'Interpreter', 'latex');
    title(ax, sprintf('Collision Shape ID %d vs %d @ TTC=%.2fs', id_vehicle(1), id_vehicle(2), ttc_val), 'Interpreter', 'latex');
    
    span_x = max_x - min_x;
    span_y = max_y - min_y;
    
    if span_x < 10, span_x = 10; end
    if span_y < 10, span_y = 10; end
    
    margin_x = span_x * 0.15;
    margin_y = span_y * 0.15;
    
    xlim(ax, [min_x - margin_x, max_x + margin_x]);
    ylim(ax, [min_y - margin_y, max_y + margin_y]);
    
    lgd = legend(ax, 'show');
    set(lgd, 'Location', 'bestoutside', 'Interpreter', 'latex');
    title(lgd, 'Prediction Time'); 
      
    hold(ax, 'off');
end

% --- Helper Function ---
function [states, dims] = get_vehicle_data(id, ev_states, model)
    D1_std = 4.3; D2_std = 1.8;
    if isfield(model, 'params') && isfield(model.params, 'D1')
        dims = [model.params.D1, model.params.D2];
    else
        dims = [D1_std, D2_std];
    end

    switch id
        case 1, states = ev_states.ev_states1;
        case 2, states = ev_states.ev_states2;
        case 3, states = ev_states.ev_states3;
        case 0, states = ev_states.ev_statesh;
        otherwise, error('ID %d non valido', id);
    end
end