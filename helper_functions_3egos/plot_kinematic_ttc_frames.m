function fig = plot_kinematic_ttc_frames(simX, simX_hum, ttcs, id_vehicle, target_time)
    dt = 0.05;
    
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
    
    ids_sorted = sort(id_vehicle);
    if isequal(ids_sorted, [1 2]), ttc_val = ttcs(frame_idx, 1);
    elseif isequal(ids_sorted, [1 3]), ttc_val = ttcs(frame_idx, 2);
    elseif isequal(ids_sorted, [2 3]), ttc_val = ttcs(frame_idx, 3);
    elseif isequal(ids_sorted, [0 1]), ttc_val = ttcs(frame_idx, 4);
    elseif isequal(ids_sorted, [0 2]), ttc_val = ttcs(frame_idx, 5);
    elseif isequal(ids_sorted, [0 3]), ttc_val = ttcs(frame_idx, 6);
    else, ttc_val = 0; error('Combinazione ID non valida'); end
    
    if isinf(ttc_val) || ttc_val < 0
        ttc_val = 4.0;
    end
    
    t_raw = linspace(0, ttc_val, 5);
    t_target = t_raw;
    if length(t_target) >= 3
        t_target(2:end-1) = round(t_raw(2:end-1) / 0.05) * 0.05;
    end
    t_eval = unique(t_target);
    
    [stateA, dimsA] = get_kinematic_state(id_vehicle(1), simX, simX_hum, frame_idx);
    [stateB, dimsB] = get_kinematic_state(id_vehicle(2), simX, simX_hum, frame_idx);
    
    get_x = @(state, t) state(1) + state(4) * cos(state(3)) * t;
    get_y = @(state, t) tan(state(3)) * (get_x(state, t) - state(1)) + state(2);
    
    [x_poly_rel, y_poly_rel] = shapePolygonLevelRobust(dimsA, dimsB, stateA(3), stateB(3));
    x_poly_rel = [x_poly_rel(:); x_poly_rel(1)];
    y_poly_rel = [y_poly_rel(:); y_poly_rel(1)];
    
    fig = figure('Name', sprintf('Kinematic Shape Evolution ID %d vs %d', id_vehicle(1), id_vehicle(2)));
    ax = axes('Parent', fig);
    set(ax, axis_font_settings);
    hold(ax, 'on');
    axis(ax, 'equal');
    box(ax, 'on');
    grid(ax, 'on');
    
    num_steps = length(t_eval);
    alphas = linspace(0.4, 1.0, num_steps);
    color_A = get_v_color_rgb(id_vehicle(1));
    color_B = get_v_color_rgb(id_vehicle(2));
    
    min_x = inf; max_x = -inf;
    min_y = inf; max_y = -inf;
    
    for k = 1:num_steps
        delta_t = t_eval(k);
        
        if k == 1
            disp_name = '+0.00 s';
        elseif k == num_steps
            disp_name = sprintf('+%.2f s', delta_t);
        else
            delta_t_disp = round(delta_t / 0.05) * 0.05;
            disp_name = sprintf('+%.2f s', delta_t_disp);
        end
        
        xA = get_x(stateA, delta_t);
        yA = get_y(stateA, delta_t);
        
        xB = get_x(stateB, delta_t);
        yB = get_y(stateB, delta_t);
        
        x_poly = x_poly_rel + xA;
        y_poly = y_poly_rel + yA;
        
        lw = 1.2;
        if k == num_steps, lw = 2.0; end
        
        plot(ax, x_poly, y_poly, 'LineStyle', '-', 'Color', [color_A, alphas(k)], ...
             'LineWidth', lw, 'DisplayName', disp_name);
         
        scatter(ax, xA, yA, 40, color_A, 'filled', 'o', ...
            'MarkerFaceAlpha', alphas(k), 'MarkerEdgeAlpha', alphas(k), ...
            'HandleVisibility', 'off');
            
        scatter(ax, xB, yB, 80, color_B, 'x', 'LineWidth', 2.0, ...
            'MarkerEdgeAlpha', alphas(k), ...
            'HandleVisibility', 'off'); 
        
        min_x = min([min_x; x_poly; xB]);
        max_x = max([max_x; x_poly; xB]);
        min_y = min([min_y; y_poly; yB]);
        max_y = max([max_y; y_poly; yB]);
    end
    
    xlabel(ax, '$x$ [m]', 'Interpreter', 'latex');
    ylabel(ax, '$y$ [m]', 'Interpreter', 'latex');
    
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

function c = get_v_color_rgb(id)
    if id == 0
        c = [0.5, 0.5, 0.5];
    elseif id == 1
        c = [0, 0, 1];
    elseif id == 2
        c = [1, 0, 0];
    elseif id == 3
        c = [1, 1, 0];
    else
        c = [0, 0, 0];
    end
end