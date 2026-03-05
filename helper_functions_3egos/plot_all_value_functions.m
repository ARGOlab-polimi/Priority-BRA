function handles = plot_all_value_functions(V_ego_vs_human, V_ego_vs_ego, brt_activated, Tf, Nsim, num_egos)
    % Time vector
    t = linspace(0, Tf, Nsim);
    default_colors = get(groot, 'defaultAxesColorOrder');
    
    % Configurazione Font per Pubblicazioni (Bilanciamento leggibilità/spazio)
    axis_font_settings = struct(...
        'FontSize', 16, ... 
        'FontName', 'Times New Roman', ...
        'TickLabelInterpreter', 'latex');
    
    % Dimensioni "Journal-ready" (Rapporto rettangolare 2:1)
    fig_width = 14; 
    fig_height = 7; 
        
    % Output struct initialization
    handles = struct();
    handles.ego_figs = cell(num_egos, 1);
    handles.ego_axes = cell(num_egos, 1);
    handles.combined_figs = cell(num_egos, 1); % Nuovi handles per i grafici combinati
    handles.combined_axes = cell(num_egos, 1);

    % --- 1. Plot HDV vs EGOS (Overview) ---
    fig_h = figure('Name', 'Human-Ego Value Function', ...
                   'Units', 'centimeters', ...
                   'Position', [2, 2, fig_width, fig_height]); 
    ax_h = axes('Parent', fig_h);
    set(ax_h, axis_font_settings);
    hold(ax_h, 'on');
    
    plot(ax_h, t, V_ego_vs_human(:, 1:num_egos), 'LineWidth', 2);
    
    % Calcolo limiti Y basato sull'attivazione
    max_vals = zeros(1, num_egos);
    for i = 1:num_egos
        if ~isempty(brt_activated{i})
             max_vals(i) = max(V_ego_vs_human((brt_activated{i}(:,1)==1), i));
        end
    end
    up_lim = max(max_vals) + 0.5;
    if isempty(up_lim) || isnan(up_lim), up_lim = 10; end % Fallback safety
    
    ylim(ax_h, [0, up_lim]); 
    xlabel(ax_h, '$t$ [s]', 'Interpreter', 'latex', 'FontSize', 16); 
    ylabel(ax_h, '$V$', 'Interpreter', 'latex', 'FontSize', 16);      
    
    legend_labels = arrayfun(@(i) sprintf('LP%d-HP', i), 1:num_egos, 'UniformOutput', false);
    legend(ax_h, legend_labels, 'Interpreter', 'latex', 'FontSize', 14, ...  
           'Location', 'best', 'Box', 'on');
           
    grid(ax_h, 'on');
    box(ax_h, 'on');
    hold(ax_h, 'off');
    
    handles.human_fig = fig_h;
    handles.human_ax = ax_h;

    % --- 2. Plot EGOS vs EGOS (Specific LP interactions) ---
    for ego_i = 1:num_egos
        fig_name = sprintf('Value Function for LP%d (vs LPs)', ego_i);
        fig_e = figure('Name', fig_name, ...
                       'Units', 'centimeters', ...
                       'Position', [5, 5, fig_width, fig_height]); 
        ax_e = axes('Parent', fig_e);
        set(ax_e, axis_font_settings);
        hold(ax_e, 'on');
        
        plot_handles = [];
        current_figure_legends = {};
        
        for ii = 1:num_egos
            try
                idx_active = (brt_activated{ego_i}(:, 2) == ii);
            catch E
                warning('Errore ego_i=%d, ii=%d: %s', ego_i, ii, E.message);
                continue; 
            end
            
            if any(idx_active)
                V_to_plot = NaN(size(t));
                V_to_plot(idx_active) = V_ego_vs_ego(idx_active, ego_i);
                
                h = plot(ax_e, t, V_to_plot, 'LineWidth', 2, ...
                         'Color', default_colors(mod(ii-1, size(default_colors,1))+1, :));
                
                plot_handles = [plot_handles, h];
                current_figure_legends = [current_figure_legends, sprintf('with LP%d', ii)];
            end
        end
        
        max_v_ego = max(V_ego_vs_ego, [], "all");
        if isempty(max_v_ego), max_v_ego = 5; end
        ylim(ax_e, [0, max_v_ego + 0.5]);
        
        xlabel(ax_e, '$t$ [s]', 'Interpreter', 'latex', 'FontSize', 16); 
        ylabel(ax_e, '$V$', 'Interpreter', 'latex', 'FontSize', 16);     
        
        if ~isempty(plot_handles)
            legend(ax_e, plot_handles, current_figure_legends, ...
                   'Interpreter', 'latex', 'FontSize', 14, 'Location', 'best', 'Box', 'on');
        end
               
        grid(ax_e, 'on');
        box(ax_e, 'on');
        hold(ax_e, 'off');
        
        handles.ego_figs{ego_i} = fig_e;
        handles.ego_axes{ego_i} = ax_e;
    end

    % --- 3. Plot COMBINED (HP + LPs) for each EGO ---
    % NUOVO CODICE RICHIESTO: Plot singolo veicolo con sia HP che LP BRT
    for ego_i = 1:num_egos
        fig_name = sprintf('Combined Value Functions (HP & LP) for LP%d', ego_i);
        % Position offset leggermente per non sovrapporsi esattamente alle precedenti
        fig_c = figure('Name', fig_name, ...
                       'Units', 'centimeters', ...
                       'Position', [8, 8, fig_width, fig_height]); 
        ax_c = axes('Parent', fig_c);
        set(ax_c, axis_font_settings);
        hold(ax_c, 'on');

        plot_handles = [];
        legends = {};

        % A) Plot HP Interaction (vs Human)
        % Usiamo il nero ('k') per distinguere nettamente l'HP dalle interazioni LP
        h_hp = plot(ax_c, t, V_ego_vs_human(:, ego_i), 'LineWidth', 2, 'Color', 'k');
        plot_handles = [plot_handles, h_hp];
        legends = [legends, 'vs HP'];

        % B) Plot LP Interaction (vs other Egos) - Solo quando attivi
        for ii = 1:num_egos
            try
                idx_active = (brt_activated{ego_i}(:, 2) == ii);
            catch
                continue;
            end
            
            if any(idx_active)
                V_to_plot = NaN(size(t));
                V_to_plot(idx_active) = V_ego_vs_ego(idx_active, ego_i);
                
                % Usa lo stesso colore assegnato a quel veicolo nel loop precedente
                col = default_colors(mod(ii-1, size(default_colors,1))+1, :);
                
                h_lp = plot(ax_c, t, V_to_plot, 'LineWidth', 2, 'Color', col);
                plot_handles = [plot_handles, h_lp];
                legends = [legends, sprintf('vs LP%d', ii)];
            end
        end

        % Calcolo limiti Y dinamici (considerando sia max HP che max LP)
        max_val_hp = max(V_ego_vs_human(:, ego_i), [], 'all');
        max_val_lp = max(V_ego_vs_ego(:, ego_i), [], 'all');
        
        if isempty(max_val_hp), max_val_hp = 0; end
        if isempty(max_val_lp), max_val_lp = 0; end
        
        ylim(ax_c, [0, max([max_val_hp, max_val_lp]) + 0.5]);

        % Etichette e stile
        xlabel(ax_c, '$t$ [s]', 'Interpreter', 'latex', 'FontSize', 16); 
        ylabel(ax_c, '$V$', 'Interpreter', 'latex', 'FontSize', 16);     
        % Titolo opzionale (rimuovere se non desiderato per paper)
        % title(ax_c, sprintf('LP%d: All Interactions', ego_i), 'Interpreter', 'latex');

        if ~isempty(plot_handles)
            legend(ax_c, plot_handles, legends, ...
                   'Interpreter', 'latex', 'FontSize', 14, 'Location', 'best', 'Box', 'on');
        end

        grid(ax_c, 'on');
        box(ax_c, 'on');
        hold(ax_c, 'off');

        handles.combined_figs{ego_i} = fig_c;
        handles.combined_axes{ego_i} = ax_c;
    end
end