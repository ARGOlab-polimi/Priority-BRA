function res = save_results(scenario_name, simX, simU, simX_hdv, V_ego_vs_ego, V_ego_vs_human, states_mpc, ...
                                                 brt_activated, ham_ego, t, time_ocp, time_tot, model)

target_folder = 'Results';

[ttcs_dyn, ev_states_dyn, time_future_dyn] = getTTC_dyn(simX, simX_hdv, model, t);

ttc_dyn_data.ttcs_dyn = ttcs_dyn;
ttc_dyn_data.ev_states_dyn = ev_states_dyn;
ttc_dyn_data.time_future_dyn = time_future_dyn;

res = struct();

res.simX = simX;
res.simU = simU;
res.simX_hdv = simX_hdv;
res.V_ego_vs_ego = V_ego_vs_ego;
res.V_ego_vs_human = V_ego_vs_human;
res.states_mpc = states_mpc;
res.brt_activated = brt_activated;
res.ham_ego = ham_ego;
res.t = t;
res.time_ocp = time_ocp;
res.time_tot = time_tot;
res.ttc_dyn_data = ttc_dyn_data;

if ~endsWith(scenario_name, '.mat')
    scenario_name = [scenario_name, '.mat'];
end

full_path = fullfile(target_folder, scenario_name);

try
    save(full_path, 'res'); 
catch ME
    warning(ME.message);
end
end