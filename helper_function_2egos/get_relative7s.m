function [rel_states, rel_dyn] = get_relative7s(ego_xy, u0, veic2, u_v, model)

lr1 = model.params.lr1;
lf1 = model.params.lf1;

lr2 = model.params.lr2;
lf2 = model.params.lf2;


% Relative transformation matrix
xytheta_rel = [cos(ego_xy(3)) sin(ego_xy(3)) 0;
    -sin(ego_xy(3)) cos(ego_xy(3)) 0;
    0 0 1]...
    *[veic2(1)-ego_xy(1); veic2(2)-ego_xy(2); veic2(3)-ego_xy(3)];

% Relative states
rel_states = [xytheta_rel; veic2(4); veic2(5); ego_xy(4); ego_xy(5)]';


% Terms for relative dynamics

beta_e1 = atan((lr1/(lr1+lf1))*tan(veic2(4)));
beta_e2 = atan((lr2/(lr2+lf2))*tan(ego_xy(4)));

common_term = rel_states(7)*cos(beta_e2)/(lr2+lf2)*tan(rel_states(6));

% Relative dynamics
rel_dyn = [
    rel_states(5)*cos(rel_states(3)+beta_e1)-rel_states(7)*cos(beta_e2)+rel_states(2)*common_term;
    rel_states(5)*sin(rel_states(3)+beta_e1)-rel_states(7)*sin(beta_e2)-rel_states(1)*common_term;
    rel_states(5)*cos(beta_e1)/(lr1+lf1)*tan(rel_states(4))-common_term;
    u_v(1);
    u_v(2);
    u0(1);
    u0(2)]';


