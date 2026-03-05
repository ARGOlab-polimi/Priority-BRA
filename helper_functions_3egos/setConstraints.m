function ocp_model = setConstraints(ocp_model, model, constr_list, constraint, uh_data, lh_data, ...
    soft_l_diag_data, soft_u_diag_data, soft_l_quad, soft_u_quad)

% State Constraints
nbx = length(constraint.x_expr); 
nx = length(model.x);
Jbx = zeros(nbx, nx);
Jbx(1,2) = 1; 
Jbx(2,4) = 1; 
Jbx(3,5) = 1;
ocp_model.set('constr_Jbx', Jbx);

% Input Constraints
nbu = length(constraint.u_expr); 
nu = length(model.u);
Jbu = zeros(nbu, nu); 
Jbu(1,1)=1; 
Jbu(2,2)=1;
ocp_model.set('constr_Jbu', Jbu);

% Nonlinear Constraints - soft constraints weights
nh = length(constraint.nonli_expr);
ocp_model.set('constr_expr_h', constraint.nonli_expr);

uh = [];
lh = [];

soft_l_diag = [];
soft_u_diag = [];

soft_l_diag2 = [];
soft_u_diag2 = [];

if constr_list.left_lane
    lh = [lh; lh_data(1)];
    uh = [uh; uh_data(1)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(1)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(1)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(1)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(1)];
end

if constr_list.right_lane
    lh = [lh; lh_data(2)];
    uh = [uh; uh_data(2)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(2)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(2)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(2)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(2)];
end

if constr_list.eucl
    lh = [lh; lh_data(3:10)];
    uh = [uh; uh_data(3:10)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(3:10)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(3:10)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(3:10)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(3:10)];
end

if constr_list.get_overtaken
    lh = [lh; lh_data(11)];
    uh = [uh; uh_data(11)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(11)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(11)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(11)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(11)];
end

if constr_list.decelerate
    lh = [lh; lh_data(12)];
    uh = [uh; uh_data(12)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(12)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(12)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(12)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(12)];
end

if constr_list.center_lane
    lh = [lh; lh_data(13)];
    uh = [uh; uh_data(13)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(13)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(13)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(13)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(13)];
end

if constr_list.lane_keeping
    lh = [lh; lh_data(14)];
    uh = [uh; uh_data(14)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(14)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(14)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(14)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(14)];
end

if constr_list.brt_constr.ego
    lh = [lh; lh_data(15)];
    uh = [uh; uh_data(15)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(15)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(15)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(15)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(15)];
end

if constr_list.brt_constr.hum
    lh = [lh; lh_data(16)];
    uh = [uh; uh_data(16)];
    soft_l_diag = [soft_l_diag; soft_l_diag_data(16)];
    soft_u_diag = [soft_u_diag; soft_u_diag_data(16)];
    soft_l_diag2 = [soft_l_diag2; soft_l_quad(16)];
    soft_u_diag2 = [soft_u_diag2; soft_u_quad(16)];
end

ocp_model.set('constr_expr_h_0', constraint.nonli_expr);
ocp_model.set('constr_lh_0', lh);
ocp_model.set('constr_uh_0', uh);

ocp_model.set('constr_expr_h', constraint.nonli_expr);
ocp_model.set('constr_lh', lh);
ocp_model.set('constr_uh', uh);


% Slack Variables
nsh = nh;
Jsh = eye(nsh);
ocp_model.set('constr_Jsh_0', Jsh);
ocp_model.set('constr_Jsh', Jsh);

% Cost on slack
ocp_model.set('cost_zl_0', soft_l_diag);          %linear term
ocp_model.set('cost_zu_0', soft_u_diag);          %linear term
ocp_model.set('cost_zl', soft_l_diag);            %linear term
ocp_model.set('cost_zu', soft_u_diag);            %linear term

ocp_model.set('cost_Zl_0', diag(soft_l_diag2));    %quadratic term
ocp_model.set('cost_Zu_0', diag(soft_u_diag2));    %quadratic term
ocp_model.set('cost_Zl', diag(soft_l_diag2));      %quadratic term
ocp_model.set('cost_Zu', diag(soft_u_diag2));      %quadratic term

