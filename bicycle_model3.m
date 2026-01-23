function [model, constraint] = bicycle_model3(track, list_constr, sim_data)
import casadi.*

model = struct();
constraint = struct();
model_name = 'Spatialbicycle_model';

% Parametri tracciato (semplificato per rettilineo)
%if strcmp(track, 'rettilineo.txt')
s0 = (0:1:200)';
kapparef = zeros(size(s0));
kapparef_s = interpolant('kapparef_s', 'linear', {s0}, kapparef);
%else
%    error('Tracciato non curvo');
%end

%% CasADi Model
% Stati Ego 
s = SX.sym('s'); 
y = SX.sym('y'); 
theta = SX.sym('theta');
delta = SX.sym('delta'); 
v = SX.sym('v');
x = vertcat(s, y, theta, delta, v);

% Controlli
derDelta = SX.sym('derDelta'); 
derV = SX.sym('derV');
u = vertcat(derDelta, derV);

% Derivate
% Set up xdot
sdot = SX.sym('sdot');
ydot = SX.sym('ydot');
thetadot = SX.sym('thetadot');
deltadot = SX.sym('deltadot');
vdot = SX.sym('vdot');
xdot = vertcat(sdot, ydot, thetadot, deltadot, vdot);

%% Parametri
% Fisici
lf = 0.959; 
lr = 1.706; 
% D1 = 4.68; 
% D2 = 2.2; 
D1 = 4.3; 
D2 = 1.8; 
% D1_h = 4.68; 
% D2_h = 2.2;
D1_h = 4.3; 
D2_h = 1.8;
y_lim = 8.0;

% Parametri per ostacoli (verranno passati dal risolutore)
p = SX.sym('p', 4 * 2); 
onoff_e = SX.sym('onoff_e');
onoff_h = SX.sym('onoff_h');
onoff_lane = SX.sym('onoff_lane');

p5_e = SX.sym('p5_e');
p6_e = SX.sym('p6_e');
p5_h = SX.sym('p5_h');
p6_h = SX.sym('p6_h');

bhj_e = SX.sym('bhj_e');
bhj_h = SX.sym('bhj_h');

HP = SX.sym('HP');

lane_parameter = SX.sym('lane_parameter');

lane_occupied = SX.sym('lane_occupied');

p = vertcat(p, onoff_e, onoff_h, onoff_lane, p5_e, p6_e, p5_h, p6_h, bhj_e, bhj_h, HP, lane_parameter, lane_occupied);

%% Vincoli non lineari
C_vec = [];

D = sqrt((D1/2)^2+(D2/2)^2); 
alpha = asin(D2/D1);


% if list_constr.left_lane 
%     y_left_expr = y+D1/2*sin(theta) + (-0.5+lane_occupied)*-1*-(D2/2+0.5) +...
%             (+0.5+lane_occupied)*-(y_lim/2-D2/2-0.5);
%     C_left = exp(y_left_expr); 
%     C_vec = [C_vec; C_left];
% end
% 
% if list_constr.right_lane 
%     y_right_expr = y+D1/2*sin(theta)+(-0.5+lane_occupied)*-1*(y_lim/2-D2/2-0.5) + ...
%             (+0.5+lane_occupied)*-(D2/2+0.5);
%     C_right = exp(-y_right_expr);
%     C_vec = [C_vec; C_right];
% end

if list_constr.left_lane 
    y_left_expr = (-0.5+lane_occupied)*-1*(y+D*sin(alpha+theta)-0.2) +...
            (+0.5+lane_occupied)*(y+D*sin(-alpha+theta)-y_lim/2+0.2);
    C_left = exp(y_left_expr); 
    C_vec = [C_vec; C_left];
end

if list_constr.right_lane 
    y_right_expr = (-0.5+lane_occupied)*-1*(y+D*sin(-alpha+theta) + y_lim/2 - 0.2) + ...
            (+0.5+lane_occupied)*(y+D*sin(alpha+theta) + 0.2);
    C_right = exp(-y_right_expr);
    C_vec = [C_vec; C_right];
end

num_obstacles = (length(p)-9) / 4;
if list_constr.eucl
    for i = 1:num_obstacles
        p_i = p((i-1)*4 + 1 : i*4);
        shdv_i = p_i(1); 
        yhdv_i = p_i(2); 
        thetahdv_i = p_i(3);
    
        dist1 = (s+D1/4*cos(theta)-shdv_i-D1_h/4*cos(thetahdv_i))^2 ...
            +(y+D1/4*sin(theta)-yhdv_i-D1_h/4*sin(thetahdv_i))^2 ...
            -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;
        dist2 = (s+D1/4*cos(theta)-shdv_i+D1_h/4*cos(thetahdv_i))^2 ...
            +(y+D1/4*sin(theta)-yhdv_i+D1_h/4*sin(thetahdv_i))^2 ...
            -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;
        dist3 = (s-D1/4*cos(theta)-shdv_i-D1_h/4*cos(thetahdv_i))^2 ...
            +(y-D1/4*sin(theta)-yhdv_i-D1_h/4*sin(thetahdv_i))^2 ...
            -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;
        dist4 = (s-D1/4*cos(theta)-shdv_i+D1_h/4*cos(thetahdv_i))^2 ...
            +(y-D1/4*sin(theta)-yhdv_i+D1_h/4*sin(thetahdv_i))^2 ...
            -(sqrt((D1/4)^2+(D2/2)^2)+sqrt((D1_h/4)^2+(D2_h/2)^2))^2;
        
        C_dist1 = exp(-(dist1)); 
        C_dist2 = exp(-(dist2));
        C_dist3 = exp(-(dist3)); 
        C_dist4 = exp(-(dist4));
        
        C_vec = vertcat(C_vec, C_dist1, C_dist2, C_dist3, C_dist4);
    end
end

% Vincoli di sorpasso e decelerazione 
if list_constr.get_overtaken
    get_overtaken = (-7+abs(y-p(6))+0.05*(s-p(5))^2);
    C_overtaken = HP*exp(-get_overtaken);
    C_vec = [C_vec; C_overtaken];
end

if list_constr.decelerate
    decelerate =  (3+v-p(8)+0.5*(s-p(5))^2);
    C_decelerate = HP*exp(-decelerate);
    C_vec = [C_vec; C_decelerate];
end

%C_vec = vertcat(C_left, C_right, C_vec, C_overtaken, C_decelerate);
if list_constr.center_lane
    % center_lane_eq = 0.5*(cos(2.8*y/pi)+1)^2 +(0.2*y)^4 - 0.31;
    %center_lane_eq = 5*(0.02*(cos(y/pi)+1)^2 +(0.135*y)^6) - 0.26;
    %center_lane_eq = log(1+exp(7*(y-6)))+log(1+exp(-7*(y+6)))+0.05*(cos(y/1.086)+1)^2;
    center_lane_eq = 0.01*((1/(1+exp(15*(y-3.35))))+(1/(1+exp(-15*(y+3.35))))-1);
    center_lane = exp(center_lane_eq);
    C_vec = [C_vec; center_lane];
end

if list_constr.lane_keeping
    lane_keeping_eq = onoff_lane*(y+lane_parameter)^2;
    lane_keeping = onoff_lane*exp(lane_keeping_eq);
    C_vec = [C_vec; lane_keeping];
end

u_safe = [];

if list_constr.brt_constr.ego == true
        u_safe = vertcat(u_safe, onoff_e*(p5_e*derDelta+p6_e*derV+bhj_e));
end

if list_constr.brt_constr.hum == true
        u_safe = vertcat(u_safe, onoff_h*(p5_h*derDelta+p6_h*derV+bhj_h));
end


%% Dinamica
kappa_s = kapparef_s(s);
beta = atan(lr/(lr+lf)*tan(delta));
f_expl = vertcat((v*cos(theta+beta))/(1-y*kappa_s),...
                  v*sin(theta+beta),...
                  v/lr*sin(beta) - kappa_s*(v*cos(theta+beta))/(1-y*kappa_s),...
                  derDelta,...
                  derV);
                  
%% Output del modello
% Limiti
model.y_min = -y_lim/2+D2/2; 
model.y_max = y_lim/2-D2/2;
model.v_min = sim_data.v_min;
model.v_max = sim_data.v_max;
model.delta_min = sim_data.delta_min;
model.delta_max = sim_data.delta_max;
model.ddelta_min = sim_data.ddelta_min; 
model.ddelta_max = sim_data.ddelta_max;
model.dv_min = sim_data.dv_min; 
model.dv_max = sim_data.dv_max;

% Riferimenti e condizioni iniziali
model.v_ref = 5;
model.x0 = [15, -3.4, 0, 0, model.v_ref];
model.x_ref = [15, -3.4, 0, 0, model.v_ref];

% Struttura vincoli
constraint.x_expr = vertcat(y, delta, v);
constraint.u_expr = vertcat(derDelta, derV);
constraint.nonli_expr = vertcat(C_vec, u_safe);

% CORREZIONE: Aggiungere la struttura dei parametri al modello
params = struct();
params.lf1 = lf;
params.lr1= lr;
params.lf2 = lf;
params.lr2= lr;
params.D1 = D1;
params.D2 = D2;
params.D1_h = D1_h;
params.D2_h = D2_h;

% Struttura modello
model.f_impl_expr = f_expl - xdot;
model.f_expl_expr = f_expl;
model.x = x; 
model.xdot = xdot; 
model.u = u; 
model.z = [];
model.p = p;
model.name = model_name;
model.params = params; 

end
