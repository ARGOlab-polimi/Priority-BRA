function [ttc_mpc, ev_states, time] = getTTC_mpc(states_mpc, simX, model, simX_hum, T, dt, t)

iter = length(simX{1}(:,1));

delta_time = dt; % the delta time is the same of the mpc 

max_time = 10; % ttc value upper limit 
time = delta_time:delta_time:max_time-T;

time_it = length(time);

lr = model.params.lr1;
lf = model.params.lf1;

ttcs = -1*ones(iter, 6);

dyn_ego=@(theta, delta, v, beta)  [v*cos(theta+beta); 
                                   v*sin(theta+beta);
                                   v*cos(beta)*tan(delta)/(lr + lf)];

ev_states1 = zeros(3,time_it,iter);
ev_states2 = ev_states1;
ev_states3 = ev_states1;


for i=1:iter
    x1 = states_mpc{1}(1,end,i);
    x2 = states_mpc{2}(1,end,i);
    x3 = states_mpc{3}(1,end,i);

    y1 = states_mpc{1}(2,end,i);
    y2 = states_mpc{2}(2,end,i);
    y3 = states_mpc{3}(2,end,i);

    theta1 = states_mpc{1}(3,end,i);
    theta2 = states_mpc{2}(3,end,i);
    theta3 = states_mpc{3}(3,end,i);

    delta1 = states_mpc{1}(4,end,i);
    delta2 = states_mpc{2}(4,end,i);
    delta3 = states_mpc{3}(4,end,i);

    v1 = states_mpc{1}(5,end,i);
    v2 = states_mpc{2}(5,end,i);
    v3 = states_mpc{3}(5,end,i);

    beta1 = atan(lr/(lr+lf)*tan(delta1));
    beta2 = atan(lr/(lr+lf)*tan(delta2));
    beta3 = atan(lr/(lr+lf)*tan(delta3));

   
    for j=1:time_it
    
    der1 = dyn_ego(theta1, delta1, v1, beta1);
    der2 = dyn_ego(theta2, delta2, v2, beta2);
    der3 = dyn_ego(theta3, delta3, v3, beta3);

    [states1_new] = [x1; y1; theta1] + delta_time*der1;
    [states2_new] = [x2; y2; theta2] + delta_time*der2;
    [states3_new] = [x3; y3; theta3] + delta_time*der3;
    
    ev_states1(:,j,i) = states1_new;
    ev_states2(:,j,i) = states2_new;
    ev_states3(:,j,i) = states3_new;
    
    x1 = states1_new(1);
    x2 = states2_new(1);
    x3 = states3_new(1);

    y1 = states1_new(2);
    y2 = states2_new(2);
    y3 = states3_new(2);

    theta1 = states1_new(3);
    theta2 = states2_new(3);
    theta3 = states3_new(3);

    end
end

total_length = length(states_mpc{1}(1,:,1)) + length(ev_states1(1,:,1));

all_states1 = zeros(3,total_length,iter);
all_states2 = all_states1;
all_states3 = all_states1;
all_statesh = all_states1;

all_states1(:,1:length(states_mpc{1}(1,:,1)),:) = states_mpc{1}(1:3,:,:);
all_states2(:,1:length(states_mpc{1}(1,:,1)),:) = states_mpc{2}(1:3,:,:);
all_states3(:,1:length(states_mpc{1}(1,:,1)),:) = states_mpc{3}(1:3,:,:);

all_states1(:,length(states_mpc{1}(1,:,1))+1:end,:) = ev_states1;
all_states2(:,length(states_mpc{1}(1,:,1))+1:end,:) = ev_states2;
all_states3(:,length(states_mpc{1}(1,:,1))+1:end,:) = ev_states3;

ev_states1 = all_states1;
ev_states2 = all_states2;
ev_states3 = all_states3;

time = linspace(delta_time,max_time, total_length);

for l=1:iter
    h.simX_0 = simX_hum(l,1:4);
    h.simU = zeros(total_length, 2);
    h.simU_0 = zeros(2, 1);

    for k=1:total_length
    
    h = human_integrate(h,k,delta_time);
    all_statesh(:,k,l) = h.simX(k, 1:3);

    states1_new = ev_states1(:,k,l);
    states2_new = ev_states2(:,k,l);
    states3_new = ev_states3(:,k,l);

    [x_v12, y_v12] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], states1_new(3), states2_new(3));
    [x_v13, y_v13] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], states1_new(3), states3_new(3));
    [x_v23, y_v23] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], states2_new(3), states3_new(3));

    [x_v1h, y_v1h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], states1_new(3), h.simX(k,3));
    [x_v2h, y_v2h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], states2_new(3), h.simX(k,3));
    [x_v3h, y_v3h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], states3_new(3), h.simX(k,3));

    %ego1 vs ego2
    if ttcs(l,1) < 0
        if inpolygon(states2_new(1), states2_new(2), x_v12+states1_new(1), y_v12+states1_new(2))
            ttcs(l,1) = time(k);
        end
    end

    %ego1 vs ego3
    if ttcs(l,2) < 0
        if inpolygon(states3_new(1), states3_new(2), x_v13+states1_new(1), y_v13+states1_new(2))
            ttcs(l,2) = time(k);
        end
    end
    
    %ego2 vs ego3
    if ttcs(l,3) < 0
        if inpolygon(states3_new(1), states3_new(2), x_v23+states2_new(1), y_v23+states2_new(2))
            ttcs(l,3) = time(k);
        end
    end

    %ego1 vs hum
    if ttcs(l,4) < 0
        if inpolygon(h.simX(k, 1), h.simX(k, 2), x_v1h+states1_new(1), y_v1h+states1_new(2))
            ttcs(l,4) = time(k);
        end
    end

    %ego2 vs hum
    if ttcs(l,5) < 0
        if inpolygon(h.simX(k, 1), h.simX(k, 2), x_v2h+states2_new(1), y_v2h+states2_new(2))
            ttcs(l,5) = time(k);
        end
    end

    %ego3 vs hum
    if ttcs(l,6) < 0
        if inpolygon(h.simX(k, 1), h.simX(k, 2), x_v3h+states3_new(1), y_v3h+states3_new(2))
            ttcs(l,6) = time(k);
        end
    end
    end
    clc
    disp(['Time istant ' num2str(l) ' out of ' num2str(iter)]);
end
ttcs(ttcs<0) = inf;

ttc_mpc = ttcs;

ev_states.ev_states1 = all_states1;
ev_states.ev_states2 = all_states2;
ev_states.ev_states3 = all_states3;
ev_states.ev_statesh = all_statesh;



figure
subplot(3,1,1)
plot(t, ttcs(:,1), '.')
ylim([-2 20])
xlim([0 t(end)])
grid on
title('Time to collision between ego1 and ego2')
xlabel('Simulation Time [s]')
ylabel('TTC [s]')
subplot(3,1,2)
plot(t, ttcs(:,2),  '.')
ylim([-2 20])
xlim([0 t(end)])
grid on
title('Time to collision between ego1 and ego3')
xlabel('Simulation Time [s]')
ylabel('TTC [s]')
subplot(3,1,3)
plot(t, ttcs(:,3), '.')
ylim([-2 20])
xlim([0 t(end)])
grid on
title('Time to collision between ego2 and ego3')
xlabel('Simulation Time [s]')
ylabel('TTC [s]')

figure
subplot(3,1,1)
plot(t, ttcs(:,4), '.')
ylim([-2 20])
xlim([0 t(end)])
grid on
title('Time to collision between ego1 and hdv')
xlabel('Simulation Time [s]')
ylabel('TTC [s]')
subplot(3,1,2)
plot(t, ttcs(:,5), '.')
ylim([-2 20])
xlim([0 t(end)])
grid on
title('Time to collision between ego2 and hdv')
xlabel('Simulation Time [s]')
ylabel('TTC [s]')
subplot(3,1,3)
plot(t, ttcs(:,6), '.')
ylim([-2 20])
xlim([0 t(end)])
grid on
title('Time to collision between ego3 and hdv')
xlabel('Simulation Time [s]')
ylabel('TTC [s]')

