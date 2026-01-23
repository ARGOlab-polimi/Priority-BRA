function [ttcs, ev_states, time] = getTTC_dyn(simX, simX_hum, model, t)

iter = length(simX{1}(:,1));

delta_time = 0.05;

time = delta_time:delta_time:10;

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
ev_statesh = ev_states1;



for i=1:iter
    x1 = simX{1}(i,1);
    x2 = simX{2}(i,1);
    x3 = simX{3}(i,1);

    y1 = simX{1}(i,2);
    y2 = simX{2}(i,2);
    y3 = simX{3}(i,2);

    theta1 = simX{1}(i,3);
    theta2 = simX{2}(i,3);
    theta3 = simX{3}(i,3);

    delta1 = simX{1}(i,4);
    delta2 = simX{2}(i,4);
    delta3 = simX{3}(i,4);

    v1 = simX{1}(i,5);
    v2 = simX{2}(i,5);
    v3 = simX{3}(i,5);

    beta1 = atan(lr/(lr+lf)*tan(delta1));
    beta2 = atan(lr/(lr+lf)*tan(delta2));
    beta3 = atan(lr/(lr+lf)*tan(delta3));

    h.simX_0 = simX_hum(i,1:4);
    h.simU = zeros(time_it, 2);
    h.simU_0 = zeros(2, 1);

    for j=1:time_it
    
    der1 = dyn_ego(theta1, delta1, v1, beta1);
    der2 = dyn_ego(theta2, delta2, v2, beta2);
    der3 = dyn_ego(theta3, delta3, v3, beta3);

    [states1_new] = [x1; y1; theta1] + delta_time*der1;
    [states2_new] = [x2; y2; theta2] + delta_time*der2;
    [states3_new] = [x3; y3; theta3] + delta_time*der3;

    h = human_integrate(h,j,delta_time);

    ev_states1(:,j,i) = states1_new;
    ev_states2(:,j,i) = states2_new;
    ev_states3(:,j,i) = states3_new;
    ev_statesh(:,j,i) = h.simX(j, 1:3);

    %evaluate irregular octagon

    [x_v12, y_v12] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], states1_new(3), states2_new(3));
    [x_v13, y_v13] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], states1_new(3), states3_new(3));
    [x_v23, y_v23] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], states2_new(3), states3_new(3));

    [x_v1h, y_v1h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], states1_new(3), h.simX(j,3));
    [x_v2h, y_v2h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], states2_new(3), h.simX(j,3));
    [x_v3h, y_v3h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], states3_new(3), h.simX(j,3));

    %ego1 vs ego2
    if ttcs(i,1) < 0
        if inpolygon(states2_new(1), states2_new(2), x_v12+states1_new(1), y_v12+states1_new(2))
            ttcs(i,1) = time(j);
        end
    end

    %ego1 vs ego3
    if ttcs(i,2) < 0
        if inpolygon(states3_new(1), states3_new(2), x_v13+states1_new(1), y_v13+states1_new(2))
            ttcs(i,2) = time(j);
        end
    end
    
    %ego2 vs ego3
    if ttcs(i,3) < 0
        if inpolygon(states3_new(1), states3_new(2), x_v23+states2_new(1), y_v23+states2_new(2))
            ttcs(i,3) = time(j);
        end
    end

    %ego1 vs hum
    if ttcs(i,4) < 0
        if inpolygon(h.simX(j, 1), h.simX(j, 2), x_v1h+states1_new(1), y_v1h+states1_new(2))
            ttcs(i,4) = time(j);
        end
    end

    %ego2 vs hum
    if ttcs(i,5) < 0
        if inpolygon(h.simX(j, 1), h.simX(j, 2), x_v2h+states2_new(1), y_v2h+states2_new(2))
            ttcs(i,5) = time(j);
        end
    end

    %ego3 vs hum
    if ttcs(i,6) < 0
        if inpolygon(h.simX(j, 1), h.simX(j, 2), x_v3h+states3_new(1), y_v3h+states3_new(2))
            ttcs(i,6) = time(j);
        end
    end
    
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
    clc
    disp(['Time istant ' num2str(i) ' out of ' num2str(iter)]);
end
ttcs(ttcs<0) = inf;

ev_states.ev_states1 = ev_states1;
ev_states.ev_states2 = ev_states2;
ev_states.ev_states3 = ev_states3;
ev_states.ev_statesh = ev_statesh;



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
