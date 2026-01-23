function ttcs = getTTC(simX, simX_hum, model, t)

iter = length(simX{1}(:,1));

delta_time = 0.01;

time = 0:delta_time:20;

time_it = length(time);

ttcs = -1*ones(iter, 6);

for i=1:iter
    %line ego1
    x1=@(t) simX{1}(i,1)+simX{1}(i,5)*cos(simX{1}(i,3))*t;
    y1=@(t) tan(simX{1}(i,3))*(x1(t)-simX{1}(i,1))+simX{1}(i,2);
    %line ego2
    x2=@(t) simX{2}(i,1)+simX{2}(i,5)*cos(simX{2}(i,3))*t;
    y2=@(t) tan(simX{2}(i,3))*(x2(t)-simX{2}(i,1))+simX{2}(i,2);
    %line ego3
    x3=@(t) simX{3}(i,1)+simX{3}(i,5)*cos(simX{3}(i,3))*t;
    y3=@(t) tan(simX{3}(i,3))*(x3(t)-simX{3}(i,1))+simX{3}(i,2);
    %line hum
    xh=@(t) simX_hum(i,1)+simX_hum(i,4)*cos(simX_hum(i,3))*t;
    yh=@(t) tan(simX_hum(i,3))*(xh(t)-simX_hum(i,1))+simX_hum(i,2);
    
    x1_series = x1(time);
    y1_series = y1(time);

    x2_series = x2(time);
    y2_series = y2(time);

    x3_series = x3(time);
    y3_series = y3(time);

    xh_series = xh(time);
    yh_series = yh(time);

    theta1 = simX{1}(i,3);
    theta2 = simX{2}(i,3);
    theta3 = simX{3}(i,3);
    thetah = simX_hum(i,3);

   
    % Evaluate irregular octagon
    [x_v12, y_v12] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], theta1, theta2);
    [x_v13, y_v13] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], theta1, theta3);
    [x_v23, y_v23] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1 model.params.D2], theta2, theta3);

    [x_v1h, y_v1h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], theta1, thetah);
    [x_v2h, y_v2h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], theta2, thetah);
    [x_v3h, y_v3h] = shapePolygonLevelRobust([model.params.D1 model.params.D2], ...
                                       [model.params.D1_h model.params.D2_h], theta3, thetah);

    for j=1:time_it
        %ego1 vs ego2
        if ttcs(i,1) < 0
            if inpolygon(x2_series(j), y2_series(j), x_v12+x1_series(j), y_v12+y1_series(j))
                ttcs(i,1) = time(j);
            end
        end

        %ego1 vs ego3
        if ttcs(i,2) < 0
            if inpolygon(x3_series(j), y3_series(j), x_v13+x1_series(j), y_v13+y1_series(j))
                ttcs(i,2) = time(j);
            end
        end
        
        %ego2 vs ego3
        if ttcs(i,3) < 0
            if inpolygon(x3_series(j), y3_series(j), x_v23+x2_series(j), y_v23+y2_series(j))
                ttcs(i,3) = time(j);
            end
        end

        %ego1 vs hum
        if ttcs(i,4) < 0
            if inpolygon(xh_series(j), yh_series(j), x_v1h+x1_series(j), y_v1h+y1_series(j))
                ttcs(i,4) = time(j);
            end
        end

        %ego2 vs hum
        if ttcs(i,5) < 0
            if inpolygon(xh_series(j), yh_series(j), x_v2h+x2_series(j), y_v2h+y2_series(j))
                ttcs(i,5) = time(j);
            end
        end

        %ego3 vs hum
        if ttcs(i,6) < 0
            if inpolygon(xh_series(j), yh_series(j), x_v3h+x3_series(j), y_v3h+y3_series(j))
                ttcs(i,6) = time(j);
            end
        end

    end

end
ttcs(ttcs<0) = inf;


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
