function rel_distances = plotRelDist(simX, simX_hdv, model, t)

num_iter= size(simX{1}, 1);
rel_distances = zeros(num_iter, 6);

for i=1:num_iter
    vertices1 = getVertices(simX{1}(i,1), simX{1}(i,2), simX{1}(i,3), model.params.D1, model.params.D2);
    vertices2 = getVertices(simX{2}(i,1), simX{2}(i,2), simX{2}(i,3), model.params.D1, model.params.D2);
    vertices3 = getVertices(simX{3}(i,1), simX{3}(i,2), simX{3}(i,3), model.params.D1, model.params.D2);
    verticesHum = getVertices(simX_hdv(i,1), simX_hdv(i,2), simX_hdv(i,3), model.params.D1_h, model.params.D2_h);
  
    % dist of 1 and 2
    rel_distances(i,1) = getDistance(vertices1, vertices2);
    
    % dist of 1 and 3
    rel_distances(i,2) = getDistance(vertices1, vertices3);

    % dist of 2 and 3
    rel_distances(i,3) = getDistance(vertices2, vertices3);

    % dist of 1 and hum
    rel_distances(i,4) = getDistance(vertices1, verticesHum);

    % dist of 2 and hum
    rel_distances(i,5) = getDistance(vertices2, verticesHum);

    % dist of 3 and hum
    rel_distances(i,6) = getDistance(vertices3, verticesHum);
end

figure
subplot(3,1,1)
plot(t, rel_distances(:,1))
hold on
plot(0, 0)
xlabel('[s]')
ylabel('[m]')
grid on 
title('Relative distance between ego1 and ego2')
subplot(3,1,2)
plot(t, rel_distances(:,2))
hold on
plot(0, 0)
xlabel('[s]')
ylabel('[m]')
grid on 
title('Relative distance between ego1 and ego3')
subplot(3,1,3)
plot(t, rel_distances(:,3))
hold on
plot(0, 0)
xlabel('[s]')
ylabel('[m]')
grid on 
title('Relative distance between ego2 and ego3')

figure
subplot(3,1,1)
plot(t, rel_distances(:,4))
hold on
plot(0, 0)
xlabel('[s]')
ylabel('[m]')
grid on 
title('Relative distance between ego1 and hdv')
subplot(3,1,2)
plot(t, rel_distances(:,5))
hold on
plot(0, 0)
xlabel('[s]')
ylabel('[m]')
grid on 
title('Relative distance between ego2 and hdv')
subplot(3,1,3)
plot(t, rel_distances(:,6))
hold on
plot(0, 0)
xlabel('[s]')
ylabel('[m]')
grid on 
title('Relative distance between ego3 and hdv')
