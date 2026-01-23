function smallestDist = getDistance(vertices1, vertices2)

smallestDist = inf;

for i=1:4
    dist1 = norm(vertices1(i,:)-vertices2(1,:));
    dist2 = norm(vertices1(i,:)-vertices2(2,:));
    dist3 = norm(vertices1(i,:)-vertices2(3,:));
    dist4 = norm(vertices1(i,:)-vertices2(4,:));

    mindist = min([dist1 dist2 dist3 dist4]);
    if mindist < smallestDist
        smallestDist = mindist;
    end
end