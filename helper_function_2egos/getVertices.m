function vertices = getVertices(x, y, angle, D1, D2)

diag = sqrt((D1/2)^2 + (D2/2)^2);

alpha = atan(D2/D1);

% the first vertice is the top right one with the car facing right, then
% they are numbered with clockwise rotation. 

x1 = x + diag*cos(alpha+angle);
x2 = x + diag*cos(-alpha+angle);
x3 = x + diag*cos(alpha+angle+pi);
x4 = x + diag*cos(-alpha+angle+pi);

y1 = y + diag*sin(alpha+angle);
y2 = y + diag*sin(-alpha+angle);
y3 = y + diag*sin(alpha+angle+pi);
y4 = y + diag*sin(-alpha+angle+pi);

vertices = [x1 y1; 
            x2 y2; 
            x3 y3; 
            x4 y4];
end