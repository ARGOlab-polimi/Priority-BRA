function [V] = getVerticesRect(xc, yc, d1, d2, theta)
    % evaluate vertices of a rotated rectangle

    w = d1 / 2;
    h = d2 / 2;
    
    c = cos(theta);
    s = sin(theta);
    
    x1 = xc + w * c - h * s;
    y1 = yc + w * s + h * c;

    x2 = xc - w * c - h * s;
    y2 = yc - w * s + h * c;

    x3 = xc - w * c + h * s;
    y3 = yc - w * s - h * c;

    x4 = xc + w * c + h * s;
    y4 = yc + w * s - h * c;

    V = [x1, y1;
         x2, y2;
         x3, y3;
         x4, y4];
end
