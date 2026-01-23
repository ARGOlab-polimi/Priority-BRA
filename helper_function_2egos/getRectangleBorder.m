function [V_border] = getRectangleBorder(xc, yc, d1, d2, theta)
% Evaluate 100 points on a border of a rotated rectangle

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

    % matrix assembly
    V = [x1, y1;
         x2, y2;
         x3, y3;
         x4, y4];

    n_points_total = 100;

    perimeter = 2 * d1 + 2 * d2;
    if perimeter == 0
        V_border = repmat([xc, yc], n_points_total, 1);
        return;
    end
    lengths_cumulative = [0, d1, d1 + d2, 2 * d1 + d2, perimeter];
    frac_cumulative = lengths_cumulative / perimeter;
    points_cumulative = round(frac_cumulative * n_points_total);
    n_side = diff(points_cumulative); 
  
    x_p1 = linspace(V(1,1), V(2,1), n_side(1) + 1);
    y_p1 = linspace(V(1,2), V(2,2), n_side(1) + 1);
    p1 = [x_p1(1:end-1)', y_p1(1:end-1)']; 
    
    x_p2 = linspace(V(2,1), V(3,1), n_side(2) + 1);
    y_p2 = linspace(V(2,2), V(3,2), n_side(2) + 1);
    p2 = [x_p2(1:end-1)', y_p2(1:end-1)'];
    
    x_p3 = linspace(V(3,1), V(4,1), n_side(3) + 1);
    y_p3 = linspace(V(3,2), V(4,2), n_side(3) + 1);
    p3 = [x_p3(1:end-1)', y_p3(1:end-1)']; 

    x_p4 = linspace(V(4,1), V(1,1), n_side(4) + 1);
    y_p4 = linspace(V(4,2), V(1,2), n_side(4) + 1);
    p4 = [x_p4(1:end-1)', y_p4(1:end-1)']; 
    
    V_border = [p1; p2; p3; p4];
    
end