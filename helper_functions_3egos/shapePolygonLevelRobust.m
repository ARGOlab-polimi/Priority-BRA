function  [x_v, y_v] = shapePolygonLevelRobust(dim1, dim2, theta1, theta2)

xy_v_ego = getVerticesRect(0, 0, dim1(1), dim1(2), theta1);
x_v_ego = xy_v_ego(:,1);
y_v_ego = xy_v_ego(:,2);

diag = (sqrt(dim2(1)^2+dim2(2)^2))/2;

angle_hum = atan((dim2(2)/2)/(dim2(1)/2));

if theta2 < 0
    theta2 = theta2 + 2*pi;
end

angle = angle_hum + theta2;

x_c_hum = [cos(angle+pi)*diag cos(1.5*pi+theta2+(0.5*pi-angle_hum))*diag ...
              cos(angle)*diag cos(0.5*pi+theta2+0.5*pi-angle_hum)*diag];

y_c_hum = [sin(angle+pi)*diag sin(1.5*pi+theta2+(0.5*pi-angle_hum))*diag ...
              sin(angle)*diag sin(0.5*pi+theta2+0.5*pi-angle_hum)*diag];

x_v = [];
y_v = x_v;

for i=1:4
    x_r = x_v_ego(i)+x_c_hum;
    y_r = y_v_ego(i)+y_c_hum;

    for j=1:4
    xy_vert_rect = getRectangleBorder(x_r(j), y_r(j), dim2(1), dim2(2), theta2);
    
    [in, on] = inpolygon(xy_vert_rect(:,1), xy_vert_rect(:,2), x_v_ego, y_v_ego);

    if numel(xy_vert_rect(in)) <= numel(xy_vert_rect(on)) 
        x_v = [x_v, x_r(j)];
        y_v = [y_v, y_r(j)];
    end
   
    end
end

l = length(x_v);
pos = zeros(1,l);
x_v_abs = abs(x_v);
y_v_abs = abs(y_v);

for k=1:l
    if  x_v_abs(k) < 0.01 && y_v_abs(k) < 0.01
        pos(k) = 1;
    end
end

pos = pos > 0;

x_v(pos) = [];
y_v(pos) = [];

xy_v = [x_v; y_v];

xy_v_new = [0; 0];

for f=1:length(xy_v)
    rep = sqrt(sum((xy_v_new-xy_v(:,f)).^2, 1));
    pos = (rep>0.01);
    if all(pos)
       xy_v_new = [xy_v_new, xy_v(:,f)];
    end
end

xy_v_new(:,1) = [];

x_v = xy_v_new(1,:);
y_v = xy_v_new(2,:);

x_v = x_v(:);
y_v = y_v(:);

cx = mean(x_v);
cy = mean(y_v);

angles = atan2(y_v - cy, x_v - cx);

[~, ind] = sort(angles);

x_v = x_v(ind);
y_v = y_v(ind);

end




