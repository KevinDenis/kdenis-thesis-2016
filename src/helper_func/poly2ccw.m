function [x,y] = poly2ccw(x,y)
%[x,y] = poly2ccw(x,y)

% Step 1: Find the centroid:
cx = mean(x);
cy = mean(y);

% Step 2: Find the angles:
a = atan2(y - cy, x - cx);

%Step 3: Find the correct sorted order:
[~, order] = sort(a);


% Step 4: Reorder the coordinates:
x = x(order);
y = y(order);

end

