function [x_pt, y_pt] = LineCentroid(pt_x, pt_y)
% LineCentroid calculates the center of a line given by two points.


% Number of Points
N = length(pt_x(:, 1));

% Preallocate
x_pt = zeros(N, 1);
y_pt = x_pt;

% Center
for ii = 1:N
    x_pt(ii) = mean([pt_x(ii), pt_y(ii)]);
    y_pt(ii) = mean([pt_x(ii), pt_y(ii)]);
end

end