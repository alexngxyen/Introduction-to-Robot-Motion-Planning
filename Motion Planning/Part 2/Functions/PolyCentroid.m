function [x_bar, y_bar] = PolyCentroid(Poly)
% PolyCentroid calculates the centroid of a polygon given all its vertices.


% Vertices
Vert_x = Poly(:, 1); Vert_y = Poly(:, 2);

% Polygon Centroid
polyin = polyshape({Vert_x}, {Vert_y});
[x_bar, y_bar] = centroid(polyin);

end