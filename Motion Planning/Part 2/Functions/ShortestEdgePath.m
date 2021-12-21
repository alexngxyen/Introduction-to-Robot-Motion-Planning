function Path = ShortestEdgePath(V, NodePath, Path_S, Path_G)
% ShortestEdgePath finds the shortest path with respect to the number of
% edges. 


N = length(NodePath);
Path = [];

for ii = 1:(N - 1)
   
    pt1 = V{NodePath(ii)};
    pt2 = V{NodePath(ii + 1)}; 
    
    [a, b, c] = computeLineThroughTwoPoints(pt1, pt2);
    m = -a/b; y_int = -c/b;
    
    if abs(m) == inf
        y = linspace(min(pt1(2), pt2(2)), max(pt1(2), pt2(2)))';
        x = pt(1)*ones(N, 1);

    else
        x = linspace(pt1(1), pt2(1))';
        y = m*x + y_int;
    end
    
    Path = [Path; [x, y]];
    
end

Path = [Path_S; Path; flip(Path_G)];

end


% Helper Function
function [a, b, c] = computeLineThroughTwoPoints(p1, p2)
% computeLineThroughTwoPoints returns equation for the line through two points.
% [a,b,c] = computeLineThroughTwoPoints(p1,p2)

tolerance = 0.1^7;
distance = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
if distance < tolerance
    error('Cannot compute line, points are too close together')
end

a = p1(2) - p2(2);
b = p2(1) - p1(1);
c = p1(1)*p2(2) - p2(1)*p1(2);

%Line definition does not change with re-scaling
a = a / distance; b = b / distance; c = c / distance;

end