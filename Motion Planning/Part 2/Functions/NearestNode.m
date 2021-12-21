function [Path, ind] = NearestNode(V, pt, O)
% NearestNode finds the line parameters to the nearest node in the free
% workspace by a straight line.


if iscell(V) == 1
    V = cell2mat(V);
end

% Distance Vector
dV_pt = V - pt; 
D = sqrt(dV_pt(:, 1).^2 + dV_pt(:, 2).^2);

% Sort in Ascending Order
[~, i] = sort(D, 'ascend');
ii = 0;
in = inf;

% Check in Free Workspace
while in ~= 0
    % Update Counter
    ii = ii + 1;
    
    % Node Point
    ptV = V(i(ii), :);
    
    % Line Parameters
    [a, b, c] = computeLineThroughTwoPoints(ptV, pt);
    m = -a/b; y_int = -c/b;
    
    % Construct Line
    if abs(m) == inf
        y = linspace(min(pt(2), ptV(2)), max(pt(2), ptV(2)))';
        x = pt(1)*ones(N, 1);

    else
        x = linspace(pt(1), ptV(1))';
        y = m*x + y_int;
    end
       
    % Line Intersects an Obstacle?
    S = 0;
    
    for k = 1:length(O)        
        for jj = 1:length(x)
            
            S = inpolygon(x(jj), y(jj), O{k}(:, 1), O{k}(:, 2)) + S;
            
        end               
    end

    % Check Indicator
    in = S;
    
    if ii == length(i) && in ~= 0
        error('No Linear Path to Nearest Node.');
    end
    
end   

% Path 
Path = [x, y];

% Node Index
ind = i(ii);

end

function [a, b, c] = computeLineThroughTwoPoints(p1, p2)
% computeLineThroughTwoPoints returns equation for the line through two points.
% [a,b,c] = computeLineThroughTwoPoints(p1,p2)
% Function takes two arguments, p1 = [x1,y1] and p2 = [x2,y2],
% and returns [a,b,c] corresponding to the line equation
% ax + by + c = 0. Error when p1 = p2.

tolerance = 0.1^7;
distance = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
if distance < tolerance
    error('Cannot compute line, points are too close together');
end

a = p1(2) - p2(2);
b = p2(1) - p1(1);
c = p1(1)*p2(2) - p2(1)*p1(2);
 
%Line definition does not change with re-scaling
a = a / distance; 
b = b / distance; 
c = c / distance;
end
