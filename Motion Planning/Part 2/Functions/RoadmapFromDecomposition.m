function [V, Road, AdjTable, S] = RoadmapFromDecomposition(trapezoids, segments)
% RoadmapFromDecomposition outputs a road map from the decomposed workspace
% given by the trapezoidal decomposition, each node, and the adjacency list.

% Centroids
[xT_bar, yT_bar] = PolyCentroid(trapezoids);
[xL_bar, yL_bar] = LineCentroid(segments);
xL_bar((xL_bar - 244).^2 < eps) = []; 
yL_bar((yL_bar - 308).^2 < eps) = [];

% Save Midpoints to Structure
S.T = [xT_bar, yT_bar];
S.L = [xL_bar, yL_bar];

% Label Nodes
T = [xT_bar, yT_bar]; L = [xL_bar, yL_bar];
V = LabelNodes(T, L);

% Create Adjacency List
AdjTable = GenerateAdj(V);

% Roadmap
Road = [];
ii = 1;

while ii <= length(AdjTable)
    for jj = 1:length(AdjTable{ii})
              
        pt1 = V{ii}; 
        pt2 = V{AdjTable{ii}(jj)};
               
        [a, b, c] = computeLineThroughTwoPoints(pt1, pt2);
        m = -a/b; y_int = -c/b;
        
        if abs(m) == inf
            y = linspace(min(pt1(2), pt2(2)), max(pt1(2), pt2(2)))';
            x = pt1(1)*ones(N, 1);
            
        else
            x = linspace(pt1(1), pt2(1))';
            y = m*x + y_int;
            
        end
        
        Road{ii, jj} = [x, y];
                
    end
    
    ii = ii + 1;
    
end

    
end


% Helper Functions
function [xT_bar, yT_bar] = PolyCentroid(Poly)
% PolyCentroid calculates the centroid of a polygon given all its vertices.

xT_bar = zeros(length(Poly), 1);
yT_bar = xT_bar;

for ii = 1:length(Poly)

    Vert_x = Poly{ii}(:, 1); Vert_y = Poly{ii}(:, 2);
    polyin = polyshape({Vert_x}, {Vert_y}, 'Simplify', false);
    [x, y] = centroid(polyin);
    xT_bar(ii) = x; yT_bar(ii) = y;

end

end

function [xL_bar, yL_bar] = LineCentroid(Line)
% LineCentroid calculates the center of a line given by two points.

xL_bar = [];
yL_bar = [];

for ii = 1:length(Line)
    
    pt_x = Line{ii}(:, 1); pt_y = Line{ii}(:, 2);
    N = length(pt_x(:, 1));
    
    x_pt = []; y_pt = [];
    
    if N == 4
        x_pt(1) = mean([pt_x(1:2)]);
        x_pt(2) = mean([pt_x(3:4)]);
        
        y_pt(1) = mean([pt_y(1:2)]);
        y_pt(2) = mean([pt_y(3:4)]);
        
    elseif N == 2
        x_pt(1) = mean([pt_x(1:2)]);
        
        y_pt(1) = mean([pt_y(1:2)]);
        
    end
        
    xL_bar = [xL_bar; x_pt']; 
    yL_bar = [yL_bar; y_pt']; 

end

end

function [a, b, c] = computeLineThroughTwoPoints(p1, p2)
% computeLineThroughTwoPoints returns equation for the line through two points.

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

function V = LabelNodes(T, L)
% LabelNodes gives each node a numeric value depending on its position
% relative to the left most node.

Nodes = [T; L];
V = cell(length(Nodes), 1);
V{1} = T(1, :); 
Nodes(1, :) = [];

for ii = 1:length(Nodes)
    D = sqrt(sum((V{ii} - Nodes).^2, 2));
    [~, i] = min(D);
    V{ii + 1} = Nodes(i, :);
    Nodes(i, :) = [];
end

end

function AdjTable = GenerateAdj(V)
% GenerateAdj generates the adjacency list for the workspace.
    
AdjTable = cell(length(V), 1);
AdjTable{1} = [26, 57];
AdjTable{2} = [3, 54];
AdjTable{3} = [2, 4];
AdjTable{4} = [3, 5];
AdjTable{5} = [4, 6];
AdjTable{6} = [5, 8];
AdjTable{7} = [8, 9];
AdjTable{8} = [6, 7, 29];
AdjTable{9} = [7, 10];
AdjTable{10} = [9, 11];
AdjTable{11} = [10, 12];
AdjTable{12} = [11, 13];
AdjTable{13} = [12, 14, 67];
AdjTable{14} = [13, 16];
AdjTable{15} = [14, 16];
AdjTable{16} = [15, 17];
AdjTable{17} = [16, 18];
AdjTable{18} = [17, 19];
AdjTable{19} = [18, 20];
AdjTable{20} = [19, 21];
AdjTable{21} = [20, 22, 27];
AdjTable{22} = [21, 23];
AdjTable{23} = [22, 24];
AdjTable{24} = [23, 25];
AdjTable{25} = [24, 26];
AdjTable{26} = [1, 25];
AdjTable{27} = [21, 28];
AdjTable{28} = [27, 29];
AdjTable{29} = [8, 28];
AdjTable{30} = [31, 64];
AdjTable{31} = [30, 32];
AdjTable{32} = [31, 33];
AdjTable{33} = [32, 34];
AdjTable{34} = [33, 35];
AdjTable{35} = [34, 36]; 
AdjTable{36} = [35, 37];
AdjTable{37} = [36, 42];
AdjTable{38} = [39, 66]; 
AdjTable{39} = [38, 40];
AdjTable{40} = [39, 41];
AdjTable{41} = [42, 40]; 
AdjTable{42} = [41, 43];
AdjTable{43} = [42, 44];
AdjTable{44} = [43, 45];
AdjTable{45} = [46, 44];
AdjTable{46} = [45, 47];
AdjTable{47} = [46, 48];
AdjTable{48} = [47, 49, 63];
AdjTable{49} = [48, 50];
AdjTable{50} = [49, 51];
AdjTable{51} = [50, 52];
AdjTable{52} = [51, 53];
AdjTable{53} = [52, 54];
AdjTable{54} = [2, 53, 55];
AdjTable{55} = [54, 56];
AdjTable{56} = [55, 57];
AdjTable{57} = [1, 56];
AdjTable{58} = [59, 64];
AdjTable{59} = [58, 60];
AdjTable{60} = [59, 61];
AdjTable{61} = [60, 62];
AdjTable{62} = [61, 63];
AdjTable{63} = [48, 62];
AdjTable{64} = [30, 58];
AdjTable{65} = [66, 67];
AdjTable{66} = [38, 65];
AdjTable{67} = [13, 65];

end
