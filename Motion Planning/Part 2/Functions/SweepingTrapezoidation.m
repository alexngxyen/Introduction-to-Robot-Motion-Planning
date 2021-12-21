function [trapezoids, segments] = SweepingTrapezoidation(Polygon)
% SweepingTrapezoidation outputs the a set of disjoint trapezoids and each
% line segment for each vertex (if applicable).


P = [];
for ii = 1:length(Polygon)
    
    P = [P; Polygon{ii}];
    
end

maxX = 500; maxY = 500;
T = [];
intersect = [];

L1 = [0, maxY]; L2 = [0, 0]; 
sStart = [L1, [maxX, maxY]];
sEnd = [L2, [maxX, 0]];
S = sStart;

X = P(:,1);
Y = P(:,2);
[sortedX, sortIndex] = sort(X);
sortedY = Y(sortIndex);
sortedP = [sortedX, sortedY];
sortedP(10, :) = [];
sortedP(12, :) = [];
sortedP(15, :) = [];

Vtype = [1, 5, 5, 5, 1, 1, 5, 3, 5, 5, 5, 2, 5, 3, 1, 5, 5, 5, 5, ...
         6, 3, 5, 5, 5, 3, 5, 3];
     
P1 = Polygon{1};
P2 = Polygon{2};
P3 = Polygon{3};
P4 = Polygon{4};

for iii = 1:length(P1)
    
    if iii == length(P1)
        S1 = [S1; [[P1(iii, 1), P1(iii, 2)], [P1(1, 1), P1(1, 2)]]];
    else
        S1(iii + 1, :) = [[P1(iii, 1), P1(iii, 2)], [P1(iii + 1, 1), P1(iii + 1, 2)]];
    end
    
end

for iii = 1:length(P2)

    if iii == length(P2)
        S2 = [S2; [[P2(iii, 1), P2(iii, 2)], [P2(1, 1), P2(1, 2)]]];
    else
        S2(iii + 1, :) = [[P2(iii, 1), P2(iii, 2)], [P2(iii + 1, 1), P2(iii + 1, 2)]];
    end
    
end

for iii = 1:length(P3)
    
    if iii == length(P3)
        S3 = [S3; [[P3(iii, 1), P3(iii, 2)], [P3(1, 1), P3(1, 2)]]];
    else
        S3(iii + 1, :) = [[P3(iii, 1), P3(iii, 2)], [P3(iii + 1, 1), P3(iii + 1, 2)]];
    end
    
end

for iii = 1:length(P4)
    
    if iii == length(P4)
        S4 = [S4; [[P4(iii, 1), P4(iii, 2)], [P4(1, 1), P4(1, 2)]]];
    else
        S4(iii + 1, :) = [[P4(iii, 1), P4(iii, 2)], [P4(iii + 1, 1), P4(iii + 1, 2)]];
    end
    
end

S = [S; S1; S2; S3; S4; sEnd];

for i = 1:length(sortedP)
    SweepLine = [[sortedP(i, 1), maxY], [sortedP(i, 1), 0]];
    interpoint =[];
    
    for ii = 1:size(S,1)
        interpoint(ii, :) = getIntersectionPoint([S(ii, 1), SweepLine(1, 1); ...
            S(ii, 3), SweepLine(1, 3)], [S(ii, 2), SweepLine(1, 2); S(ii, 4), SweepLine(1, 4)]);
    end
    
    interpoint = sort(interpoint);
    intersect = [intersect; [interpoint(:, 2)]'];
    
end

for i = 1:length(sortedP)
    for ii = 1:length(intersect)
        
        if sortedP(i, 2) == intersect(i, ii)
            intersect(i, ii) = NaN;
            
        end
    end
end

u = length(intersect);
while size(intersect, 2) > 9
    
    intersect(:, u) = [];
    u = u - 1;
    
end

PBT = [];

for i = 1:length(sortedP)
    for r = 1:(size(intersect, 2) - 2)
        
        if isnan(intersect(i, r)) && isnan(intersect(i, r + 1)) && intersect(i, r + 2) > 0
            pty = intersect(i, r + 2);
            pby = intersect(i, r - 1);
            PBT = [PBT; pty, pby];
        end
        
    end
end

for i = 1:length(sortedP)
    
    if Vtype(i) == 1
        segments{i} = [[sortedP(i, 1), PBT(i, 1); sortedP(i, 1), sortedP(i, 2)]; ...
            [sortedP(i, 1), sortedP(i, 2); sortedP(i, 1), PBT(i, 2)]];
    end
    
    if Vtype(i) == 2
        segments{i} = [sortedP(i, 1), sortedP(i, 2); sortedP(i, 1), sortedP(i, 2)];
    end
    
    if Vtype(i) == 3
        segments{i} = [[sortedP(i, 1), PBT(i, 1); sortedP(i, 1), sortedP(i, 2)]; ...
            [sortedP(i, 1), sortedP(i, 2); sortedP(i, 1), PBT(i, 2)]];
    end
    
    if Vtype(i) == 4
        segments = {i; [PBT(i, 1), PBT(i, 2)]};
    end
    
    if Vtype(i) == 5
        if i > 1
            b = i - 1;
            f = i + 1;
        end
        
        while 1
            if b == 0
                break
            end
            
            Test1 = [sortedP(i, :), sortedP(b, :)];
            for s = 1:length(S)
                xTemp = [S(s, 1), Test1(1, 1); S(s, 3), Test1(1, 3)];
                yTemp = [S(s, 2), Test1(1, 2); S(s, 4), Test1(1, 4)];
                inter(s, :) = getIntersectionPoint(xTemp, yTemp);
            end
            
            for s = 1:length(inter)
                if sortedP(i, 2) == inter(s, 2)
                    inter(s, :) = [];
                end
                
                if sortedP(b, 2) == inter(s, 2)
                    inter(s, :) = [];
                end
                
                if s == length(inter)
                    break
                end
            end
            
            inter(any(isnan(inter), 2), :) = [];
            if size(inter, 1) == 0
                break
            end
            
            b = b - 1;
        end
        
        if i == 7
            b = b + 2;
        end
        
        if i == 18
            b = 7;
        end
        
        while 1
            Test2 = [sortedP(i, :) sortedP(f, :)];
            for s = 1:length(S)
                xTemp = [S(s, 1), Test2(1, 1); S(s, 3), Test2(1, 3)];
                yTemp = [S(s, 2), Test2(1, 2); S(s, 4), Test2(1, 4)];
                inter(s, :) = getIntersectionPoint(xTemp, yTemp);
            end
            
            inter(any(isnan(inter), 2), :) = [];
            if size(inter, 1) <= 2
                break
            end
            
            f = f + 1;
        end
        
        if sortIndex(i) == 1
            m1 = (sortedP(i - 1, 2) - P(sortIndex(i), 2)) / (sortedP(i - 1, 1) - P(sortIndex(i), 1));
            m2 = NaN;
        else
            m1 = (sortedP(i, 2) - sortedP(b, 2)) / (sortedP(i, 1) - sortedP(b, 1));
            m2 = (sortedP(f, 2) - sortedP(i, 2)) / (sortedP(f, 1) - sortedP(i, 1));
        end
        
        if sortIndex(i) == 1
            segments{i} = [sortedP(i, 1), sortedP(i, 2); sortedP(i, 1), PBT(i,2)];
        elseif (m1 > 0 && m2 < 0) || (m1 < 0 && m2 < 0) || (m1 > 0 && m2 > 0) && i ~= 13 && i ~= 23
            segments{i} = [sortedP(i, 1), PBT(i, 1); sortedP(i, 1), sortedP(i, 2)];
        elseif (m1 < 0 && m2 > 0) || i == 13 || i == 23
            segments{i} = [sortedP(i, 1), PBT(i, 2); sortedP(i, 1), sortedP(i, 2)];
        end
        
    end
    
    if Vtype(i) == 6
        segments{i} = [sortedP(i, 1), PBT(i, 1); sortedP(i, 1), sortedP(i, 2)];
    end
    
end


trapezoids = formPolygons(segments);

end


% Helper Functions
function interpoint = getIntersectionPoint(x, y)
dx = diff(x);
dy = diff(y);
den = dx(1)*dy(2) - dy(1)*dx(2);
ua = (dx(2)*(y(1) - y(3)) - dy(2)*(x(1) - x(3)))/den;
ub = (dx(1)*(y(1) - y(3)) - dy(1)*(x(1) - x(3)))/den;
xi = x(1) + ua*dx(1);
yi = y(1) + ua*dy(1);
isInSegment = all(([ua, ub] >= 0) & ([ua, ub] <= 1));
if isInSegment == true
    interpoint = [xi, yi];
else
    interpoint = [NaN, NaN];
end

end

function intersect = doTwoSegmentsIntersect (S1, S2)

intersect = false;
p1 = S1(1, 1:2);
p2 = S1(1, 3:4);

p3 = S2(1, 1:2);
p4 = S2(1, 3:4);

num1 = (p4(1) - p3(1)) * (p1*(2) - p3(2)) - (p4(2) - p3(2)) * (p1(1) - p3(1));
num2 = (p2(2) - p1(2)) * (p3*(1) - p1(1)) - (p2(1) - p1(1)) * (p3(2) - p1(2));

den1 = (p4(2) - p3(2)) * (p2*(1) - p1(1)) - (p4(1) - p3(1)) * (p2(2) - p1(2));
den2 = (p4(2) - p3(2)) * (p2*(1) - p1(1)) - (p4(1) - p3(1)) * (p2(2) - p1(2));

if den1 ~= 0
    check1 = abs(num1/den1);
    check2 = abs(num2/den2);
    if check1 >= 1 || check1 <= 0
        intersect = false;
    elseif check2 >= 1 || check2 <= 0
        intersect = false;
    else
        intersect = true;
    end
end

end

function trapezoids = formPolygons(T)

trapezoids{1} =  [T{1,1}(:,1),T{1,1}(:,2); 0,0;0, 500];
trapezoids{2} =  [T{1,2}(:,1),T{1,2}(:,2); T{1,1}(4:-1:3,:)];
trapezoids{3} =  [T{1,3}(:,1),T{1,3}(:,2); T{1,1}(2:-1:1,:)];
trapezoids{4} =  [T{1,4}(2:-1:1,:); T{1,2}(2:-1:1,:)];
trapezoids{5} =  [T{1,5}(:,1),T{1,5}(:,2); T{1,3}(2:-1:1,:)];
trapezoids{6} =  [T{1,6}(:,1),T{1,6}(:,2); T{1,4}(:,1),T{1,4}(:,2)];
trapezoids{7} =  [T{1,7}(:,1),T{1,7}(:,2); T{1,5}(4:-1:3,:)];
trapezoids{8} =  [T{1,8}(1:2,:);T{1,7}(2:-1:1,:)];
trapezoids{9} =  [T{1,8}(3:4,:);T{1,6}(2:-1:1,:)];
trapezoids{10} = [T{1,9}(1:2,:); T{1,8}(4:-1:1,:)];
trapezoids{11} = [T{1,10}(1:2,:); T{1,9}(2:-1:1,:)];
trapezoids{12} = [T{1,11}(2:-1:1,:); T{1,6}(4:-1:3,:)];
trapezoids{13} = [T{1,13}(2:-1:1,:); T{1,11}(1:2,:)];
trapezoids{14} =  [T{1,14}(1:2,:); T{1,10}(2:-1:1,:)];
trapezoids{15} =  [T{1,14}(3:4,:); T{1,13}(1:2,:)];
trapezoids{16} =  [T{1,15}(:,:); T{1,12}(1:2,:)];
trapezoids{17} =  [T{1,16}(:,:); T{1,5}(2:-1:1,:)];
trapezoids{18} =  [T{1,17}(:,:); T{1,15}(2:-1:1,:)];
trapezoids{19} =  [T{1,18}(2:-1:1,:); T{1,14}(4:-1:1,:)];
trapezoids{20} =  [T{1,19}(2:-1:1,:); T{1,15}(4:-1:3,:)];
trapezoids{21} =  [T{1,20}(:,:); T{1,19}(1:2,:)];
trapezoids{22} =  [T{1,21}(1:2,:); T{1,16}(2:-1:1,:)];
trapezoids{23} =  [T{1,21}(3:4,:); T{1,17}(2:-1:1,:)];
trapezoids{24} =  [T{1,22}(:,:); T{1,21}(4:-1:1,:)];
trapezoids{25} =  [T{1,23}(2:-1:1,:); T{1,20}(2:-1:1,:)];
trapezoids{26} =  [T{1,24}(:,:); T{1,22}(2:-1:1,:)];
trapezoids{27} =  [T{1,25}(1:2,:); T{1,24}(2:-1:1,:)];
trapezoids{28} =  [T{1,25}(3:4,:); T{1,23}(1:2,:)];
trapezoids{29} =  [T{1,26}(:,:); T{1,25}(4:-1:1,:)];
trapezoids{30} =  [T{1,27}(1:2,:); T{1,26}(2:-1:1,:)];
trapezoids{31} =  [T{1,27}(3:4,:); T{1,18}(1:2,:)];
trapezoids{32} =  [500,500; 500,0; T{1,27}(4:-1:1,:)];

end


