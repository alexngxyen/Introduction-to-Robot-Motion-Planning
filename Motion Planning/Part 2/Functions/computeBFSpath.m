function P = computeBFSpath(AdjTable, start, goal)
%=========================================================================%
% computeBFSpath outputs a path from start to goal along the BFS tree 
% rooted at start. Here, we assume the graph is fully-connected.
%
%   Inputs:
%     start  -  Start node
%      goal  -  Goal node
%  AdjTable  -  Graph described by an adjacency table 
%
%  Outputs:    
%         P  -  Path from start to goal along the BFS tree rooted at start                 
%=========================================================================%

% Run computeBFStree Function
[pVec, pNodes] = computeBFStree(AdjTable, start);

% Parents Vector
parents(pNodes(1)) = start;
parents(pNodes(2:end)) = pVec(2:end);

% Extract-Path Algorithm
P = [goal];               % Path Array
u = goal;                 % Node u

while parents(u) ~= u
    % Work Backwards from Goal to Start 
    u = parents(u);
    P = [u P];
end

end






