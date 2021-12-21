function [pVec, pNodes] = computeBFStree(AdjTable, start)
%=========================================================================%
% computeBFStree outputs a vector of pointer parents describing the BFS 
% tree rooted at start. Here, we assume the graph is fully-connected.
%
%   Inputs:
%        start  -  Start node
%     AdjTable  -  Graph described by an adjacency table 
%  
%  Outputs:    
%         pVec  -  Pointers for parents vector 
%       pNodes  -  Respective nodes of pointers for parents vector
%=========================================================================%

% Initialize (Unvisited) Parents Vector
parents(1:length(AdjTable)) = -1;      

% Parents Vector Start Node
parents(start) = start;

% Create an Empty Queue
Q = [];                    
Q = [Q start];

% Breadth-First-Search (BFS) Algorithm
pVec = 0;                               % Pointers for Parents Vector
pNodes = [];                            % Nodes of Pointers for Parents Vector

while ~isempty(Q)
   % First-in-First-Out (FIFO) Queue
   v = Q(1);
   Q(1) = [];
   pNodes = [pNodes v];
      
   for kk = 1:length(AdjTable{v})
       % Check Neighbors to Node v Connected by an Edge
       u = AdjTable{v}(kk);
       
       % Update the Visited Neighbors
       if parents(u) == -1
           parents(u) = v;
           Q = [Q u];
           pVec = [pVec v];
       end   
   end
end

end


