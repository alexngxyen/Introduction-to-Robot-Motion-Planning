function [outvar] = computeDistancePointToSegment(q, p1, p2)
 % computeDistancePointToSegment returns the distance between point q
 % and the segment between points p1 and p2. It also returns a variable wP
 % which tells if the closest point is on the segment (wP = 0), is closest
 % to the first point (wP = 1) or the second (wP = 2).

 % Obtain the parameters defining the line passing through points p1 and p2
[a,b,c]=computeLineThroughTwoPoints(p1, p2);
%the points should be inputed as column vector
 %We find the orthogonal projection of point q on the line that goes through points p1 and p2
 %see the InfoQuiz: Geometry-HW1 for the formula
 q_x=q(1); q_y=q(2);
 
 %to turn all the points into a vector represenation
 p_1(1,1)=p1(1); p_1(2,1)=p1(2);

 p_2(1,1)=p2(1); p_2(2,1)=p2(2);
 
 qq(1,1)=q(1); qq(2,1)=q(2);
 
 q_prep=[-(a*c+b*(-b*q_x+a*q_y))/(a^2+b^2) 
         -(b*c+a*(b*q_x-a*q_y))/(a^2+b^2)];
 %then compute distance of q_prep to each endpoint of the segment
 d_1=norm(p_1-q_prep); %finds the length of the line segement connecting p1 and q_prep
 d_2=norm(p_2-q_prep);
 d=norm(p_1-p_2);
 if max(d_1,d_2)<=d
     wP = 0; %the closet point is on the seqment
     q_dist_to_segment=norm(qq-q_prep);
 elseif d_2>d
     wP = 1; %the closest point is the first point of the segment (wP = 1)
     q_dist_to_segment=norm(qq-p_1);
 else
     wP = 2; %the closest point is the first point of the segment (wP = 2)
     q_dist_to_segment=norm(qq-p_2);
 end

% If the projection line is between the endpoints then return the distance
% to line otherwise return distance to closest endpoint.


outvar = [q_dist_to_segment,wP];

end