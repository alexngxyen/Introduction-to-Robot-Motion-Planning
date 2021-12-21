function path = Bug1(start, goal, obstaclesList, step_size)
% Bug1 outputs the path from start to goal for the Bug 1 algorithm. Here,
% we assume a path is always obtainable so no stopping criteria is added. 

	current_position = start;
    path = [start];
    tolerance = 0.1;
    num_obst_navigated = 0;

    while norm(current_position - goal) > step_size 

        unit_vector_to_goal = (goal - current_position) / norm(goal - current_position);
        current_position = current_position + (unit_vector_to_goal * step_size);

        for i = 1:length(obstaclesList) 

            distance_to_obstacle = computeDistancePointToPolygon(current_position, obstaclesList{i});
            distance_to_obstacle = distance_to_obstacle(1);
            
            if(distance_to_obstacle < tolerance)
                
                c_path = circumnavigate(current_position, goal, obstaclesList{i}, step_size);
                path = [path; c_path];
                current_position = c_path(max(length(c_path)), :);
                num_obst_navigated = num_obst_navigated + 1;
                
            end
            
        end
        
        path = [path;current_position];
        disp(current_position); 

    end

    path = [path;goal];
    disp("success!");

end