function path = circumnavigate(initial_position, goal_position, obstacle, step_size)
% circumnavigate outputs the path of a robot traversing the polygon edges. 

    not_circumnavigated = 0;

    path = [initial_position];
    tangent_vector = computeTangentVectorToPolygon(initial_position, obstacle);
    position = initial_position + (tangent_vector*step_size);
    path = [path; position];
    dist_goal = [norm(initial_position - goal_position)];
    dist_point = position;
    
    counter = 0;
    while not_circumnavigated == 0
        
        disp(counter);
        tangent_vector = computeTangentVectorToPolygon(position, obstacle);
        position = position + (tangent_vector*step_size);
        path = [path; position];
        dist_goal = [dist_goal, norm(position - goal_position)];
        dist_point = [dist_point ; position];
        
        if norm(position - initial_position) < step_size * 5
            if counter > 50
                not_circumnavigated = 1;
            end
        end
        
        counter = counter + 1;
        
    end

    position = initial_position;
    path = [path; position];
    [~,dist_index] = min(dist_goal);
    min_dist_point = dist_point(dist_index, :);
    disp(norm(position - min_dist_point));
    
    while norm(position - min_dist_point) > step_size * 3.5
        
        tangent_vector = computeTangentVectorToPolygon(position, obstacle);
        position = position + (tangent_vector*step_size);
        path = [path; position];
        
    end

    hold on
    fill(obstacle(:, 1), obstacle(:, 2), 'k');
    plot(path(:,1), path(:,2), '-r', 'linewidth', 2)
    hold off

end
