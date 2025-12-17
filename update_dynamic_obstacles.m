% This function is used to update the all given position and velocity information of dynamical obstacles
% Assumptions: 
% - All obstacles are circular
% - There is no acceleration on the obstacles

function dyn_obs = update_dynamic_obstacles(dyn_obs, room, Ts)

% Use the room data in order to collision/bounce phenomenon
xmin = room(1); xmax = room(2);
ymin = room(3); ymax = room(4);

    for i = 1:length(dyn_obs)
    
        % Extract the Position and the Velocity data of each obstacle
        position  = dyn_obs{i}{2};   % [cx cy r]
        velocity  = dyn_obs{i}{3};   % [vx vy]
    
        cx = position(1);  cy = position(2);  r = position(3);
        vx = velocity(1);   vy = velocity(2);
    
        % Basic Physics: pos_new = pos_curr + velocity*time 
        % Position next step
        cx_next = cx + vx*Ts;
        cy_next = cy + vy*Ts;
    
        % If obstacle hit the vertical walls in the room:
        if cx_next - r < xmin || cx_next + r > xmax
            vx = -vx;
            cx_next = cx + vx*Ts;
        end
    
        % If obstacle hit the horizontal walls in the room:
        if cy_next - r < ymin || cy_next + r > ymax
            vy = -vy;
            cy_next = cy + vy*Ts;
        end
    
        % Updating Position and Velocity of the Obstacles
        dyn_obs{i}{2} = [cx_next, cy_next, r];
        dyn_obs{i}{3} = [vx, vy];
    
    end


end
