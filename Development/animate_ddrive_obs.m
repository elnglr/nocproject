% Differential-drive robot animation with obstacles and multiple goal points.

% INPUTS:
    % traj: State trajectory [x, y, theta, omega_r, omega_l]
    % params: Struct with robot parameters L= Half of Wheel Base, r = Radius of Wheels
    % Ts: Sampling Time
    % obs: Cell type variable containing obstacles information
    %               Example: obs = { {'circle',[3 3 0.6]}, 
    %                               {'circle',[6 2 0.8]} };
                % So far we only worked with circles ; thus , rectangular shape
                % is not defined in our problem yet.
    % goals:   Target Points positions -> each row = [x_goal, y_goal]
    % room:    [xmin xmax ymin ymax]
    % fastscale: drawing speed factor (1 = normal, 10 = very fast)
    % goal_radius: Tolerance between center of target point and COG of Robot%
    % robot_shape: Mobile Robot's shape in terms of "Vertical" or "Horizontal"

% OUTPUT:
    % Simulation itself in the main.m file


function animate_ddrive_obs(traj, params, Ts, obs, goals, room, fastscale, goal_radius,robot_shape)

    
    % Main Figure Setup
    figure; hold on; grid on; axis equal;

    title('Differential-drive Robot Animation');
    xlabel('x [m]'); ylabel('y [m]');
    
    % Room Setup
    xlim([room(1), room(2)]);
    ylim([room(3), room(4)]);
    rectangle('Position',[room(1), room(3), room(2)-room(1), room(4)-room(3)],...
              'EdgeColor','k','LineWidth',3);

    % Plotting all goals given by the user
    for i = 1:size(goals,1)
        plot(goals(i,1), goals(i,2), 'go', 'MarkerSize', 10, ...
            'MarkerFaceColor', 'g', 'DisplayName', sprintf('Goal %d', i));
    end


    % Plotting all obstacles given by the user

    for i = 1:length(obs)
        o = obs{i};
        if strcmpi(o{1}, 'circle')

            % Center x , Center y coordinates and Radius
            cx = o{2}(1); cy = o{2}(2); r = o{2}(3);
            % Easy way to put circle on the simulation but User must have
            % Image Processing Toolbox !!!
            viscircles([cx,cy], r, 'Color', 'k');

        elseif strcmpi(o{1}, 'rect')

            % Initial x ,  y coordinates and Width , Height
            x = o{2}(1); y = o{2}(2); w = o{2}(3); h = o{2}(4);
            rectangle('Position',[x, y, w, h], 'FaceColor',[0.5 0.5 0.5], 'EdgeColor','k');

        end
    end

    % --- Time display text ---
    time_text = text(room(1)+0.3, room(4)-0.3, 'Time: 0.00 s', ...
                     'FontSize', 12, 'FontWeight', 'bold', ...
                     'Color', [0.1 0.1 0.5], 'BackgroundColor', [1 1 1 0.7], ...
                     'Margin', 4);

    % Robot Parameters
    Ltot = 2 * params.L;
    W = params.r * 4;
    robot_radius = sqrt( 2*(Ltot/2)^2 );
    goal_tolerance = goal_radius;

    % Robot's Safety Circular Area for preventing to hit the obstacles
    theta_circle = linspace(0,2*pi,50);
    robot_circle_x = robot_radius*cos(theta_circle);
    robot_circle_y = robot_radius*sin(theta_circle);


    % Initialize Dynamic Vectors
    traj_line = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
    COGvel = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Velocity Vector');

    robot_body = plot(NaN, NaN, 'k-', 'LineWidth', 2,'DisplayName','Robots Body');
    robot_circle_plot = plot(NaN, NaN, 'b--','LineWidth',1.5,'DisplayName','Robots Safety Circle');
    legend('show','Location','bestoutside');

    current_goal_idx = 1;
    goal_reached = false;

    % Animation Loop starts here
    for k = 1:size(traj,2)

        % The first 3 states of the Robot Dynamics xpos, ypos, theta of COG
        xpos = traj(1,k);
        ypos = traj(2,k);
        theta = traj(3,k);


        % X and Y components of Linear Velocity(v) of the Robot
        v = (params.r/2) * (traj(4,k) + traj(5,k));
        vx = v * cos(theta);
        vy = v * sin(theta);

        % Trajectory line
        set(traj_line, 'XData', traj(1,1:k), 'YData', traj(2,1:k));

        % Robot body: These are 4 points on the corners to define
        % Rectangular Shape
        
        if robot_shape == "Vertical"
        corners = [-Ltot/2, -W/2;
                    Ltot/2, -W/2;
                    Ltot/2,  W/2;
                   -Ltot/2,  W/2]';


        elseif robot_shape == "Horizontal"
                corners = [-W/2, -Ltot/2;
                    W/2, -Ltot/2;
                    W/2,  Ltot/2;
                   -W/2,  Ltot/2]';
        end

        % Rotation matrix to get the coordinate of each corners of robot
        % while rotating on "2D" !

        R = [cos(theta) -sin(theta);
             sin(theta)  cos(theta)];

        rotated = R * corners;

        % Update on Robot's body motion after one sample time
        bodyX = rotated(1,:) + xpos;
        bodyY = rotated(2,:) + ypos;

        % Dynamics of the Robot's Body
        set(robot_body, 'XData', [bodyX bodyX(1)], 'YData', [bodyY bodyY(1)]);
      
        % Safety Circular Area Dynamics of the Robot
        set(robot_circle_plot, 'XData', xpos + robot_circle_x, ...
                               'YData', ypos + robot_circle_y);
        
        % Velocity vector of COG Scaling for better visualising
        scale = 0.3;
        set(COGvel, 'XData', xpos, 'YData', ypos, 'UData', vx*scale, 'VData', vy*scale);
        % --- Update time text dynamically ---
        set(time_text, 'String', sprintf('Time: %.2f s', k*Ts));

        % Collision Check
        % Check whether the robot hit the obstacle or not
        hit = false;
        for i = 1:length(obs)
            o = obs{i};
            if strcmp(o{1}, 'circle') % The obstacle is Circular Type
                cx=o{2}(1); cy=o{2}(2); r=o{2}(3);
                 % This is the criterion for the violence of safety region
                 % and hit the obstacle
                if norm([xpos-cx, ypos-cy]) <= robot_radius + r
                    hit = true; break;
                end
            elseif strcmp(o{1}, 'rect') % The obstacle is Rectangular Type
                x1=o{2}(1); y1=o{2}(2); w=o{2}(3); h=o{2}(4);
                 % This is the criterion for the violence of safety region
                 % and hit the obstacle
            end
        end


        %% Print on the Animation if goal is achieved
        if hit
            text(mean(room(1:2)), mean(room(3:4)), ...
                 'Obstacle hit!', 'Color', 'r', 'FontSize', 14, ...
                 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            drawnow; break;
        end

        % 
        goal = goals(current_goal_idx,:);

        dist_to_goal = sqrt((xpos - goal(1))^2 + (ypos - goal(2))^2);
        if dist_to_goal <= goal_tolerance
            text(goal(1), goal(2)+0.1, sprintf('Goal %d reached!', current_goal_idx), ...
                 'Color','g','FontSize',10,'FontWeight','bold','HorizontalAlignment','center');
            current_goal_idx = current_goal_idx + 1;

            if current_goal_idx > size(goals,1)
                goal_reached = true;
                text(mean(room(1:2)), mean(room(3:4)), ...
                     'All goals reached!', 'Color','g','FontSize',14, ...
                     'FontWeight','bold','HorizontalAlignment','center');
                drawnow; break;
            end
        end
        %%

        % Fastscale: 1 (Normal) --> 10 (Faster)
        if mod(k, fastscale) == 0
            drawnow;
            pause(Ts);
        end
    end

    if ~goal_reached
        disp('Simulation ended before all goals were reached.');
    end
end
