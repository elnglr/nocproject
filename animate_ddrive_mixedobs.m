function animate_ddrive_mixedobs(traj, params, Ts, obs, goals, room, fastscale, goal_radius, robot_shape,pred_traj, saveVideo)

    if nargin < 10
        saveVideo = false; % default: video kaydetme
    end
    if saveVideo
        vname = 'ddrive_animation.mp4'; % video dosya ismi
        writerObj = VideoWriter(vname, 'MPEG-4');
        writerObj.FrameRate = max(1, round(1 / (Ts * fastscale))); % FPS
        writerObj.Quality = 100; % Maksimum kalite
        open(writerObj);
        disp(['🎥 Video kaydı başlatıldı: ', vname]);
    end

%% Figure Setup
% figure; hold on; grid on; axis equal;
% 
% title('Differential-drive Robot Animation'); 
% xlabel('x [m]'); ylabel('y [m]');
    figure('Position', [100, 100, 1920, 1080]); % Full HD çözünürlük
    set(gcf, 'Color', 'w');
    hold on; grid on; axis equal;
    title('Differential-drive Robot Animation', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('x [m]'); ylabel('y [m]');
    set(gca, 'FontSize', 13, 'LineWidth', 1.5); box on;
% Room Setup
xlim([room(1), room(2)]); 
ylim([room(3), room(4)]);
rectangle('Position',[room(1), room(3), room(2)-room(1), room(4)-room(3)],...
          'EdgeColor','k','LineWidth',3);

%% Plot goals
    % Plotting all goals given by the user
    for i = 1:size(goals,1)
        plot(goals(i,1), goals(i,2), 'go', 'MarkerSize', 10, ...
            'MarkerFaceColor', 'g', 'DisplayName', sprintf('Goal %d', i));
    end

%% Extract obstacles
stc_obs = obs.Static;
dyn_obs = obs.Dynamic;

%% Initialize Circular and Rectangular Obstacles
for i = 1:length(stc_obs)

    o = stc_obs{i};

    if strcmpi(o{1}, 'rect')

        x = o{2}(1); y = o{2}(2); w = o{2}(3); h = o{2}(4);
        rectangle('Position',[x, y, w, h],'EdgeColor','k','LineWidth',1.5);

        % Ellipsoid Coordinates around rectangle
        cx = x + w/2; cy = y + h/2;
        a = w/2*sqrt(2); b = h/2*sqrt(2);
        th = linspace(0,2*pi,200);

        plot(cx + a*cos(th), cy + b*sin(th), 'm--','LineWidth',1.5);

    elseif strcmpi(o{1}, 'circle')

        cx = o{2}(1); cy = o{2}(2); r = o{2}(3);
        viscircles([cx,cy], r,'Color','k');

    end

end

%% Initialize Dynamic obstacles plots
theta_dyn = linspace(0,2*pi,50);
dyn_obs_plot = gobjects(length(dyn_obs),1);

for i = 1:length(dyn_obs)

    cx = dyn_obs{i}{2}(1); cy = dyn_obs{i}{2}(2); r = dyn_obs{i}{2}(3);

    dyn_x = cx + r*cos(theta_dyn);
    dyn_y = cy + r*sin(theta_dyn);

    dyn_obs_plot(i) = plot(dyn_x, dyn_y, 'r-', 'LineWidth',2);

end

%% Robot parameters

Ltot = 2*params.L; W = params.r*4;
robot_radius = sqrt(2*(Ltot/2)^2);
goal_tolerance = goal_radius;

% Robot's Safety Circular Area for preventing to hit the obstacles
theta_circle = linspace(0,2*pi,50);
robot_circle_x = robot_radius*cos(theta_circle);
robot_circle_y = robot_radius*sin(theta_circle);

%% Initialize robot graphics

traj_line = plot(NaN, NaN, 'b-', 'LineWidth',1.5, 'DisplayName','Trajectory');
COGvel = quiver(0,0,0,0, 'r','LineWidth',2,'MaxHeadSize',2,'DisplayName','Velocity Vector');

robot_body = plot(NaN,NaN,'k-','LineWidth',2,'DisplayName','Robot Body');
robot_circle_plot = plot(NaN,NaN,'b--','LineWidth',1.5,'DisplayName','Robot Safety Circle');
pred_line = plot(NaN, NaN, 'k-o', 'LineWidth', 0.5,'DisplayName','Predicted Trajectory');


current_goal_idx = 1; 
goal_reached = false;

%% Initialize obstacle closest points & tangent handles

nTot = length(stc_obs) + length(dyn_obs);
h_close = gobjects(nTot,1);
h_tan   = gobjects(nTot,1);

for i = 1:nTot
    h_close(i) = plot(NaN, NaN, 'go','MarkerFaceColor','r','MarkerSize',6); % Closest Point
    h_tan(i)   = plot([NaN NaN], [NaN NaN], 'm-','LineWidth',1.2);         % Tangent Line
end

%% Time display
time_text = text(room(1)+0.3, room(4)-0.3,'Time: 0.00 s','FontSize',12,...
                 'FontWeight','bold','Color',[0.1 0.1 0.5], 'BackgroundColor',[1 1 1 0.7],'Margin',4);


%% Animation loop
for k = 1:size(traj,2)

    % The first 3 states of the Robot Dynamics xpos, ypos, theta of COG
    xpos = traj(1,k); 
    ypos = traj(2,k); 
    theta = traj(3,k);

    % X and Y components of Linear Velocity(v) of the Robot
    v = (params.r/2) * (traj(4,k) + traj(5,k));
    vx = v * cos(theta);
    vy = v * sin(theta);

    % Robot Trajectory line
    set(traj_line,'XData',traj(1,1:k),'YData',traj(2,1:k));

 % Robot body: These are 4 points on the corners to define
 % Rectangular Shape
        
 if robot_shape == "Vertical"
 corners = [-Ltot/2, -W/2;
             Ltot/2, -W/2;
             Ltot/2,  W/2;
            -Ltot/2, W/2]';


 elseif robot_shape == "Horizontal"
 corners = [-W/2, -Ltot/2;
             W/2, -Ltot/2;
             W/2,  Ltot/2;
            -W/2,  Ltot/2]';
 end

    % Rotation matrix to get the coordinate of each corners of robot
    % while rotating on "2D" !
    % Rotation Matrix
    R = [cos(theta) -sin(theta); 
        sin(theta) cos(theta)];

    rotated = R*corners;

    % Update on Robot's body motion after one sample time
    bodyX = rotated(1,:) + xpos;
    bodyY = rotated(2,:) + ypos;
   
    % Dynamics of the Robot's Body
    set(robot_body,'XData',[bodyX bodyX(1)], 'YData',[bodyY bodyY(1)]);

    % Safety Circular Area Dynamics of the Robot
    set(robot_circle_plot, 'XData', xpos + robot_circle_x, ...
                           'YData', ypos + robot_circle_y);

    % Velocity vector of COG Scaling for better visualising
    scale = 0.3;    
    set(COGvel,'XData',xpos,'YData',ypos,'UData',vx*scale,'VData',vy*scale);



     set(pred_line, 'XData', pred_traj(1,:,k), 'YData', pred_traj(2,:,k));

    % Time text
    set(time_text,'String',sprintf('Time: %.2f s', k*Ts));

    %% 
    obs_counter = 1;
    
    % Check the Detection radiues for considering specific obstacles
    if isempty(params.detect_radius)
        detect_r = 3*robot_radius; 
    else
        detect_r = params.detect_radius;
    end

    %% Plotting Tangent and Closest Point 

    % Static Obstacles
    for i = 1:length(stc_obs)

        o = stc_obs{i};

        if strcmpi(o{1},'circle')
            cx = o{2}(1); cy = o{2}(2); r = o{2}(3);

            nvec = [xpos-cx; 
                    ypos-cy]; 

            dist = norm(nvec); 

        % Only draw if within detection radius
        if dist <= detect_r

            n_hat = nvec/dist;
            % Closest Point
            x_cl = cx + r*n_hat(1); 
            y_cl = cy + r*n_hat(2);
            
            % Perpendicular vector to unit vector n
            perp = [-n_hat(2); n_hat(1)]; 
            perp = perp/norm(perp);

            len = 2; 
            % Tangent Line
            tan_x = [x_cl-len*perp(1), x_cl+len*perp(1)]; 
            tan_y = [y_cl-len*perp(2), y_cl+len*perp(2)];

            set(h_close(obs_counter),'XData',x_cl,'YData',y_cl);
            set(h_tan(obs_counter),'XData',tan_x,'YData',tan_y);

        else
            set(h_close(obs_counter),'XData',nan,'YData',nan);
            set(h_tan(obs_counter),'XData',nan,'YData',nan);
        end



        elseif strcmpi(o{1},'rect')

            x = o{2}(1); y = o{2}(2);
            w = o{2}(3); h = o{2}(4);

            % Ellipsoid Parameters
            cx = x+w/2; cy = y+h/2; 
            a = w/2*sqrt(2); b = h/2*sqrt(2);

            nvec = [xpos-cx; 
                    ypos-cy]; 

            dist=norm(nvec); 

            % Only draw if within detection radius
            if dist <= detect_r

                n_hat = nvec/dist;


                % Closest Distance from a point to Ellipse, From the paper:

                % Dynamic Control Barrier Function-based Model Predictive Control 
                % to Safety-Critical Obstacle-Avoidance of Mobile Robot :
                % Page 3 , Equation 6

                theta = atan(n_hat(2) / n_hat(1));
                num = a^2 * b^2 * (1 + (tan(theta))^2);
                denom = b^2 + a^2 * ((tan(theta))^2);

                l = sqrt( num / denom);

                % Closest Point
                x_cl = cx + (l*n_hat(1)); 
                y_cl = cy + (l*n_hat(2));
    
                % Perpendicular vector to unit vector n
                perp = [-n_hat(2); n_hat(1)]; 
                perp = perp/norm(perp);

    
                len = 2; 
                % Tangent Line
                tan_x = [x_cl-len*perp(1), x_cl+len*perp(1)]; 
                tan_y = [y_cl-len*perp(2), y_cl+len*perp(2)];
    
                set(h_close(obs_counter),'XData',x_cl,'YData',y_cl);
                set(h_tan(obs_counter),'XData',tan_x,'YData',tan_y);
                
            else
                set(h_close(obs_counter),'XData',nan,'YData',nan);
                set(h_tan(obs_counter),'XData',nan,'YData',nan);
            end

        end

        obs_counter = obs_counter + 1;
    end

    % Dynamic Obstacles
    dyn_obs = update_dynamic_obstacles(dyn_obs, room, Ts);
    for i = 1:length(dyn_obs)

        cx = dyn_obs{i}{2}(1); cy = dyn_obs{i}{2}(2); r = dyn_obs{i}{2}(3);

        dyn_x = cx + r*cos(theta_dyn); 
        dyn_y = cy + r*sin(theta_dyn);
        set(dyn_obs_plot(i),'XData',dyn_x,'YData',dyn_y);

        nvec = [xpos-cx; 
                ypos-cy]; 
        dist = norm(nvec); 

     % Only draw tangent if within detection radius
    if dist <= detect_r
        n_hat = nvec/dist;

        % Closest Point
        x_cl = cx + r*n_hat(1); 
        y_cl = cy + r*n_hat(2);

        % Perpendicular vector to unit vector n
        perp = [-n_hat(2); n_hat(1)]; 
        perp = perp/norm(perp);

        len = 2; 
        % Tangent Line
        tan_x = [x_cl-len*perp(1), x_cl+len*perp(1)]; 
        tan_y = [y_cl-len*perp(2), y_cl+len*perp(2)];

        set(h_close(obs_counter),'XData',x_cl,'YData',y_cl);
        set(h_tan(obs_counter),'XData',tan_x,'YData',tan_y);
    else
        set(h_close(obs_counter),'XData',nan,'YData',nan);
        set(h_tan(obs_counter),'XData',nan,'YData',nan);
    end
    obs_counter = obs_counter + 1;


    end

%% Collision and Goal Check
hit = false;

% Static obstacles
for i = 1:length(stc_obs)
    o = stc_obs{i};
    
    if strcmp(o{1}, 'circle')
        cx = o{2}(1); cy = o{2}(2); r = o{2}(3);
        if norm([xpos-cx, ypos-cy]) < (robot_radius-0.001) + r
            hit = true; 
            break;
        end
        
    elseif strcmp(o{1}, 'rect')

        % Ellipsoid Constraints
        x = o{2}(1); y = o{2}(2);
        w = o{2}(3); h = o{2}(4);

        cx = x + w/2; cy = y + h/2;
        a = w/2 * sqrt(2);
        b = h/2 * sqrt(2);
        
        nvec = [xpos-cx; 
               ypos-cy];
        dist = norm(nvec);

        n_hat = nvec/dist;
        

        % Dynamic Control Barrier Function-based Model Predictive Control 
        % to Safety-Critical Obstacle-Avoidance of Mobile Robot :
        % Page 3 , Equation 6
        theta = atan(n_hat(2) / n_hat(1));
        num = a^2 * b^2 * (1 + (tan(theta))^2);
        denom = b^2 + a^2 * ((tan(theta))^2);

        l = sqrt( num / denom);
        
        % Closest Point
        x_cl = cx + (l*n_hat(1)); 
        y_cl = cy + (l*n_hat(2));

        % Distance check
        if norm([xpos-x_cl, ypos-y_cl]) < (robot_radius-0.001)
            hit = true;
            break;
        end
    end
end

% Dynamic obstacles
for i = 1:length(dyn_obs)
    o = dyn_obs{i};
    if strcmp(o{1}, 'circle')
        cx = o{2}(1); cy = o{2}(2); r = o{2}(3);
        if norm([xpos-cx, ypos-cy]) < (robot_radius-0.0001) + r
            hit = true; 
            break;
        end
    end
end

% Obstacle hit display
if hit
    text(mean(room(1:2)), mean(room(3:4)), 'Obstacle hit!', ...
        'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
    drawnow;
    break;
end

% Goal checking
goal = goals(current_goal_idx,:);
dist_to_goal = norm([xpos-goal(1), ypos-goal(2)]);

if dist_to_goal <= goal_tolerance
    text(goal(1), goal(2)+0.1, sprintf('Goal %d reached!', current_goal_idx), ...
        'Color','g','FontSize',10,'FontWeight','bold','HorizontalAlignment','center');
    
    current_goal_idx = current_goal_idx + 1;
    
    if current_goal_idx > size(goals,1)
        goal_reached = true;
        text(mean(room(1:2)), mean(room(3:4)), 'All goals reached!', ...
            'Color','g','FontSize',14,'FontWeight','bold','HorizontalAlignment','center');
        drawnow;
        break;
    end
end
legend([traj_line, robot_body, robot_circle_plot, COGvel, pred_line],...
       {'Trajectory','Robot Body','Safety Circle','Velocity Vector','Predicted Trajectory'},...
       'Location','bestoutside');

        % Save frame
        if saveVideo && mod(k, fastscale) == 0
            frame = getframe(gcf);
            writeVideo(writerObj, frame);
        end

    %% Draw everything
    if mod(k,fastscale)==0
        drawnow; pause(Ts);
    end
end



if current_goal_idx > size(goals,1)
    disp('All goals reached!');
elseif ~goal_reached
    disp('Simulation ended before all goals were reached.');
end

end
