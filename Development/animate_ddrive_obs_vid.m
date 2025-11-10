%======================================================================
% Differential-drive robot animation with obstacles and multiple goal points.
% Now with Full HD video recording support (MPEG-4, 60fps, high quality).
%======================================================================

function animate_ddrive_obs_vid(traj, params, Ts, obs, goals, room, fastscale, goal_radius, robot_shape, saveVideo)

    % -------------------- DEFAULTS --------------------
    if nargin < 10
        saveVideo = true; % default: video kaydetme
    end

    % -------------------- VIDEO SETUP --------------------
    if saveVideo
        vname = 'ddrive_animation.mp4'; % video dosya ismi
        writerObj = VideoWriter(vname, 'MPEG-4');
        writerObj.FrameRate = max(1, round(1 / (Ts * fastscale))); % FPS
        writerObj.Quality = 100; % Maksimum kalite
        open(writerObj);
        disp(['🎥 Video kaydı başlatıldı: ', vname]);
    end

    % -------------------- FIGURE SETUP --------------------
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

    % Goals
    for i = 1:size(goals,1)
        plot(goals(i,1), goals(i,2), 'go', 'MarkerSize', 10, ...
             'MarkerFaceColor', 'g', 'DisplayName', sprintf('Goal %d', i));
    end

    % Obstacles
    for i = 1:length(obs)
        o = obs{i};
        if strcmpi(o{1}, 'circle')
            cx = o{2}(1); cy = o{2}(2); r = o{2}(3);
            viscircles([cx,cy], r, 'Color', 'k');
        elseif strcmpi(o{1}, 'rect')
            x = o{2}(1); y = o{2}(2); w = o{2}(3); h = o{2}(4);
            rectangle('Position',[x, y, w, h], 'FaceColor',[0.5 0.5 0.5], 'EdgeColor','k');
        end
    end

    % Time Display
    time_text = text(room(1)+0.3, room(4)-0.3, 'Time: 0.00 s', ...
                     'FontSize', 12, 'FontWeight', 'bold', ...
                     'Color', [0.1 0.1 0.5], 'BackgroundColor', [1 1 1 0.7], ...
                     'Margin', 4);

    % Robot Params
    Ltot = 2 * params.L;
    W = params.r * 4;
    robot_radius = sqrt( 2*(Ltot/2)^2 );
    goal_tolerance = goal_radius;

    % Safety Circle
    theta_circle = linspace(0,2*pi,50);
    robot_circle_x = robot_radius*cos(theta_circle);
    robot_circle_y = robot_radius*sin(theta_circle);

    % Dynamic Elements
    traj_line = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
    COGvel = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Velocity Vector');
    robot_body = plot(NaN, NaN, 'k-', 'LineWidth', 2,'DisplayName','Robot Body');
    robot_circle_plot = plot(NaN, NaN, 'b--','LineWidth',1.5,'DisplayName','Safety Circle');
    legend('show','Location','bestoutside');

    current_goal_idx = 1;
    goal_reached = false;

    % -------------------- ANIMATION LOOP --------------------
    for k = 1:size(traj,2)

        xpos = traj(1,k);
        ypos = traj(2,k);
        theta = traj(3,k);

        % Linear velocity and components
        v = (params.r/2) * (traj(4,k) + traj(5,k));
        vx = v * cos(theta);
        vy = v * sin(theta);

        % Update Trajectory
        set(traj_line, 'XData', traj(1,1:k), 'YData', traj(2,1:k));

        % Robot shape
        if robot_shape == "Vertical"
            corners = [-Ltot/2, -W/2;
                        Ltot/2, -W/2;
                        Ltot/2,  W/2;
                       -Ltot/2,  W/2]';
        else
            corners = [-W/2, -Ltot/2;
                        W/2, -Ltot/2;
                        W/2,  Ltot/2;
                       -W/2,  Ltot/2]';
        end

        R = [cos(theta) -sin(theta);
             sin(theta)  cos(theta)];
        rotated = R * corners;

        bodyX = rotated(1,:) + xpos;
        bodyY = rotated(2,:) + ypos;

        set(robot_body, 'XData', [bodyX bodyX(1)], 'YData', [bodyY bodyY(1)]);
        set(robot_circle_plot, 'XData', xpos + robot_circle_x, ...
                               'YData', ypos + robot_circle_y);
        scale = 0.3;
        set(COGvel, 'XData', xpos, 'YData', ypos, 'UData', vx*scale, 'VData', vy*scale);

        set(time_text, 'String', sprintf('Time: %.2f s', k*Ts));

        % Collision Check
        hit = false;
        for i = 1:length(obs)
            o = obs{i};
            if strcmp(o{1}, 'circle')
                cx=o{2}(1); cy=o{2}(2); r=o{2}(3);
                if norm([xpos-cx, ypos-cy]) <= robot_radius + r
                    hit = true; break;
                end
            end
        end

        if hit
            text(mean(room(1:2)), mean(room(3:4)), ...
                 'Obstacle hit!', 'Color', 'r', 'FontSize', 14, ...
                 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            drawnow; break;
        end

        % Goal Check
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

        % Save frame
        if saveVideo && mod(k, fastscale) == 0
            frame = getframe(gcf);
            writeVideo(writerObj, frame);
        end

        % Animation update
        if mod(k, fastscale) == 0
            drawnow;
            pause(Ts);
        end
    end

    % -------------------- FINISH --------------------
    if ~goal_reached
        disp('Simulation ended before all goals were reached.');
    end

    if saveVideo
        close(writerObj);
        disp('✅ Video başarıyla kaydedildi (Full HD).');
    end
end
