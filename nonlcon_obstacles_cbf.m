function [c,ceq] = nonlcon_obstacles_cbf(u_vec, z0, params, Ts, Np, obs, robot_radius, room)


ceq = [];
c = [];

xmin = room(1); xmax = room(2);
ymin = room(3); ymax = room(4);


stc_obs = obs.Static;
dyn_obs = obs.Dynamic;

n_stc = length(stc_obs);   % number of static obstacles
n_dyn = length(dyn_obs);   % number of dynamic obstacles
gamma1 = 0.20;              % CBF gain
gamma2 = 0.15;
detect_r = params.detect_radius; % LiDAR Detection Radius :In paper it was 12

z = z0;

for k = 1:Np


    u = u_vec(2*k-1:2*k);

    % x_idx = 2*k-1; % x input index (tau_r)
    % y_idx = 2*k;   % y input index (tau_l)

    % Integrate one step
    z_next = NumCalc(@fz_ddrive_accurate, (k-1)*Ts, z, u, Ts, params, "rungekutta4");
    robot_pos = z_next(1:2);
    dyn_obs = update_dynamic_obstacles(dyn_obs, room, Ts);



%% Circular Static obstacles (linearized constraints)
c_st = zeros(n_stc,1);

for i = 1:n_stc
    o = stc_obs{i};

    if strcmpi(o{1}, 'circle')
        cx = o{2}(1); cy = o{2}(2); r = o{2}(3);
        obs_center = [cx; cy];

        if norm(robot_pos - obs_center) <= detect_r % Take the only detected ones for computational reasons
            p0 = z(1:2);           
            delta0 = p0 - obs_center;
            d0 = norm(delta0);
    
            R = r + robot_radius;
    
            nvec = delta0/d0;
    
            % n' * (p1 - p0) >= R - d0
            rhs = R - d0;
            lhs = nvec' * (robot_pos - p0);
    
            % Must be writen in the following form for Matlab Functions: c <= 0
            c_st(i) = rhs - lhs;

        else
            c_st(i) = -1; % Feasible by default
        end

    %% Rectangular Static Obstacles (linearized constraints)
    elseif strcmpi(o{1}, 'rect')
    
        rect = o{2};
        x = rect(1); y = rect(2);
        w  = rect(3); h = rect(4);


        % Ellipse Parameters
        cx = x + w/2;
        cy = y + h/2;
        a = (w/2)*sqrt(2);     a_safe = a + robot_radius;
        b = (h/2)*sqrt(2);     b_safe = b + robot_radius;
    
        p0 = z(1:2);
        px0 = p0(1); py0 = p0(2);
    
        if norm(robot_pos - [cx, cy]) <= detect_r % Take the only detected ones for computational reasons
            % Ellipse Formula
            Ellipse0 = ((px0-cx)^2)/(a_safe^2) + ((py0-cy)^2)/(b_safe^2) - 1; % >=0
        
            % Gradient
            grad_Ellipse0 = [2*(px0-cx)/(a_safe^2); % dpx
                             2*(py0-cy)/(b_safe^2)];% dpy
        
        
            % Linearized constraint:
            c_ell = Ellipse0 + grad_Ellipse0' * (robot_pos - p0); % >=0
        
            % fmincon format: c <= 0
            c_st(i) = -c_ell;

        else
            c_st(i) = -1; % Feasible by default

        end
    end

end


%% Dynamic Obs (Nonlinear Constraints)
c_dyn = zeros(n_dyn,1);
    for j = 1:n_dyn
        o = dyn_obs{j};
        oc = o{2}(1:2);
        r = o{2}(3);

        v = (params.r/2)     * (z_next(4) + z_next(5)); % Linear velocity [m/s]
        
        % if norm(robot_pos - oc') <= detect_r % Take the only detected ones for computational reasons
        %     p0 = z(1:2);           
        %     delta0 = p0 - oc';
        %     d0 = norm(delta0);
        % 
        %     R = r + robot_radius;
        % 
        %     nvec = delta0/d0;
        % 
        %     % n' * (p1 - p0) >= R - d0
        %     rhs = R - d0;
        %     lhs = nvec' * (robot_pos - p0);
        % 
        %     % Must be writen in the following form for Matlab Functions: c <= 0
        %     c_dyn(j) = rhs - lhs;
        % 
        % else
        %     c_dyn(j) = -1; % Feasible by default
        % end
        % 


        % 
        if norm(robot_pos - oc') <= detect_r-4 % Take the only detected ones for computational reasons
            if norm(robot_pos - oc') -r-robot_radius<1 || norm(v)>4.5, gamma2 = 0.05;end
            c_dyn(j) = SOCBF(z, u, o, params, Ts, gamma1, gamma2, detect_r, room);
        else
            c_dyn(j) = -1; % Feasible by default
        end

        % R = r + robot_radius;
        % if norm(robot_pos - oc') <= detect_r % Take the only detected ones for computational reasons
        %     c_dyn(j) = R- norm(z_next(1:2)-oc',2);
        % else
        %     c_dyn(j) = -1; % Feasible by default
        % end


        
    end



    % ROOM constraints as a box
    c_room = zeros(4,1);
    x = robot_pos(1)-robot_radius; 
    y = robot_pos(2)-robot_radius;

    c_room(1) = xmin - x;
    c_room(2) = x - xmax;
    c_room(3) = ymin - y;
    c_room(4) = y - ymax;

    c_room = [
        xmin + robot_radius - robot_pos(1);
        robot_pos(1) - (xmax - robot_radius);
        ymin + robot_radius - robot_pos(2);
        robot_pos(2) - (ymax - robot_radius)
    ];

    % Put the constraints together
    if n_dyn ~= 0 ,c = [c; c_st;c_dyn; c_room];

    else, c = [c; c_st; c_room];end

    % Update state for next step
    z = z_next;

end


end




