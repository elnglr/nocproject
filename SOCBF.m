function c_socbf = SOCBF(xk, uk, o, params, Ts, gamma1, gamma2, detect_r, room)
    % o format: { 'circle', [cx,cy,r], [vx,vy] }
    c_socbf = -1.0;
    robot_radius = params.robot_radius;

    if ~strcmpi(o{1}, 'circle'), return; end

    cx = o{2}(1); cy = o{2}(2); ro = o{2}(3);
    if length(o) >= 3
        vobs = o{3}(:); % [vx; vy]
    else
        vobs = [0;0];
    end

    obs_center = [cx; cy];
    R = ro + robot_radius;

    % robot position
    p_k = xk(1:2);
    dist_k = norm(p_k - obs_center);

    if nargin>=9 && ~isempty(detect_r) && dist_k > detect_r
        c_socbf = -1.0; return;
    end

    % predict robot positions (using current control uk)
    x1 = NumCalc(@fz_ddrive_accurate, 0, xk, uk, Ts, params, "rungekutta4");
    x2 = NumCalc(@fz_ddrive_accurate, 0, x1, uk, Ts, params, "rungekutta4");
    p1 = x1(1:2); p2 = x2(1:2);

    % predict obstacle positions assuming constant velocity
    obs_k   = obs_center;
    obs_k1  = obs_center + vobs * Ts;
    obs_k2  = obs_center + vobs * (2*Ts);

    % compute h for moving obstacle (relative)
    h_k   = norm(p_k  - obs_k)  - R;
    h_k1  = norm(p1   - obs_k1) - R;
    h_k2  = norm(p2   - obs_k2) - R;

    % second-order error as before
    he_k   = (h_k1 - h_k)  + gamma1 * h_k;
    he_k1  = (h_k2 - h_k1) + gamma1 * h_k1;

    % CBF inequality (fmincon form c<=0)
    c_socbf = (1 - gamma2) * he_k - he_k1;
end
