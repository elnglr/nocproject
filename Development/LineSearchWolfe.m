% Strong Wolfe Line Search (with optional 'Bisection', 'Backtracking' update)
% Based on Nonlinear Optimization Lecture Slides 4

%   Strong Wolfe conditions:
%   Define phi(alpha) = func(xk + alpha*dk)
%
%   1) Armijo (Sufficient Decrease)
%     phi_alpha <= phi0 + c1*alpha*dphi0 , where c1 is [0,1] , For BFGS:1e-4
%
%   2) Curvature condition:
%     abs(dphi_alpha) <= c2*abs(dphi0) , where c2 is (c1,1) , For BFGS: 0.9
%
%
% If method = 'Bisection'  ->  Strong Wolfe Conditions + Bisection method used to find alpha
% If method = 'Backtracking' -> Strong Wolfe Conditions + alpha = 0.5*alpha


% INPUTS:
    % func: Function (scalar)
    % xk: Current Point
    % dk: Search Direction
    % method_LineSearch: Methods to provide suitable step size alpha: 'Bisection' | 'Backtracking'
    % method_Grad: Methods to provide gradient of a function: Forward Finite Differences , Central Finite Differences
    % c1, c2: Wolfe constants Armijo Constant, Curvature Constant respectively
    
% OUTPUT:
    % alpha: Step Size satisfying Strong Wolfe


function [alpha] = LineSearchWolfe(func, xk, dk, method_LineSearch, method_Grad, c1, c2)
    if nargin < 4 || isempty(method_LineSearch), method_LineSearch = 'Bisection'; end
    if nargin < 5 || isempty(method_Grad), method_Grad = 'finiteforwardifferences'; end
    if nargin < 6 || isempty(c1), c1 = 1e-4; end
    if nargin < 7 || isempty(c2), c2 = 0.9; end


    phi0 = func(xk);                       % % Obtain phi(alpha=0) = f(xk + alpha=0*dk)
    g0 = NumGrad(func, xk, method_Grad);   % % Obtain grad_f(xk + alpha=0*dk)
    dphi0 = g0' * dk;                      % % Obtain phi_(alpha=0)                 
   
    % Initial step sizes an iyeration
    alpha = 1;        % Initial step size
    phi_prev = phi0;  % f(xk + alpha=0 * dk)
    iter = 0;

    % fprintf('\n--- Line Search Iterations (Strong Wolfe) ---\n');
    % fprintf('Iter | Alpha      | Armijo | Curvature\n');
    % fprintf('----------------------------------------\n');

    while true

        iter = iter + 1;
        
        % Obtain f(xk + alpha * dk ) and the gradient of this poiny
      
        phi_alpha = func(xk + alpha * dk);% Obtain f(xk + alpha*dk)
        g_alpha = NumGrad(func, xk + alpha * dk, method_Grad);  % Obtain grad_f(xk + alpha*dk)
        dphi_alpha = g_alpha' * dk; % Obtain grad_phi(alpha)

        % Check Armijo and Curvature conditions --> Strong Wolfe Conditions
        armijo = (phi_alpha <= phi0 + c1 * alpha * dphi0);
        curvature = abs(dphi_alpha) <= c2 * abs(dphi0);

        % fprintf('%4d | %.6f | %4d | %9d\n', iter, alpha, armijo, curvature);

        % If both conditions satisfied -> Strong Wolfe satisfied
        if armijo && curvature
            % fprintf('Strong Wolfe satisfied at iter %d | alpha = %.6f\n', iter, alpha);
            break; % Strong Wolfe Conditions Satisfied 
        end

        % Armijo Condition Violation or Function Increase = They have both
        % same meaning 
        if ~armijo || (phi_alpha >= phi_prev)
            if strcmpi(method_LineSearch, 'Bisection')
                % Bisection method to get suitable size of alpha
                alpha = Bisec(func, xk, dk, alpha, phi0, dphi0, c1, c2, method_Grad);
                break;
            else
                % Backtracking -> Each iteration shrink
                % the alpha
                alpha = alpha * 0.5;
            end

        % Curvature Condition Violation
        elseif ~curvature
            if strcmpi(method_LineSearch, 'Bisection')
                 % Bisection method to get suitable size of alpha
                alpha = Bisec(func, xk, dk, alpha, phi0, dphi0, c1, c2, method_Grad);
                break;
            else
                % Backtracking  -> Each iteration shrink the alpha                
                alpha = alpha * 0.5;
            end
        end

        % Update previous phi for next iteration
        phi_prev = phi_alpha;
    end
end


% Bisection / Wolfe Line Search
% Finds a step length alpha satisfying the Strong Wolfe conditions
% Based on Nonlinear Optimization Course Slide 4, Pages 15-16-17

function alpha = Bisec(func, xk, dk, alpha_prev, phi0, dphi0, c1, c2, method_Grad)
   
    % Initialize step length interval
    alpha_min = 0;
    alpha_max = 0;
    alpha = alpha_prev;
    iter = 0;

    % fprintf('--- Bisection Iterations ---\n');
    % fprintf('Iter | Alpha      | Armijo | Curvature\n');
    % fprintf('----------------------------------------\n');

    while true
        iter = iter + 1;

        phi_alpha = func(xk + alpha * dk); % Obtain phi(alpha) = f(xk + alpha*dk)
        g_alpha = NumGrad(func, xk + alpha * dk, method_Grad); % Obtain grad_f(xk + alpha*dk)
        dphi_alpha = g_alpha' * dk;  % Obtain grad_phi(alpha)

        % Check Armijo and Curvature Conditions --> Strong Wolfe Conditions
        armijo = (phi_alpha <= phi0 + c1 * alpha * dphi0);
        curvature = abs(dphi_alpha) <= c2 * abs(dphi0);
        % fprintf('%4d | %.6f | %6d | %9d\n', iter, alpha, armijo, curvature);

        % If both conditions satisfied -> Strong Wolfe satisfied
        if armijo && curvature
            % fprintf('Bisec found Strong Wolfe at iter %d | alpha = %.6f\n', iter, alpha);
            break;
        end

        if ~armijo

            % Armijo not satisfied -> shrink the interval
            alpha_max = alpha;
            alpha = 0.5 * (alpha_min + alpha_max);

        elseif ~curvature

            % Curvature not satisfied -> enlarge the interval
            alpha_min = alpha;
            if alpha_max == 0
                alpha = 2 * alpha_min;
            else
                alpha = 0.5 * (alpha_min + alpha_max);
            end


        end
    end
end
