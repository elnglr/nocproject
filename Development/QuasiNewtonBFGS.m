% QuasiNewton-BFGS Solver with optional Strong Wolfe line search or Armijo-only (Backtracking)

% INPUTS:
    % func: Given scalar Function
    % x0 : Given initial point
    % optim_settings: Optimization Settings such as 
        % optim_settings.xk_tol = Tolerance on the worst case between x_k+1 - xk --> inf_norm
        % optim_settings.grad_tol = Tolerance on the Gradient which is absolute value of gradient
        % optim_settings.method_grad = Method to provide Numerical Gradient of the (Cost) Function
        % optim_settings.method_linesearch = Method to provide Suitable alpha(step size) wih Global Convergence

% OUTPUT:
    % Results: It is struct that contains as below:
    % Results.x_values = After optimization and selection of alpha --> Update the x_k+1 = x_k + alpha * dk and store x_k+1
    % Results.f_values = After optimization and selection of alpha --> Update func(x_k+1) 
    % Results.iterations = Store the iterations

function Results = QuasiNewtonBFGS(func, x0, optim_settings)
    if nargin < 3 
        optim_settings.xk_tol = 1e-6;
        optim_settings.grad_tol = 1e-6;
        optim_settings.max_iter = 250;
        optim_settings.method_grad = 'ForwardFiniteDifferences';
        optim_settings.method_linesearch = 'Bisection';
    end

    % Initializations on optimization settings
    xk_tol = optim_settings.xk_tol;
    grad_tol = optim_settings.grad_tol;
    max_iter = optim_settings.max_iter;
    method_Grad = optim_settings.method_grad;
    method_LineSearch = optim_settings.method_linesearch;


    % Initializations on variables
    n = length(x0);
    xk = x0;      % Initial point x0
    Hk = eye(n);  % Initial Inverse of Hessian approximation

    % Initialize results
    Results.x_values = xk;
    Results.f_values = func(xk);
    Results.iterations = 0;

    for k = 1:max_iter

        gk = NumGrad(func, xk, method_Grad);   % Gradient at xk
        dk = -Hk * gk;                         % Search direction * Gradient

        % Strong Wolfe line search
        % Methods for choosing feasible alpha : bisec , backtrack
        alpha = LineSearchWolfe(func, xk, dk,method_LineSearch);

        % Update xk and its gradient
        xk_new = xk + alpha * dk; 
        gk_new = NumGrad(func, xk_new, method_Grad);

        % BFGS update for variations in 2 successive points and their gradients
        sk = xk_new - xk;
        yk = gk_new - gk;

        % BFGS inversion of Hessian according to Sherman-Morrison Identity
        % Broyden Fletcher Goldfarb and Shanno (BFGS) update formula for
        % Inversion of Hessian
        % Nonlinear Optimization Course Slide 7, Pages 8-9
        Hk = Hk + ((sk'*yk + yk'*Hk*yk)*(sk*sk')/(sk'*yk)^2) - (Hk*yk*sk' + sk*yk'*Hk)/(sk'*yk);

        % Update
        xk_prev = xk;
        xk = xk_new;

        % Store results
        Results.x_values = [Results.x_values, xk];
        Results.f_values = [Results.f_values, func(xk)];
        Results.iterations = k;

        % Termination conditions
        % 1) Gradient norm  or  2 Successive point difference in worst case
        if norm(gk_new,1) < grad_tol || norm(xk(1:2) - xk_prev(1:2), Inf) < xk_tol
            break;
        end


    end



end
