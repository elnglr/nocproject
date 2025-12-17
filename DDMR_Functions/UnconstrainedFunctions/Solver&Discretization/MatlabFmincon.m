function Results = MatlabFmincon(func, x0, optim_settings)
% MatlabFmincon  - fmincon that returns the same Results struct
% as QuasiNewtonBFGS (Results.x_values, Results.f_values, Results.iterations).
%
% Usage:
%   Results = MatlabFmincon(func, x0, optim_settings)
%
% INPUTS:
%   func            - Given scalar Function
%   x0              - Given initial point
%   optim_settings  - struct with optional fields:
%       optim_settings.xk_tol      = Tolerance on the worst case between x_k+1 - xk --> inf_norm
%       optim_settings.grad_tol    = Tolerance on the Gradient which is absolute value of gradient
%       optim_settings.method_grad = Method to provide Numerical Gradient of the (Cost) Function
%       optim_settings.Display     = Method to display the solver iteration
%       optim_settings.Algorithm   = Method to choose algorithm of the solver
%
% OUTPUT:
%   Results: It is struct that contains as below:
%       Results.x_values   = After optimization --> Update the x_k+1 
%       Results.f_values   = After optimization --> Update func(x_k+1) 
%       Results.iterations = Store the iterations


    % -------------------- Default Settings --------------------
    if nargin < 3 || isempty(optim_settings)
        optim_settings.xk_tol = 1e-6;
        optim_settings.grad_tol = 1e-6;
        optim_settings.max_iter = 250;
        optim_settings.Display = 'off';
        optim_settings.Algorithm = 'sqp';
    end

    Np = 10; % Prediction Horizon
    lb = -1.85 * ones(2 * Np, 1); % Lower Control Input bound
    ub =  1.85 * ones(2 * Np, 1); % Upper Control Input bound

    % Initialize Storage
    x_history = x0(:);                
    f_history = func(x0(:));

    

    % fmincon Options
    options = optimoptions('fmincon', ...
        'Display', optim_settings.Display, ...
        'Algorithm', optim_settings.Algorithm, ...
        'MaxIterations', optim_settings.max_iter, ...
        'OptimalityTolerance', optim_settings.grad_tol, ...
        'StepTolerance', optim_settings.xk_tol ...
        );
    A = []; b = []; Aeq = []; beq = [];

    % Run fmincon
    [x_opt, fval, exitflag, output] = fmincon(func, x0, A, b, Aeq, beq, lb, ub, [], options);

    % Store the data
    x_history = [x_history, x_opt(:)];
    f_history = [f_history, fval];

    % Embed to the output structure
    Results.x_values   = x_history;
    Results.f_values   = f_history;

    % Take the number of iterations
    if isfield(output, 'iterations')
        Results.iterations = output.iterations;
    else
        Results.iterations = size(x_history, 2) - 1;
    end

end
