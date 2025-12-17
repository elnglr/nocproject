% This function decides which solver to solve optimization problem to get the Optimal Control Inputs : [tau_r* ; tau_l*]

% INPUTS:
    % func: Given Cost Function to be minimized to get optimal control inputs
    % x0 : Given initial point to solve the optimization problem
    % optim_settings: Optimization Settings such as 
        % optim_settings.xk_tol = Tolerance on the worst case between x_k+1 - xk --> inf_norm
        % optim_settings.grad_tol = Tolerance on the Gradient which is absolute value of gradient
        % optim_settings.method_grad = Method to provide Numerical Gradient of the (Cost) Function
        % optim_settings.method_linesearch = Method to provide Suitable alpha(step size) wih Global Convergence
        % optim_settings.method_Solver = Solver to give us a optimal solution, e.g. "QuasiNewton-BFGS"

% OUTPUT:
    % Opt: Literally Same as the output of any solver. (e.g. Results = QuasiNewtonBFGS(.) )

function [Opt] = OptSolver(func, x0,optim_settings)

    method_solver = optim_settings.Solver;
    
    if strcmpi(method_solver,"QuasiNewton-BFGS")
        Opt = QuasiNewtonBFGS(func, x0, optim_settings);
    
    elseif strcmpi(method_solver, "fmincon")
        Opt = MatlabFmincon(func, x0, optim_settings);
    
    
        %% NEW TASK: PERFOM  GAUSSNEWTON METHOD
           
        % elseif strcmpi(method_solver, "GaussNewton")
        %     Opt = GaussNewton(func,x0,optim_settings);
        % 


    end

end

