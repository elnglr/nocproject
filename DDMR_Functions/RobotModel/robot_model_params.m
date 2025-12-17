% This function is  used for returning parameters and model handle for different configurations

function [robot_model, params] = robot_model_params(model_name)
% INPUTS:
%   model_name = "Simplified" | "Accurate"

% OUTPUTS:
%   robot_model  : Function handle to the dynamics model
%   params       : Structure Containing model parameters

fprintf('-------------------- Model Selection --------------------\n');
fprintf('Selected model: "%s"\n', model_name);

switch lower(model_name)

    %% -------------------- Simplified Model --------------------
    case 'simplified'
        params.r = 0.05;       % wheel radius [m]
        params.L = 0.20;       % half wheelbase [m]
        params.Jw = 0.0025;    % wheel inertia [kg*m^2]
        params.Bw = 0.02;   % wheel viscous friction
        params.n = 5;          % Number of States
        params.m = 2;          % Number of Input
        params.z0 = [1;1;0;0;0]; % Initial Condition
        
        robot_model = @fz_ddrive; % Chosen Robot Model

        fprintf('Simplified Model Parameters\n');
        fprintf('r     = %.4f [m]\n', params.r);
        fprintf('L     = %.4f [m]\n', params.L);
        fprintf('Jw    = %.4f [kg*m^2]\n', params.Jw);
        fprintf('viscw = %.4f [N*m*s/rad]\n', params.Bw);
        fprintf('viscw = %.4f [N*m*s/rad]\n', params.Bw);


    %% -------------------- Accurate Mechanical Model --------------------
    case 'accurate'
        params.r = 0.05;       % wheel radius [m]
        params.L = 0.20;       % half wheelbase [m]
        params.Jw = 0.0025;    % wheel inertia [kg*m^2]
        params.M  = 5.0;       % total mass [kg]
        params.Jb = 0.005;     % body inertia [kg*m^2]
        params.d  = 0.00;      % COG offset [m]
        params.Bw = 0.02;      % wheel viscous friction torque [N*m*s/rad]
        params.n = 5;          % Number of States
        params.m = 2;          % Number of Inputs
        params.z0 = [1;1;0;0;0]; % Initial Conditions

        robot_model = @fz_ddrive_accurate; % Chosen Robot Model

        fprintf('Accurate Mechanical Model Parameters\n');
        fprintf('r  = %.4f [m], L = %.3f [m]\n', params.r, params.L);
        fprintf('Jw = %.4f [kg*m^2], Bw = %.4f [N*m*s/rad]\n', params.Jw, params.Bw);
        fprintf('M  = %.4f [kg], Jb = %.4f [kg*m^2]\n', params.M, params.Jb);
        fprintf('d  = %.4f [m]\n', params.d);

    otherwise
        error('Unknown model_name: %s', model_name);
end

end
