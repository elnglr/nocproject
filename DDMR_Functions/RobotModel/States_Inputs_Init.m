% This function is used for printing initializations
% INPUTS:
%   model_name = "Simplified" | "Accurate"

function States_Inputs_Init(model_name,z0,u_prev)

switch lower(model_name)
    

    case 'simplified'

        fprintf('-------------------- States and Inputs Initializations --------------------\n');
        fprintf('Robot States z = [x_pos; y_pos; theta; omega_r; omega_l] \n')
        fprintf('Initial robot state z0: [%.2f; %.2f; %.2f; %.2f; %.2f]\n', z0);
        fprintf('Initial actuator inputs u0: [%.2f, %.2f]\n', u_prev(1),u_prev(2));
        fprintf('\n')


    case 'accurate'

        fprintf('-------------------- States and Inputs Initializations --------------------\n');
        fprintf('Robot States z = [x_pos; y_pos; theta; omega_r; omega_l] \n')
        fprintf('Initial robot state z0: [%.2f; %.2f; %.2f; %.2f; %.2f]\n', z0);
        fprintf('Initial actuator inputs u0: [%.2f, %.2f]\n', u_prev(1),u_prev(2));
        fprintf('\n')

end

end

