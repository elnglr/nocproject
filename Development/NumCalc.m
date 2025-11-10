% This function is going to be used for discretization of the Dynamics

% INPUTS:
    % fz: z_dot = fz(z,u,Ts,t,params) is the function describes whole dynamics of the robot
    % t: time
    % z: States of the system to be described
    % h: Step Size or Sampling Time
    % params: Parameters which contains Robot parameters
    % method: There are methods to solve the ode / discretize the system: "forwardEuler", "rungekutta2", "rungekutta4"

% OUTPUT:
    % z_next: Next step of the states (State evolution in Discrete Time x_k --> x_k+1)
    % u_next: Next step of the inputs (Inputs evolution in Discrete Time u_k --> u_k+1)

function [z_next,u_next] = NumCalc(fz, t, z, u, h, params, method)
    

    switch lower(method)

        %% -------------------- Forward Euler Method --------------------
        case "forwardeuler"
            [zdot,u_next] = fz(t,z,u,params);
            z_next = z + h*zdot;

        %% -------------------- Runge-Kutta2 Method --------------------
        case "rungekutta2"
            [k1,u1] = fz(t, z, u, params);
            [k2,u2] = fz(t+h/2, z + h/2*k1, u1, params);
            z_next = z + k2*h;
            u_next = u2;
        %% -------------------- Runge-Kutta4 Method --------------------
        case "rungekutta4"
            [k1,u1] = fz(t, z, u, params);
            [k2,u2] = fz(t+h/2, z + h/2*k1, u1, params);
            [k3,u3] = fz(t+h/2, z + h/2*k2, u2, params);
            [k4,u4] = fz(t+h, z + h*k3, u3, params);
            z_next = z + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
            u_next = u4;
    end



end
