% Barrier Cost Functions to avoid the Obstacles

%% Candidates: Application aim is to convert Constrained problem into Unconstrained one
%% 1) Const * -ln( a* c(x)>=0 )
% -ln(c(x)>=0) which is smooth and
% continuously differentiable function behaves like a barrier if c(x)~= 0,
% then -ln(c(x)>=0) will go to infinity to act like a barrier to push our
% solution to the feasible region before hitting the obstacles. But there
% is an issue about its rate. It has slow trigger wrt. exp(-c(x)).

% Don't forget that -ln(c(x)>=0) is only defined when c(x)>0, if there is
% such c(x)<=0, We'll apply huge penalisation such as 1e6 to the J_obs but
% I think it is not logical right now (31/10/2025), I will discuss why.

%% 2) Const * exp( -a* c(x)>=0 )
% exp(*c(x)>=0) which is smooth and continuosly differentiable function
% it behaves like a pseudo barrier function, bcs it does not diverge to the
% infinity the way we want, but locally it is better than suitable one -ln(.) 
% Moreover its% rate is really fast wrt. -ln(c(x)>=0), and in our application robot must act
% dodge the obstacles as much as fast it is.
%  tune2 * exp(-tune3 * d/(tune1)) will give us huge penaliation if d goes to zero
%  penalisation. 


% INPUTS:
    % z: States of the Dynamical System
    % obs: This structure contains the information of obstacles:
    % Example: obs={
    %               {'circle',[3 3 0.6]}, 
    %               {'circle',[6 2 0.8]}    }
    % o = obs{i}; 'circle' = o{1}; center_x=o{2}(1); center_y=o{2}(2); radius=o{2}(3);

% Important Variables in the code
    % d: Safety Distance establihed by follows :
    % d = norm(z(1:2)-o{2}(1:2)',2) - o{2}(3) - robot_radius
    % norm(z(1:2)-o{2}(1:2)',2) : Distance between the COG of Robot and Center of Obstacles
    % o{2}(3) - robot_radius : This is the safety region that is defined as
    % Difference between Radius of obstacles and Given robot_radius which is
    % approximately sqrt(2*L^2);
    % and Tune parameters such as tune1,2,3 to tune barrier functions

% OUTPUTS:
    % J_obs = The Scalar Cost Function to consider obstacle avoidance

function J_obs = Cost_obsbarrier(z, obs, robot_radius)
    J_obs = 0;

    for i = 1:length(obs)
        o = obs{i}; % Take the instances of Obstacles

        if strcmpi(o{1},'circle')

            d = norm(z(1:2)-o{2}(1:2)') - o{2}(3) - robot_radius;

            % if d <= 0 % d<=0 is not defined in -ln(d)
            %     % If the Safety Region is violated --> Penalise the cost
            %     % function quickly and strongly. suc
            %     J_obs = J_obs + 1e6;
            % else
            %     J_obs = J_obs - 0.01*log(d); "It is not logical"
            % end
            % 

            % For tuning :)
            tune1 = 0.2;
            tune2=  1e3;
            tune3 = 10;

            d_norm = d/tune1;
          

            % % Huge Penalisation if d<=0
            % if d<=0 , d_norm = -1.81 ;% It will give us 
            % end  % -1.8 "It is not logical"
            % exponential barrier
            J_obs = J_obs + tune2 * exp(-tune3 * d_norm);

    end
    end

end