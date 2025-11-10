% This function is going to be used for taking gradient of the any function

% INPUTS:
    % J: [m x 1] Given Vector/Scalar(m=1) Cost function whose gradient will be taken
    % x: [n x 1] Decision Variables in Cost Function 
    % method_Grad : Method to provide Numerical Gradient of the (Cost) Function

% OUTPUT:
    % J_grad: [n x m] Gradient of the Vector/Scalar(m=1) Cost Function

% Reminder : 
% Jacobian(J(x)) : [m x n], 
% Gradient(J(x))  : [n x m],
% Gradient(J(x)) = Jacobian(J(x))'

function J_grad = NumGrad(J, x, method_Grad)



    if nargin < 3 || isempty(method_Grad),method_Grad = 'forwardfinitedifferences'; end

    n = length(x);    % Number of States
    m = length(J(x)); % Number of Functions
    J_grad = zeros(n,m);
    Jx = J(x);

    switch lower(method_Grad)

        %% -------------------- Forward Finite Differences Method --------------------
        case 'forwardfinitedifferences'
            tau = 1e-6;
            for i = 1:n
                p = zeros(n,1);
                p(i,1)=1;
                J_grad(i,:) = (J(x + tau*p) - Jx)'/tau;
            end

        %% -------------------- Central Finite Differences Method --------------------
        case 'centralfinitedifferences'
            tau = 1e-8;
            for i = 1:n
                p = zeros(n,1);
                p(i,1)=1;
                J_grad(i,:) = (J(x + tau*p) - J(x - tau*p))' / (2*tau);
            end

         %% -------------------- Imaginary Part Trick Method --------------------
        case 'imaginarytrick' % It does not work .......WHY!!!! :(
            error(' SOMEHOW IT  IS NOT WORKING :(')
            tau = 1e-6;
            for i = 1:n
                p = zeros(n,1);
                p(i,1)=1;
                J_grad(i,:) = imag(J(x + 1j*tau*p)) / tau;
            end

    end

end
