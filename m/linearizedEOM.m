function state_dot = linearizedEOM(t, state_vec, forces, perturbations, knowns)

% Function to calculate the respose of a linearized quadrotor model with
% feedback loops intergrated
%
% Inputs: 
%         t             -> time
%         state_vec     -> state vector
%         forces        -> motor forces
%         perturbations -> vector to describe any perturbations
%         knowns        -> vector of known values
% Outputs: 
%         state_dot     -> derivative of the state vector

    %% Define known variables
    mu = knowns(1); % [N/(m/s)^2]
    I_x = knowns(3); % x moment of inertia [kg*m^2]
    I_y = knowns(4); % y moment of inertia [kg*m^2]
    I_z = knowns(5); % z moment of inertia [kg*m^2]
    g = knowns(7); % gravity [m/s^2]
    k_1 = knowns(8); % lateral k_1
    k_2 = knowns(9); % lateral k_2
    k_3 = knowns(10); % lateral k_3
    k_4 = knowns(11); % longitudinal k_1
    k_5 = knowns(12); % longitudinal k_2
    k_6 = knowns(13); % longitudinal k_3
    R = knowns(14); % radius from CG to motor force [m]
    k_m = knowns(15); % control moment coefficient
    
    %% Define variables
    x = state_vec(1); % x position
    y = state_vec(2); % y position
    z = state_vec(3); % z position
    phi = state_vec(4); % roll angle
    theta = state_vec(5); % pitch angle
    u = state_vec(7); % x velocity
    v = state_vec(8); % y velocity
    w = state_vec(9); % z velocity
    p = state_vec(10); % roll rate
    q = state_vec(11); % pitch rate
    r = state_vec(12); % yaw rate
    f_1 = forces(1); % motor force 1
    f_2 = forces(2); % motor force 2
    f_3 = forces(3); % motor force 3
    f_4 = forces(4); % motor force 4
    
    %% Define Aerodynamic/Control Forces/Moments
    M_aero = -mu*norm([p;q;r])*[p;q;r]; % aerodynamic moment
    M_cntl = [(R/sqrt(2))*(-f_1-f_2+f_3+f_4);...
        (R/sqrt(2))*(f_1-f_2-f_3+f_4);...
        k_m*(f_1-f_2+f_3-f_4)]; % control moment
    
    %% Linear Equations of Motion
    % rate of change of euler angles
    euler_dot = [p; q; r];
    % acceleration
    v_dot = g*[-theta; phi; 0];
    
    if knowns(16) == 1 && (t >= 0 && t <= 2)
        % time and distance requiered for required travel
        t_r = knowns(19);
        x_r = knowns(17);
        y_r = knowns(18);
        % velocity to meet requirements
        u_r = (x_r-x)/(t_r-t);
        v_r = (y_r-y)/(t_r-t);
        % angular acceleration
        v_angular_dot = [-((k_1*p)/I_x) - ((k_2*phi)/I_x) + (k_3*(v_r-v))/I_x;...
            -((k_4*q)/I_y) - ((k_5*theta)/I_y) + (k_6*(u_r-u))/I_y;...
            (M_aero(3) + M_cntl(3) + perturbations(3))/I_z];
        % velocity
        xyz_dot = [u_r-u; v_r-v; w];
    else
        % angular acceleration
        v_angular_dot = [-((k_1*p)/I_x) - ((k_2*phi)/I_x) + (k_3*v)/I_x;...
            -((k_4*q)/I_y) - ((k_5*theta)/I_y) + (k_6*u)/I_y;...
            (M_aero(3) + M_cntl(3) + perturbations(3))/I_z];
        % velocity
        xyz_dot = [u; v; w];
    end
    
    state_dot = [xyz_dot; euler_dot; v_dot; v_angular_dot]; % vector output for the integrals

end