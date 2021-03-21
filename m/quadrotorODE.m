function state_dot = quadrotorODE(t,state_vec,forces,knowns)

% Function to calculate the respose of a non-linearized quadrotor model 
% with feedback loops intergrated
%
% Inputs: 
%         t             -> time
%         state_vec     -> state vector
%         forces        -> motor forces
%         knowns        -> vector of known values
% Outputs: 
%         state_dot     -> derivative of the state vector

    %% Define known variables
    mu = knowns(1); % [N/(m/s)^2]
    nu = knowns(2); % [N/(rad/s)^2]
    I_x = knowns(3); % x moment of inertia [kg*m^2]
    I_y = knowns(4); % y moment of inertia [kg*m^2]
    I_z = knowns(5); % z moment of inertia [kg*m^2]
    m = knowns(6); % mass [kg]
    g = knowns(7); % gravity [m/s^2]
    k_1 = knowns(8); % lateral k_1
    k_2 = knowns(9); % lateral k_2
    k_3 = knowns(10); % lateral k_3
    k_4 = knowns(11); % longitudinal k_1
    k_5 = knowns(12); % longitudinal k_2
    k_6 = knowns(13); % longitudinal k_3
    k_m = knowns(15); % moment coefficient
    
    %% Define variables
    phi = state_vec(4); % roll angle
    theta = state_vec(5); % pitch angle
    psi = state_vec(6); % yaw angle
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
    F_aero = -nu*norm([u;v;w])*[u;v;w]; % aerodynamic force 
    M_cntl = [-k_1*p-k_2*phi;...
        -k_4*q-k_5*theta;...
        k_m*(f_1-f_2+f_3-f_4)]; % control moment
    F_cntl = [0; 0; -sum(forces)]; % control force
    
    %% Non-Linear Equations of Motion
    % velocity
    xyz_dot = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]...
        * [u;v;w];
    % rate of change of the euler angles
    euler_dot = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
        0 cos(phi) -sin(phi);...
        0 sin(phi)*(1/cos(theta)) cos(phi)*(1/cos(theta))]...
        * [p;q;r];
    % acceleration
    v_dot = [r*v-q*w; p*w-r*u; q*u-p*v]...
        + g*[-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)]...
        + (1/m)*F_aero + (1/m)*F_cntl;
    % angular acceleration
    v_angular_dot = [((I_y-I_z)/I_x)*q*r; ((I_z-I_x)/I_y)*q*r; ((I_x-I_y)/I_z)*q*r]...
        + [(1/I_x)*M_aero(1); (1/I_y)*M_aero(2); (1/I_z)*M_aero(3)]...
        + [(1/I_x)*M_cntl(1); (1/I_y)*M_cntl(2); (1/I_z)*M_cntl(3)];
    
    state_dot = [xyz_dot; euler_dot; v_dot; v_angular_dot]; % vector output for the integrals

end