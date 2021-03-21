function [p,q,r] = rollrate2pqr(phi,theta,psi,phi_dot,theta_dot,psi_dot)

% Function to calculate the roll, pitch, and yaw from a given position with
% a known roll rate, pitch rate, and yaw rate
%
% Inputs: 
%         phi       -> yaw
%         theta     -> pitch
%         psi       -> yaw
%         phi_dot   -> roll rate
%         theta_dot -> pitch rate
%         psi_dot   -> yaw rate
% Outputs:
%         p         -> roll velocity
%         q         -> pitch velocity
%         r         -> yaw velocity


    %% Sovle for pqr vector
    pqr = [phi_dot,theta_dot,psi_dot]...
        *inv([1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
        0, cos(phi), sin(phi);...
        0, sin(phi)*sec(theta), cos(phi)*sec(theta)]);

    %% Define p, q, and r based on previous calculations
    p = pqr(1);
    q = pqr(2);
    r = pqr(3);
    
end