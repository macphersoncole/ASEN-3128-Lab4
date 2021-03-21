%% ASEN 3128 - LAb 4 - Main
% Script to compare the lineared and non-linearized control responses of a
% quadrotor with a feedback loop incorperated into the equations of motion
%
%
% Author: Cole MacPherson
% Collaborators: N. Herrington, J. Schmitz, P. Yuvanondha
% Date: 5th Mar 2021

%% Housekeeping
clc;
clear;
close all;
tic

%% declare constants
m = 0.068; % mass of the quadrotor [kg]
R = 0.06; % radial distance from CG to propeller [m]
k_m = 0.0024; % control moment coefficient [N*m/N]
I_x = 5.8e-5; % x-axis moment of inertia [kg*m^2]
I_y = 7.2e-5; % y-axis moment of inertia [kg*m^2]
I_z = 1.0e-4; % z-axis moment of inertia [kg*m^2]
nu = 1e-3; % aerodynamic force coefficient [N/(m/s)^2]
mu = 2e-6; % aerodynamic moment coefficient [N*m/(rad/s)^2]
g = 9.81; % graviational constant [m/s^2]
lambda_1 = -2; % 1st eigenvalue
lambda_2 = -20; % 2nd eigenvalue

%% Problem 1
% Solve for k_1 and k_2 (Lateral)
syms k_1 k_2 % declare k1 and k2 as symbolic variables
eqn1 = lambda_1^2 + lambda_1*(k_1/I_x) + (k_2/I_x) == 0; % 1st eigenvalue solution
eqn2 = lambda_2^2 + lambda_2*(k_1/I_x) + (k_2/I_x) == 0; % 2nd eigenvalue solution

[A,b] = equationsToMatrix([eqn1, eqn2],[k_1, k_2]); % convert equations to a matrix

x_lat = double(A\b); % solve the system of equations
k_1 = x_lat(1); % define k_1 value from system solutions
k_2 = x_lat(2); % define k_2 value from system solutions
k_3 = 0;

% Solve for k_3 and k_4 (Longitudinal)
syms k_4 k_5 % declare k1 and k2 as symbolic variables
eqn1 = lambda_1^2 + lambda_1*(k_4/I_y) + (k_5/I_y) == 0; % 1st eigenvalue solution
eqn2 = lambda_2^2 + lambda_2*(k_4/I_y) + (k_5/I_y) == 0; % 2nd eigenvalue solution

[C,d] = equationsToMatrix([eqn1, eqn2],[k_4, k_5]); % convert equations to a matrix

x_long = double(C\d); % solve the system of equations
k_4 = x_long(1); % define k_3 value from system solutions
k_5 = x_long(2); % define k_4 value from system solutions
k_6 = 0;

%% Problem 2 (Closed-Loop Linearized)
% define known vector to pass into function
knowns = [mu nu I_x I_y I_z m g k_1 k_2 k_3 k_4 k_5 k_6 R k_m 0];
% define time span for ode45
t_span = [0 8];
% define motor trim forces
forces = (1/4)*m*g.*ones(1,4);
% define perturbations
perturbations = zeros(1,3);

% a (+5 deg in roll)
state_vec = zeros(1,12);
state_vec(4) = deg2rad(5);

[t_2a, state_2a] = ode45(@(t_2a,state_2a) linearizedEOM(t_2a,state_2a,forces,perturbations,knowns),t_span,state_vec);

% b (+5 deg in pitch)
state_vec = zeros(1,12);
state_vec(5) = deg2rad(5);

[t_2b, state_2b] = ode45(@(t_2b,state_2b) linearizedEOM(t_2b,state_2b,forces,perturbations,knowns),t_span,state_vec);

% c (+0.1 rad/sec in roll rate)
[p,q,r] = rollrate2pqr(0,0,0,0.1,0,0);
state_vec = zeros(1,12);
state_vec(10:12) = [p,q,r];

[t_2c, state_2c] = ode45(@(t_2c,state_2c) linearizedEOM(t_2c,state_2c,forces,perturbations,knowns),t_span,state_vec);

% d (+0.1 rad/sec in pitch rate)
[p,q,r] = rollrate2pqr(0,0,0,0,0.1,0);
state_vec = zeros(1,12);
state_vec(10:12) = [p,q,r];

[t_2d, state_2d] = ode45(@(t_2d,state_2d) linearizedEOM(t_2d,state_2d,forces,perturbations,knowns),t_span,state_vec);

%% Problem 2 Plots
plotP2N3(t_2a,state_2a,t_2b,state_2b,t_2c,state_2c,t_2d,state_2d,'2');

%% Problem 3 (Non-Linearized)
% a (+5 deg in roll)
state_vec = zeros(1,12);
state_vec(4) = deg2rad(5);
perturbations(2) = 0.5;

[t_3a, state_3a] = ode45(@(t_3a,state_3a) quadrotorODE(t_3a,state_3a,forces,knowns),t_span,state_vec);

% b (+5 deg in pitch)
state_vec = zeros(1,12);
state_vec(5) = deg2rad(5);

[t_3b, state_3b] = ode45(@(t_3b,state_3b) quadrotorODE(t_3b,state_3b,forces,knowns),t_span,state_vec);

% c (+0.1 rad/sec in roll rate)
[p,q,r] = rollrate2pqr(0,0,0,0.1,0,0);
state_vec = zeros(1,12);
state_vec(10:12) = [p,q,r];

[t_3c, state_3c] = ode45(@(t_3c,state_3c) quadrotorODE(t_3c,state_3c,forces,knowns),t_span,state_vec);

% d (+0.1 rad/sec in pitch rate)
[p,q,r] = rollrate2pqr(0,0,0,0,0.1,0);
state_vec = zeros(1,12);
state_vec(10:12) = [p,q,r];

[t_3d, state_3d] = ode45(@(t_3d,state_3d) quadrotorODE(t_3d,state_3d,forces,knowns),t_span,state_vec);

%% Problem 3 Plots
plotP2N3(t_3a,state_3a,t_3b,state_3b,t_3c,state_3c,t_3d,state_3d,'3');

%% Problem 4
k_3 = -18.475e-5; % found through guess and check
k_6 = 22.95e-5; % found through guess and check

% define known vector to pass into function
knowns = [mu nu I_x I_y I_z m g k_1 k_2 k_3 k_4 k_5 k_6 R k_m 0];

% a (+5 deg in roll)
state_vec = zeros(1,12);
state_vec(4) = deg2rad(5);

[t_4a, state_4a] = ode45(@(t_4a,state_4a) linearizedEOM(t_4a,state_4a,forces,perturbations,knowns),t_span,state_vec);

% b (+5 deg in pitch)
state_vec = zeros(1,12);
state_vec(5) = deg2rad(5);

[t_4b, state_4b] = ode45(@(t_4b,state_4b) linearizedEOM(t_4b,state_4b,forces,perturbations,knowns),t_span,state_vec);

% c (+0.1 rad/sec in roll rate)
[p,q,r] = rollrate2pqr(0,0,0,0.1,0,0);
state_vec = zeros(1,12);
state_vec(10:12) = [p,q,r];

[t_4c, state_4c] = ode45(@(t_4c,state_4c) linearizedEOM(t_4c,state_4c,forces,perturbations,knowns),t_span,state_vec);

% d (+0.1 rad/sec in pitch rate)
[p,q,r] = rollrate2pqr(0,0,0,0,0.1,0);
state_vec = zeros(1,12);
state_vec(10:12) = [p,q,r];

[t_4d, state_4d] = ode45(@(t_4d,state_4d) linearizedEOM(t_4d,state_4d,forces,perturbations,knowns),t_span,state_vec);

%% Problem 4 Plots
plotP2N3(t_4a,state_4a,t_4b,state_4b,t_4c,state_4c,t_4d,state_4d,'4');

%% Problem 5

% define known vector to pass into function
knowns = [mu nu I_x I_y I_z m g k_1 k_2 k_3 k_4 k_5 k_6 R k_m 1 1 0 2];

% a lateral
state_vec = zeros(1,12);

[t_5a, state_5a] = ode45(@(t_5a,state_5a) linearizedEOM(t_5a,state_5a,forces,perturbations,knowns),t_span,state_vec);

% define known vector to pass into function
knowns = [mu nu I_x I_y I_z m g k_1 k_2 k_3 k_4 k_5 k_6 R k_m 1 0 1 2];

% b longitudnal
state_vec = zeros(1,12);

[t_5b, state_5b] = ode45(@(t_5b,state_5b) linearizedEOM(t_5b,state_5b,forces,perturbations,knowns),t_span,state_vec);

%% Problem 5 Plots
figure
sgtitle(['Problem 5 Model (Lateral)']);
    % x position vs time
    subplot(4,3,1);
    plot(t_5a, state_5a(:,1));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('x [m]');
    title('x position vs time');
    % y position vs time
    subplot(4,3,2);
    plot(t_5a, state_5a(:,2));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('y [m]');
    title('y position vs time');
    % z position vs time
    subplot(4,3,3);
    plot(t_5a, state_5a(:,3));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('z [m]');
    title('z position vs time');
    % roll vs time
    subplot(4,3,4);
    plot(t_5a, state_5a(:,4));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('\phi [rad]');
    title('roll vs time');
    % pitch vs time
    subplot(4,3,5);
    plot(t_5a, state_5a(:,5));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    title('pitch vs time');
    % yaw vs time
    subplot(4,3,6);
    plot(t_5a, state_5a(:,6));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('\psi [rad]');
    title('yaw vs time');
    % x velocity vs time
    subplot(4,3,7);
    plot(t_5a, state_5a(:,7));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('u [rad]');
    title('x velocity vs time');
    % y velocity vs time
    subplot(4,3,8);
    plot(t_5a, state_5a(:,8));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('v [rad]');
    title('y velocity vs time');
    % z velocity vs time
    subplot(4,3,9);
    plot(t_5a, state_5a(:,9));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('w [rad]');
    title('z velocity vs time');
    % roll rate vs time
    subplot(4,3,10);
    plot(t_5a, state_5a(:,10));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('p [rad/s]');
    title('roll rate vs time');
    % pitch rate vs time
    subplot(4,3,11);
    plot(t_5a, state_5a(:,11));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('q [rad/s]');
    title('pitch rate vs time');
    % yaw rate vs time
    subplot(4,3,12);
    plot(t_5a, state_5a(:,12));
    grid on;
    xlim([0 t_5a(end)]);
    xlabel('time [s]');
    ylabel('r [rad/s]');
    title('rate rate vs time');
    
figure
sgtitle(['Problem 5 Model (Longitudnal)']);
    % x position vs time
    subplot(4,3,1);
    plot(t_5b, state_5b(:,1));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('x [m]');
    title('x position vs time');
    % y position vs time
    subplot(4,3,2);
    plot(t_5b, state_5b(:,2));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('y [m]');
    title('y position vs time');
    % z position vs time
    subplot(4,3,3);
    plot(t_5b, state_5b(:,3));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('z [m]');
    title('z position vs time');
    % roll vs time
    subplot(4,3,4);
    plot(t_5b, state_5b(:,4));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('\phi [rad]');
    title('roll vs time');
    % pitch vs time
    subplot(4,3,5);
    plot(t_5b, state_5b(:,5));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    title('pitch vs time');
    % yaw vs time
    subplot(4,3,6);
    plot(t_5b, state_5b(:,6));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('\psi [rad]');
    title('yaw vs time');
    % x velocity vs time
    subplot(4,3,7);
    plot(t_5b, state_5b(:,7));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('u [rad]');
    title('x velocity vs time');
    % y velocity vs time
    subplot(4,3,8);
    plot(t_5b, state_5b(:,8));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('v [rad]');
    title('y velocity vs time');
    % z velocity vs time
    subplot(4,3,9);
    plot(t_5b, state_5b(:,9));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('w [rad]');
    title('z velocity vs time');
    % roll rate vs time
    subplot(4,3,10);
    plot(t_5b, state_5b(:,10));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('p [rad/s]');
    title('roll rate vs time');
    % pitch rate vs time
    subplot(4,3,11);
    plot(t_5b, state_5b(:,11));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('q [rad/s]');
    title('pitch rate vs time');
    % yaw rate vs time
    subplot(4,3,12);
    plot(t_5b, state_5b(:,12));
    grid on;
    xlim([0 t_5b(end)]);
    xlabel('time [s]');
    ylabel('r [rad/s]');
    title('rate rate vs time');
        

%% End Housekeeping
toc
