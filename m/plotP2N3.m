function plotP2N3(t_2a,state_2a,t_2b,state_2b,t_2c,state_2c,t_2d,state_2d,N)

    %% Problem 2 Plots
    % a (+5 deg in roll)
    figure
    sgtitle(['Deviation by +5^{o} in Roll [' N '.a]']);
        % x position vs time
        subplot(4,3,1);
        plot(t_2a, state_2a(:,1));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('x [m]');
        title('x position vs time');
        % y position vs time
        subplot(4,3,2);
        plot(t_2a, state_2a(:,2));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('y [m]');
        title('y position vs time');
        % z position vs time
        subplot(4,3,3);
        plot(t_2a, state_2a(:,3));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('z [m]');
        title('z position vs time');
        % roll vs time
        subplot(4,3,4);
        plot(t_2a, state_2a(:,4));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('\phi [rad]');
        title('roll vs time');
        % pitch vs time
        subplot(4,3,5);
        plot(t_2a, state_2a(:,5));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('\theta [rad]');
        title('pitch vs time');
        % yaw vs time
        subplot(4,3,6);
        plot(t_2a, state_2a(:,6));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('\psi [rad]');
        title('yaw vs time');
        % x velocity vs time
        subplot(4,3,7);
        plot(t_2a, state_2a(:,7));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('u [rad]');
        title('x velocity vs time');
        % y velocity vs time
        subplot(4,3,8);
        plot(t_2a, state_2a(:,8));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('v [rad]');
        title('y velocity vs time');
        % z velocity vs time
        subplot(4,3,9);
        plot(t_2a, state_2a(:,9));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('w [rad]');
        title('z velocity vs time');
        % roll rate vs time
        subplot(4,3,10);
        plot(t_2a, state_2a(:,10));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('p [rad/s]');
        title('roll rate vs time');
        % pitch rate vs time
        subplot(4,3,11);
        plot(t_2a, state_2a(:,11));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('q [rad/s]');
        title('pitch rate vs time');
        % yaw rate vs time
        subplot(4,3,12);
        plot(t_2a, state_2a(:,12));
        grid on;
        xlim([0 t_2a(end)]);
        xlabel('time [s]');
        ylabel('r [rad/s]');
        title('rate rate vs time');

    % b (+5 deg in pitch)
    figure
    sgtitle(['Deviation by +5^{o} in Pitch [' N '.b]']);
        % x position vs time
        subplot(4,3,1);
        plot(t_2b, state_2b(:,1));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('x [m]');
        title('x position vs time');
        % y position vs time
        subplot(4,3,2);
        plot(t_2b, state_2b(:,2));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('y [m]');
        title('y position vs time');
        % z position vs time
        subplot(4,3,3);
        plot(t_2b, state_2b(:,3));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('z [m]');
        title('z position vs time');
        % roll vs time
        subplot(4,3,4);
        plot(t_2b, state_2b(:,4));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('\phi [rad]');
        title('roll vs time');
        % pitch vs time
        subplot(4,3,5);
        plot(t_2b, state_2b(:,5));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('\theta [rad]');
        title('pitch vs time');
        % yaw vs time
        subplot(4,3,6);
        plot(t_2b, state_2b(:,6));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('\psi [rad]');
        title('yaw vs time');
        % x velocity vs time
        subplot(4,3,7);
        plot(t_2b, state_2b(:,7));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('u [rad]');
        title('x velocity vs time');
        % y velocity vs time
        subplot(4,3,8);
        plot(t_2b, state_2b(:,8));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('v [rad]');
        title('y velocity vs time');
        % z velocity vs time
        subplot(4,3,9);
        plot(t_2b, state_2b(:,9));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('w [rad]');
        title('z velocity vs time');
        % roll rate vs time
        subplot(4,3,10);
        plot(t_2b, state_2b(:,10));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('p [rad/s]');
        title('roll rate vs time');
        % pitch rate vs time
        subplot(4,3,11);
        plot(t_2b, state_2b(:,11));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('q [rad/s]');
        title('pitch rate vs time');
        % yaw rate vs time
        subplot(4,3,12);
        plot(t_2b, state_2b(:,12));
        grid on;
        xlim([0 t_2b(end)]);
        xlabel('time [s]');
        ylabel('r [rad/s]');
        title('rate rate vs time');

    % c (+0.1 rad/s in roll rate)
    figure
    sgtitle(['Deviation by +0.1 rad/s in Roll Rate [' N '.c]']);
        % x position vs time
        subplot(4,3,1);
        plot(t_2c, state_2c(:,1));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('x [m]');
        title('x position vs time');
        % y position vs time
        subplot(4,3,2);
        plot(t_2c, state_2c(:,2));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('y [m]');
        title('y position vs time');
        % z position vs time
        subplot(4,3,3);
        plot(t_2c, state_2c(:,3));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('z [m]');
        title('z position vs time');
        % roll vs time
        subplot(4,3,4);
        plot(t_2c, state_2c(:,4));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('\phi [rad]');
        title('roll vs time');
        % pitch vs time
        subplot(4,3,5);
        plot(t_2c, state_2c(:,5));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('\theta [rad]');
        title('pitch vs time');
        % yaw vs time
        subplot(4,3,6);
        plot(t_2c, state_2c(:,6));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('\psi [rad]');
        title('yaw vs time');
        % x velocity vs time
        subplot(4,3,7);
        plot(t_2c, state_2c(:,7));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('u [rad]');
        title('x velocity vs time');
        % y velocity vs time
        subplot(4,3,8);
        plot(t_2c, state_2c(:,8));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('v [rad]');
        title('y velocity vs time');
        % z velocity vs time
        subplot(4,3,9);
        plot(t_2c, state_2c(:,9));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('w [rad]');
        title('z velocity vs time');
        % roll rate vs time
        subplot(4,3,10);
        plot(t_2c, state_2c(:,10));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('p [rad/s]');
        title('roll rate vs time');
        % pitch rate vs time
        subplot(4,3,11);
        plot(t_2c, state_2c(:,11));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('q [rad/s]');
        title('pitch rate vs time');
        % yaw rate vs time
        subplot(4,3,12);
        plot(t_2c, state_2c(:,12));
        grid on;
        xlim([0 t_2c(end)]);
        xlabel('time [s]');
        ylabel('r [rad/s]');
        title('rate rate vs time');

    % d (+0.1 rad/s in pitch rate)
    figure
    sgtitle(['Deviation by +0.1 rad/s in Pitch Rate [' N '.d]']);
        % x position vs time
        subplot(4,3,1);
        plot(t_2d, state_2d(:,1));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('x [m]');
        title('x position vs time');
        % y position vs time
        subplot(4,3,2);
        plot(t_2d, state_2d(:,2));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('y [m]');
        title('y position vs time');
        % z position vs time
        subplot(4,3,3);
        plot(t_2d, state_2d(:,3));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('z [m]');
        title('z position vs time');
        % roll vs time
        subplot(4,3,4);
        plot(t_2d, state_2d(:,4));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('\phi [rad]');
        title('roll vs time');
        % pitch vs time
        subplot(4,3,5);
        plot(t_2d, state_2d(:,5));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('\theta [rad]');
        title('pitch vs time');
        % yaw vs time
        subplot(4,3,6);
        plot(t_2d, state_2d(:,6));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('\psi [rad]');
        title('yaw vs time');
        % x velocity vs time
        subplot(4,3,7);
        plot(t_2d, state_2d(:,7));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('u [m/s]');
        title('x velocity vs time');
        % y velocity vs time
        subplot(4,3,8);
        plot(t_2d, state_2d(:,8));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('v [m/s]');
        title('y velocity vs time');
        % z velocity vs time
        subplot(4,3,9);
        plot(t_2d, state_2d(:,9));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('w [m/s]');
        title('z velocity vs time');
        % roll rate vs time
        subplot(4,3,10);
        plot(t_2d, state_2d(:,10));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('p [rad/s]');
        title('roll rate vs time');
        % pitch rate vs time
        subplot(4,3,11);
        plot(t_2d, state_2d(:,11));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('q [rad/s]');
        title('pitch rate vs time');
        % yaw rate vs time
        subplot(4,3,12);
        plot(t_2d, state_2d(:,12));
        grid on;
        xlim([0 t_2d(end)]);
        xlabel('time [s]');
        ylabel('r [rad/s]');
        title('rate rate vs time');

end