%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ECE 688 Project Simulation
% Combined backstepping-based controller simulation
% Simulation file
%
% Prepared for - Prof. Chris Nielsen
%
% Prepared by - Alexander Leung
%
% This file solves the closed loop system and creates some plots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulation Parameters
clc;
clear all;
close all;
options = odeset('RelTol',1e-15, 'abstol', 1e-15);
t_span = [0 100];


%% Set the initial conditions
x_0 = [1 -0.5 -2];
y_0 = [-1 1 2.5];
psi_0 = [1 2 0];
u_0 = [0 0 0];
v_0 = [0 0 0];
r_0 = [0 0 0];

x_d_0 = [0 0 0];
y_d_0 = [0 0 0];
psi_d_0 = [0 0 0];
u_d_0 = [0.1 0.1 0.1];      %constant forward velocity
v_d_0 = [0 0 0];            %zero sway reference
r_d_0 = [0.1 0.1 0.1];      %yaw velocity of 0.1 -> tracking a circle
    
%Create desired trajectory control signals.
%tau_sd=0.1 implies constant speed. tau_yd=0 means no angular
%acceleraton.   
tau_sd=0.1;
tau_yd=0;

% Now, assign ICs to state vector
X_0(1) = x_0(1); X_0(2) = y_0(1); X_0(3) = psi_0(1); 
X_0(4) = u_0(1); X_0(5) = v_0(1); X_0(6) = r_0(1); 
X_0(7) = x_d_0(1); X_0(8) = y_d_0(1); X_0(9) = psi_d_0(1); 
X_0(10) = u_d_0(1); X_0(11) = v_d_0(1); X_0(12) = r_d_0(1);
X_0(13) = tau_sd(1); X_0(14) = tau_yd(1); 



%% Simulate the system and create plots to show control signal tracking error
% Be sure to comment out the next section!!!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %BEGIN COMMENT SECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tau_s=[];   %declare empty var
% tau_y=[];   %declare empty var
% 
% [t, x] = ode45(@backstepping_DE, t_span, X_0, options); %% medium accuracy ODE solver, Runge-Kutta method
% for i=1:length(x)
%     [outx tau_s(i) tau_y(i)] = backstepping_DE(t(i),x(i,:));
% end
% tau_s=tau_s';
% tau_y=tau_y';
% 
% subplot(2,1,1);
% title('Tracking error using cascade backstepping-based controllers');
% xlabel('t');
% ylabel('tracking error');
% grid on;
% hold on;
% plot(t,x(:,1)-x(:,7),'k-');         %x_e plot
% plot(t,x(:,2)-x(:,8),'k--');        %y_e plot
% plot(t,x(:,3)-x(:,9),'k:', 'LineWidth',2);         %psi_e plot
% axis([0 80 -4 2]);
% legend('x_e','y_e','\psi_e');
% legend('show');
% subplot(2,1,2);
% title('Cascade backstepping-based control signals');
% xlabel('t');
% ylabel('control signals');
% grid on;
% hold on;
% plot(t,tau_s,'k-');
% plot(t,tau_y,'k--');
% axis([0 80 -0.5 2.5]);
% legend('\tau_s','\tau_y');
% legend('show');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %END COMMENT SECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Simulate system and create plots for phase plot
% Be sure to comment out the previous section!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %BEGIN COMMENT SECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
title('Phase trajectories for different initial conditions');
xlabel('x');
ylabel('y');
grid on;
hold on;

[t, x] = ode45(@backstepping_DE, t_span, X_0, options); 
plot(x(:,7),x(:,8),'k-', 'LineWidth',2);       %desired
%
plot(X_0(1), X_0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x(:,1),x(:,2),'k--');       %x-y plot for ic (1, -1, 1)
%
X_0(1) = x_0(2); X_0(2) = y_0(2); X_0(3) = psi_0(2); 
[t, x] = ode45(@backstepping_DE, t_span, X_0, options); 
plot(X_0(1), X_0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x(:,1),x(:,2),'k-.', 'LineWidth',2);       %x-y plot for ic (-0.5, 1, 2)
%
X_0(1) = x_0(3); X_0(2) = y_0(3); X_0(3) = psi_0(3); 
[t, x] = ode45(@backstepping_DE, t_span, X_0, options); 
plot(X_0(1), X_0(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x(:,1),x(:,2),'k:', 'LineWidth',2);       %x-y plot for ic (2.5, 2.5, 0)
%
axis([-3 3 -1.5 3]);
legend('Desired trajectory','','(x_0,y_0,\psi_0)=(1,-1,1)','','(x_0,y_0,\psi_0)=(-0.5,1,2)','','(x_0,y_0,\psi_0)=(-2,2.5,0)');
legend('show');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %END COMMENT SECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
