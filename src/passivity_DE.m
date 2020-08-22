%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ECE 688 Project Simulation
% Passivity-based controller simulation
% DE file
%
% Prepared for - Prof. Chris Nielsen
%
% Prepared by - Alexander Leung
%
% This file implement system dynamics and simulates
% the closed-loop system as well as reference trajectory.
%
% Controller: eqns (17) and (26) from Jiang.
%               (eqns (12) and (21) in my paper)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [dx,tau_s,tau_y] = passivity_DE( t, X)
    %Plant (boat) properties
    %Damping
    d11=0;
    d22=0.2;
    d33=0;
    %Mass/intertia
    m11=0.1;
    m22=0.1;
    m33=0.1;
    
    %Controller properties/parameters
    lamb0=10;
    lamb1=8;
    lamb2=5;
    c1=175;
    c2=175;
    c3=175;

    %Assign state variables
    x = X(1);
    y = X(2);
    psi = X(3);
    u = X(4);
    v = X(5);
    r = X(6);
    x_d = X(7);
    y_d = X(8);
    psi_d = X(9);
    u_d = X(10);
    v_d = X(11);
    r_d = X(12);
    tau_sd = X(13);
    tau_yd = X(14);
    

    %% System dynamics
    %Actual system dynamics 
    A=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    dx(1:3,1) = A*[u v r]';
    x_dot=dx(1,1);
    y_dot=dx(2,1);
    psi_dot=dx(3,1);
    
    %Virtual system dynamics
    A_d=[cos(psi_d) -sin(psi_d) 0; sin(psi_d) cos(psi_d) 0; 0 0 1];
    dx(7:9,1) = A_d*[u_d v_d r_d]';
    x_d_dot=dx(7,1);
    y_d_dot=dx(8,1);
    psi_d_dot=dx(9,1);
    %u_d_dot=(m22/m11)*v_d*r_d-(d11/m11)*u_d+(1/m11)*tau_sd;
    u_d_dot=0;
    dx(10,1)=u_d_dot;
    v_d_dot=-(m11/m22)*u_d*r_d-(d22/m22)*v_d;
    dx(11,1)=v_d_dot;
    %r_d_dot=((m11-m22)/m33)*u_d*v_d-(d33/m33)*r_d+(1/m33)*tau_yd;
    r_d_dot=0;
    dx(12,1)=r_d_dot;
    
    
    %% State transform to error signals
    z_1=x*cos(psi)+y*sin(psi);    
    z_1_dot=x_dot*cos(psi)-x*sin(psi)*psi_dot+y_dot*sin(psi)+y*cos(psi)*psi_dot;
    z_2=-x*sin(psi)+y*cos(psi);
    z_2_dot=-x_dot*sin(psi)-x*cos(psi)*psi_dot+y_dot*cos(psi)-y*sin(psi)*psi_dot;
    z_3=psi;
    z_2_dot=psi_dot;
    z_1_d=x_d*cos(psi_d)+y_d*sin(psi_d);
    z_2_d=-x_d*sin(psi_d)+y_d*cos(psi_d);
    z_3_d=psi_d;
    
    %Define error variables
    z_1e=z_1-z_1_d;
    z_2e=z_2-z_2_d;
    z_3e=z_3-z_3_d;
    u_e=u-u_d;
    v_e=v-v_d;
    r_e=r-r_d;
    
    %Error dynamics
    z_1e_dot=u_e+z_2e*r_d+z_2*r_e;
    z_2e_dot=v_e-z_1e*r_d-z_1*r_e;
    z_3e_dot=r_e;
    %u_e_dot=m22/m11*(v*r-v_d*r_d)-d11/m11*u_e+(1/m11)*(tau
    v_e_dot=-(m11/m22)*u_e*r_d-(d22/m22)*v_e-(m11/m22)*u*r_e;
    
    
	%% Implement passivity-based controllers (17) and (26)
    % (eqn 17):
    tau_s=tau_sd+m11*(-(m22/m11)*(v*r-v_d*r_d)+(d11/m11)*u_e-c1*(u_e+...
        lamb2*(z_1e-lamb1*z_2e*r_d))-((z_1e-lamb1*z_2e*r_d)-...
        lamb0*m11/m22*r_d*v_e)-lamb2*(u_e+z_2e*r_d+z_2*r_e)+...
        lamb1*lamb2*r_d_dot*z_2e+lamb1*lamb2*r_d*(v_e-z_1e*r_d-z_1*r_e)); 
    %Actuator dynamics u_dot (or \dot{u}), function of tau_s
    u_dot=(m22/m11)*v*r-(d11/m11)*u+(1/m11)*tau_s;
    dx(4,1)=u_dot;
    
    alpha_1=-c2*((z_1e-lamb1*z_2e*r_d)*(z_2+lamb1*r_d*z_1)-z_2e*z_1-...
        lamb0*m11/m22*v_e*u+z_3e);
    alpha_1_dot=-c2*((z_1e_dot-lamb1*(z_2e_dot*r_d+z_2e*r_d_dot))*(z_2...
        +lamb1*r_d*z_1)+(z_1e-lamb1*z_2e*r_d)*(z_2_dot+...
        lamb1*(r_d_dot*z_1+r_d*z_1_dot))-z_2e_dot*z_1-z_2e*z_1_dot-...
        lamb0*m11/m22*(v_e_dot*u+v_e*u_dot)+z_3e_dot);
    r_e_bar=r_e-alpha_1;
    
    % (eqn 26):
    tau_y=tau_yd+m33*(-(m11-m22)/m33*(u*v-u_d*v_d)+d33/m33*r_e-...
        c3*r_e_bar+alpha_1_dot-((z_1e-lamb1*z_2e*r_d)*(z_2+...
        lamb1*r_d*z_1)-z_2e*z_1-lamb0*m11/m22*v_e*u+z_3e));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Actuators/control/other signals DE 
    dx(5,1)=-(m11/m22)*u*r-(d22/m22)*v;
    dx(6,1)=((m11-m22)/m33)*u*v-(d33/m33)*r+(1/m33)*tau_y;
 
    dx(13,1)=0; %do not change tau_sd
    dx(14,1)=0; %do not change tau_yd
end

