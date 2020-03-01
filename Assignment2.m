%% Assignment 2 Code
%{
Name: Tim Breda
Collaborators: Sam Taylor, Michael Vogel, Stephen Albert, Tyler Pirner
Date: 2/6/20
Purpose: The purpose of this code is to model the flight dynamics and
kinematics of a quadcopter.
%}

%% Housekeeping
clc;clear;close all;

%% Variable Declaration

% Rolling Spider Properties
m = .068; %mass (kg)
rad = .06; %radius (m)
A = pi*rad^2; %area
cd = .6; %drag coefficient
% Moment of Inertia Components (kg*m^2)
I_x = 6.8e-5;
I_y = 9.2e-5;
I_z = 1.35e-4;
I = [I_x,I_y,I_z]'; % moments of inertia in vector form
% Other Parameters
[~,~,~,rho] = atmosisa(1607); % density of air in Boulder
g = [0,0,9.81]; % gravitational acceleration
k = .0024; %Control Moment Ceofficient
alpha = 2e-6; % Aerodynamic Moment Coefficient
beta = 1e-6; % Control Force Coefficient
eta = 1e-3; % Aerodynamic Force Coefficient
xi = 3e-3;
%% #1: Simulating Trim State (question 7)

Trim(1:4) = m*9.81/4;
% Defining Inital State Vectors
X0 = [1,1,-1,0,0,0,0,0,0,0,0,0];% [x,y,z,vx,vy,vz,p,q,r,psi,theta,phi]
tspan = [0 3]; %time
W = [0,0,0]; %wind vector

% Compiling Parameters into Cell Array
params = {Trim,W,rho,A,cd,rad,m};

% Integration and ODE Call
[t,X] = ode45(@(t,X) QuadCop(t,X,params),tspan,X0);
R = [X(:,1),X(:,2),X(:,3)]; % positions
V = [X(:,4),X(:,5),X(:,6)]; % velocities

% Plotting Trajectory vs. Time
figure(1)
%  plot3(R(:,1),R(:,2),R(:,3))
plot3(R(1,1),R(1,2),R(1,3),'ro')
title('Quadcopter Hover Flight')
grid minor
xlabel('N [m]'); 
ylabel('E [m]'); 
zlabel('D [m]');
% flipping z axis
set(gca, 'ZDir','reverse')
%% #2 5 m/s Translation (question 8) with Azimuth == 0 degrees

V_trans = 5; % translational velocity (m/s)

% Solving for phi, or bank angle needed to translate
phi = atan2((25*eta),(m*g(3)));
F_net = m*g(3)*cos(phi) + 25*xi*sin(phi)^2;
% assigning the forces on each rotor
Trim(1:4) = F_net/4;
% Initial State Vector [x,y,z,vx,vy,vz,p,q,r,psi,theta,phi]
X0 = [1,1,-1,0,V_trans*cos(phi),-V_trans*sin(phi),0,0,0,0,0,phi];
% Time interval for integration
tspan = [0 3];
% new parameters
params = {Trim,W,rho,g,A,cd,rad,m};
% ODE Call
[t,X] = ode45(@(t,X) QuadCop(t,X,params),tspan,X0);
R = [X(:,1),X(:,2),X(:,3)];
V = [X(:,4),X(:,5),X(:,6)];
Euler = [X(:,10),X(:,11),X(:,12)];

% R_new = zeros(length(t),3);
% V_new = zeros(length(t),3);
% Converting back to inertial coordinates
for i = 1:length(t)
   psi = X(i,10);
   theta = X(i,11);
   phi = X(i,12);
   Conv2Inert = [cos(theta)*cos(psi), ...
               sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
               cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
               cos(theta)*sin(psi), ...
               sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
               cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ... 
               sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)]^-1;
   Position = Conv2Inert*[X(i,1),X(i,2),X(i,3)]';
   Velocity = Conv2Inert*[X(i,4),X(i,5),X(i,6)]';
   Position = Position';
   Velocity = Velocity';
   R_new(i,1:3) = Position;    %(Conv2Inert*Position)';
   V_new(i,1:3) = Velocity;    %(Conv2Inert*Velocity)';
end


%% plotting translation over time
figure(2)
plot3(R_new(:,1),R_new(:,2),R_new(:,3));
title('Quadcopter Steady 5 m/s Translation');
grid minor
xlabel('N [m]'); 
ylabel('E [m]'); 
zlabel('D [m]');
zlim([-3 0]);
% flipping z axis
set(gca, 'ZDir','reverse')
% %% #2.5 5 m/s Translation (question 8) with Azimuth == 90 degrees
% V_trans = 5; % translational velocity (m/s)
% 
% % Solving for phi, or bank angle needed to translate
% theta = atan2((25*eta),(m*g(3)));
% F_net = m*g(3)*cos(theta) + 25*xi*sin(theta)^2;
% % assigning the forces on each rotor
% Trim(1:4) = F_net/4;
% % Initial State Vector [x,y,z,vx,vy,vz,p,q,r,psi,theta,phi]
% X0 = [1,1,-1,0,V_trans*cos(theta),-V_trans*sin(theta),0,0,0,0,0,theta];
% % Time interval for integration
% tspan = [0 3];
% % new parameters
% params = {Trim,W,rho,g,A,cd,rad,m};
% % ODE Call
% [t,X] = ode45(@(t,X) QuadCop(t,X,params),tspan,X0);
% R = [X(:,1),X(:,2),X(:,3)];
% V = [X(:,4),X(:,5),X(:,6)];
% Euler = [X(:,10),X(:,11),X(:,12)];
% 
% % R_new = zeros(length(t),3);
% % V_new = zeros(length(t),3);
% % Converting back to inertial coordinates
% for i = 1:length(t)
%    psi = X(i,10);
%    theta = X(i,11);
%    phi = X(i,12);
%    Conv2Inert = [cos(theta)*cos(psi), ...
%                sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
%                cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
%                cos(theta)*sin(psi), ...
%                sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
%                cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ... 
%                sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)]^-1;
%    Position = Conv2Inert*[X(i,1),X(i,2),X(i,3)]';
%    Velocity = Conv2Inert*[X(i,4),X(i,5),X(i,6)]';
%    Position = Position';
%    Velocity = Velocity';
%    R_new(i,1:3) = Position;    %(Conv2Inert*Position)';
%    V_new(i,1:3) = Velocity;    %(Conv2Inert*Velocity)';
% end
% 
% 
% % plotting translation over time
% figure(3)
% plot3(R_new(:,1),R_new(:,2),R_new(:,3));
% title('Quadcopter Steady 5 m/s Translation');
% grid minor
% xlabel('N [m]'); 
% ylabel('E [m]'); 
% zlabel('D [m]');
% zlim([-3 0]);
% % flipping z axis
% set(gca, 'ZDir','reverse')
%% ODE Function Creation
function[dX] = QuadCop(t,X,Pa)

%Redefining Quadcopter Properties
m = .068; %mass (kg)
rad = .06; %radius (m)
k = .0024; %(Nm/N)
% Moment of Inertia Components (kg*m^2)
I_x = 6.8e-5;
I_y = 9.2e-5;
I_z = 1.35e-4;
[~,~,~,rho] = atmosisa(1607); % density of air in Boulder
g = 9.81; % gravitational acceleration
alpha = 2e-6;
beta = 1e-6;
eta = 1e-3;
xi = 3e-3;
% Redefining Parameters
Trim=Pa{1};W=Pa{2};
% Redefining State Vector Quantities
% inertial positions (inertial coordinates)
xE = X(1);
yE = X(2);
zE = X(3);
% inertial velocity (body coordinates)
u = X(4);
v = X(5);
w = X(6);
% inertial angular velocities (body coordinates)
p = X(7);
q = X(8);
r = X(9);
% Euler Angles
psi = X(10);
theta = X(11);
phi = X(12);
% Forces
f1 = Trim(1);
f2 = Trim(2);
f3 = Trim(3);
f4 = Trim(4);
% Aerodynamic Forces
X_a = -eta^2*u^2*sign(u);
Y_a = -eta^2*v^2*sign(v);
Z_a = -xi^2*w^2*sign(w);
% Control Forces
X_c = 0;
Y_c = 0;
Z_c = -sum(Trim);
% Compiling Forces
X = X_a + X_c;
Y = Y_a + Y_c;
Z = Z_a + Z_c;
% Aerodynamic Moments
L_a = -alpha^2*p^2*sign(p);
M_a = -alpha^2*q^2*sign(q);
N_a = -beta^2*r^2*sign(r);
% Control Moments
L_c = (rad/sqrt(2))*(f1+f2-f3-f4);
M_c = (rad/sqrt(2))*(f2+f3-f1-f4);
N_c = k*(f1-f2+f3-f4);
% Compiling Moments
L = L_a + L_c;
M = M_a + M_c;
N = N_a + N_c;
% Translational Rates
u_dot = r*v-q*w-g*sin(theta)+1/m*X;
v_dot = p*w-r*u+g*sin(phi)*cos(theta)+1/m*Y;
w_dot = q*u-p*v+g*cos(phi)*cos(theta)+1/m*Z;
dVbdt = [u_dot,v_dot,w_dot]';
% Attitude Rates
p_dot = (I_y-I_z)/I_x*q*r+1/I_x*L;
q_dot = (I_z-I_x)/I_y*p*r+1/I_y*M;
r_dot = (I_x-I_y)/I_z*p*q+1/I_z*N;
dOmegabdt = [p_dot,q_dot,r_dot]';
% Conversion Matrices
ConvTrans = [cos(theta)*cos(psi), ...
               sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
               cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
               cos(theta)*sin(psi), ...
               sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
               cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ... 
               sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta)];
% p,q,r to phidot,thetadot,psidot  
ConvAtt = [1,sin(phi)*tan(theta),cos(phi)*tan(theta); ...
             0,cos(phi),-sin(phi); ... 
             0,sin(phi)*sec(theta),cos(phi)*sec(theta)];
         
% Conversion to inertial frame from body frame
dVEdt = ConvTrans*dVbdt;
% Conversion from Eulerian Angular Momentum
dEulerdt = ConvAtt*dOmegabdt;

% Output from ODE
dX = [dVEdt;dVbdt;dOmegabdt;dEulerdt];
end

