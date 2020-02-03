%% ASEN 3128 - Aircraft Dynamics Assignment 1
%{
Author: Timothy Breda
Collaborators:  Kyle Li, Sam Taylor, Michael Vogel, Tyler Pirner
Date: 1/30/20
Purpose: The purpose of this code is to track the motion and dynamics
parameters of a golf ball flying through the air.
*** BOULDER ATM ASSUMPTION *** not sea level
%}

%% Housekeeping
clc;clear;close all;

%% Variable Declaration
% golfball variables
m = .03; %mass of golf ball in kg
d = .03; %diameter in centimeters
cd = .6; %drag coefficient
% Other Parameters
g = 9.81;
[~,~,~,rho] = atmosisa(1607);
% area calc
r = .5*d;
A = pi*r^2;
%% Part A

% inertial velocitty represented in  body coordinates
Vel = [0,20,-20]; % N, E, D
X0 = [0,0,0,Vel];
time = linspace(0,5,100);
% wind definition <N,E,D>
w = [0,0,0];
% compiling the parameters into one cell array
params = {w,rho,g,A,cd,r,m};

% Integration and ODE Call
[t,X] = ode45(@(t,X) golfball(t,X,params),time,X0);
R = [X(:,1),X(:,2),X(:,3)];
V = [X(:,4),X(:,5),X(:,6)];

% cutting off data and only plotting after ball reaches the ground
i = 0;
test = 0;
while test<=0
    i = i+1;
    test = R(i,3);
end
% finding max height
maxZ = -min(R(:,3));
% finding distance to golf ball at the final location
distance = sqrt((R(i,1))^2 + (R(i,2)^2));
% Plotting in 3D
figure(1)
plot3(R(1:i,1),R(1:i,2),R(1:i,3));
grid minor
title('Golf Ball Trajectory without Wind');
xlabel('North (m)');
ylabel('East (m)');
zlabel('Down (m)');
set(gca,'Zdir','reverse');%flips z axis in graph

%% Part B: Varying Windspeeds

for j = 1:20

w = [j*5,0,0];
% compiling the parameters into one cell array
params = {w,rho,g,A,cd,r,m};

% Integration and ODE Call
[t,X] = ode45(@(t,X) golfball(t,X,params),time,X0);
R = [X(:,1),X(:,2),X(:,3)];
V = [X(:,4),X(:,5),X(:,6)];

% cutting off data and only plotting after ball reaches the ground
i = 0;
test = 0;
while test<=0
    i = i+1;
    test = R(i,3);
end

%Holding 
maxZ_vec(j) = -min(R(:,3));
% finding distance to golf ball at the final location
distance_vec(j) = sqrt((R(i,1))^2 + (R(i,2)^2));
    
% Plotting part b
figure(2)
plot3(R(1:i,1),R(1:i,2),R(1:i,3));
grid minor
title('Golf Ball Trajectories with Wind');
xlabel('North (m)');
ylabel('East (m)');
zlabel('Down (m)');
set(gca,'Zdir','reverse');%flips z axis in graph
hold on
end
vec = linspace(5,100,20);
% plotting distances to ball
% Plotting in 3D
figure(3)
plot(vec,distance_vec,'*');
grid minor
title('Golf Ball Distances varying Wind');
xlabel('Wind Speed (m/s)');
ylabel('Final Distance from Start to Golfball (m)');

%% Part C: Varying Mass 

% Solving for KE
vel_tot = sqrt((Vel(1))^2 + (Vel(2))^2 + (Vel(3))^2);
KE = .5*m*vel_tot^2;
% varying masses
mass = linspace(.001,.1,20);
% velocity magnitudes while keeping KE constant
vel_c = sqrt((2*KE)./mass);
% components of velocity to be used later
comp_vels = (1/sqrt(2)).*vel_c;
for k = 1:20
% inertial velocitty represented in  body coordinates
Vel = [0,comp_vels(k),-comp_vels(k)]; % N, E, D
X0 = [0,0,0,Vel];
time = linspace(0,5,100);
% no wind
w = [0,0,0];
m = mass(k);
% compiling the parameters into one cell array
params = {w,rho,g,A,cd,r,m};

% Integration and ODE Call
[t,X] = ode45(@(t,X) golfball(t,X,params),time,X0);
R = [X(:,1),X(:,2),X(:,3)];
V = [X(:,4),X(:,5),X(:,6)];

% cutting off data and only plotting after ball reaches the ground
i = 0;
test = 0;
while test<=0
    i = i+1;
    test = R(i,3);
end

distance_vec2(k) = sqrt((R(i,1))^2 + (R(i,2)^2));

end
%% Plotting Part C
figure(4)
plot(mass,distance_vec2,'*');
grid minor
title('Golf Ball Distances Varying Mass');
xlabel('Mass (kg)');
ylabel('Final Distance from Start to Golfball (m)');


%% Defining the ODE Function named "golfball"
function [dX] = golfball(t,X,P)

% Initial Condition Declaration
x_E = X(1);
y_E = X(2);
z_E = X(3);
xdot_E = X(4);
ydot_E = X(5);
zdot_E = X(6);

% Redefining Parameters from Above
w = P{1};
rho = P{2};
g = P{3};
A = P{4};
cd = P{5};
r = P{6};
m = P{7};

% Equations to determine outstanding forces 
v_rel = [xdot_E-w(1),ydot_E-w(2),zdot_E-w(3)]; %calculating relative wind
mag_V = norm(v_rel); %magnitude of V_rel
unit_V = v_rel/mag_V; % unit vector for heading
F_d = .5*rho*mag_V^2*cd*A*unit_V;%drag force
F_g = [0,0,g*m];%gravity
dRdt = [xdot_E,ydot_E,zdot_E];%v super E
dVdt = (-F_d+F_g)/m;%a super E

% Defining Outputs
dX(:,1:3) = dRdt;
dX(:,4:6) = dVdt;
dX = dX';
end

