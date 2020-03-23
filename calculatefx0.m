% Ex12_1.m 
% Define the linearized model parameters 
u0=20; 
g=9.81; 
m=1000; 
f=0.015; 
Theta=0; 
rho=1.202; 
A=1; 
Cd=0.5; 
uw=2; 
% Calculate the equilibrium force, Fx0:
Fx0 = m*g*sin(Theta) + f*m*g + 0.5*rho*A*Cd*(u0+uw)^2; 
% The time constant and dc gain are: 
%Tau=((m)/(rho*A*Cd*(u0+uw))); 
Tau = 100;
K=68.5/m; 

first_block = m*g*sin(Theta);
second_block = f*m*g*cos(Theta);
third_block = 0.5*rho*A*Cd;

Thetaprim = 0.034;
first_block_prim = m*g*sin(Thetaprim);
second_block_prim = f*m*g*cos(Thetaprim);

% The throttle actuator is first-order with integrator:
Ka=10;
TauA=0.05; 
uc = 30;