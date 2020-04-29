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
% Tau = ((m)/(rho*A*Cd*(u0+uw)));
% K = 1/Tau;
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
uc = 20;

KU = 29;
TU = 14;


%% Regulator PID Cruise Control - Facut de mana
TI_M = TU*7;
TD_M = TU/9;

KP_M = KU*6;
KI_M = KP_M/TI_M;
KD_M = KP_M*TD_M;

%% Regulator PID Adaptive Cruise Control - Facut de mana
KU_ACC = 1;
D = 50; % distanta de referinta

%% Performante:
% Timpul de raspuns: 28s
% Suprareglarea: 4.3%


%% Model intrare-stare-iesire
TauC = Tau;
kc = K;
k1 = 1;
k2 = 1;
k3 = 1;
k4 = 1;
A = [0 -1 0 0; 0 -1/TauC 0 0; 1 0 0 0; 0 0 1 0]
B = [0; kc/TauC; 0; 0]
C = [-k1 -k2 -k3 -k4]
D = 0

sys = ss(A, B, C, D)





