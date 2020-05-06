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

A = [0 -1; 0 -1/TauC];
B = [0; kc/TauC];
C = [1 0];
D = 0;
Ts = 0.01;

Distanta = 50;
C=eye(4);
C1 = [1 0];
D1=0;
In=eye(4);
% Perioada de esantionare
Ts=0.01;
% Discretizare
ad = eye(2)+A*Ts;
bd = B*Ts+0.5*A*B*Ts;
cd = [-Ts 0];
dd = 0;
% [Ad1,Bd1,Cd1,Dd1] = c2dm(A,B,C1,D1,Ts,'zoh');
% [Ad2,Bd2,Cd2,Dd2] = c2dm(ad,bd,cd,dd,Ts,'zoh');


%% Date
sigma=0.043;
tt=10;
zeta=abs(log(sigma)/sqrt(pi^2+log(sigma)^2));
omegan=6*zeta/tt;

%% alfa1, alfa2
alfa1=-2*exp(-zeta*omegan*Ts)*cos(omegan*Ts*sqrt(1-zeta^2));
alfa2=exp(-2*zeta*omegan*Ts);
alfa3=exp(-5*omegan*Ts);

%% Polinomul caracteristic
I1=eye(4);
Ad1=[ad zeros(2);-Ts*1 0 1 0; 0 0 Ts*1 1];
Bd1=[bd; 0; 0];
%% Matricea de controlabilitate
R1=[Bd1 Ad1*Bd1 Ad1*Ad1*Bd1 Ad1*Ad1*Ad1*Bd1];

%% Ultima linie a lui R^-1
R_inv = inv(R1);
H1 = R_inv(4,:);

P_cr1=(Ad1*Ad1+alfa1*Ad1+alfa2*I1)*(Ad1-alfa3*eye(4))*(Ad1-alfa3*eye(4));

%% Matricea de reactie dupa stare cu integrator
fi=-1 * H1 * P_cr1;
K1 = fi(1);
K2 = fi(2);
K3 = fi(3);
K4 = fi(4);
