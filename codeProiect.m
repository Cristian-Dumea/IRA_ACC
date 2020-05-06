close all;
clear all;
clc;

TauC=100;
kc=0.0685;
% Matricile sistemului
A = [0 -1 0 0; 0 -1/TauC 0 0; 1 0 0 0; 0 0 1 0]
B = [0; kc/TauC; 0; 0]
C=eye(4)
C1 = [1 0 0 0]
D=zeros(4,1);
D1=0;
In=eye(4);
% Perioada de esantionare
Ts=0.25;
% Discretizare
[Ad1,Bd1,Cd1,Dd1] = c2dm(A,B,C1,D1,Ts,'zoh');
%  Introducerea unui integrator si calcularea modelului extins
%   Modelul extins al partii fixate
%   A1=[Ad1 zeros(2,1); -Cd1 1]
%   B1=[Bd1;0];
%   C1=[-Cd1 0];
%   D1=0;
 % Matricea de controlabilitate
 R1=[Bd1 Ad1*Bd1 Ad1*Ad1*Bd1 Ad1*Ad1*Ad1*Bd1];
% % Ultima linie a lui R^-1
R_inv=inv(R1);
H1=R_inv(4,:)

% % Date
 sigma=0.0043;
 tt=1;
zeta=abs(log(sigma)/sqrt(pi^2+log(sigma)^2));
omegan=3/(tt*zeta);
% % alfa1, alfa2 
alfa1=-2*exp(-zeta*omegan*Ts)*cos(omegan*Ts*sqrt(1-zeta^2));
alfa2=exp(-2*zeta*omegan*Ts);
alfa3=exp(-5*omegan*Ts);
% % Polinomul caracteristic
I1=eye(4);
P_cr1=(Ad1*Ad1+alfa1*Ad1+ alfa2*I1)*(Ad1-alfa3*I1)*(Ad1-alfa3*I1)
% % Matricea de reactie dupa stare cu integrator
fi=-H1*P_cr1
% 
% %% Reglarea dupa stare estimata
% % 
% % % Polinomul caracteristic al estimatorului
%  Pce = Ad1^2;
% % % Matricea de observabilitate
%  O =[-Cd1 ; -Cd1*Ad1; -Cd1*Ad1*Ad1; -Cd1*Ad1*Ad1*Ad1]
% % % Ultima coloana din O^-1
%   O_inv=inv(O);
%   g=O_inv(:, 4);
% % % Matricea estimatorului
%   L=Pce*g