

% Defintion des paramètres du système
mr= 0.4; % masse roue + moteur
mp= 2.3; % masse pendule
L= 0.355; % longueur du pendule
%J=  m*l^2/3; % inertie du pendule
g= 9.81; % cste de gravité
d1= 0.2; % coeff. de frottement visqueux roue
d2= 0.1; % coeff. de frottement visqueux pendule

Rroue=0.072; % m rayon de la roue

A=[0 0 1 0;0 0 0 1;0 g*mp/mr -d1/mr -d2/(L*mr);0 g*(mr+mp)/(L*mr) -d1/(L*mr) -d2*(mr+mp)/(L^2*mr*mp)];
B=[0;0;1/mr;1/(L*mr)];

Q=100*eye(4);
R=1;
[K_lqr,S,P1] = lqr(A,B,Q,R);


ki=0.0245;


Fmax=0.176*2/Rroue; %Forcemax
a=-15.7/0.69;
b=15.7;
R=12/2.8; %approximation de la résistance interne rotor bloqué
%%Graphique
figure(3)
load('couple.mat')


plot(coupleinput)
ylabel('C(N.m)');
xlabel('t(s)');
title('Couple moteur')
legend('Couple')
%legend('Couple')