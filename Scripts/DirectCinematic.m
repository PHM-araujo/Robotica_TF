clear; clc;

% Parametros Robô
l0 = 2;
l1 = 2;
l2 = 2;
l3 = 2;

% Cinematica direta
%L = Link([theta d a alpha]);
L1=Link([0 0 0 0]);
L2=Link([0 0 l0 pi/2]);
L3=Link([0 0 l1 0]);
L4=Link([0 0 l2 0]);
L5=Link([0 0 l3 0]);

Rbt=SerialLink([L1 L2 L3 L4 L5]);
q = [0 0 0 0 0];

% Cinematica inversa
% Gerar matriz de posição da ferramenta
x = 1;
y = 1;
z = 1;
pos_tool = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
q1 = Rbt.ikcon(pos_tool)

% Gerar Trajetória
% Vetor de tempo 0 - 2 em passos de 0.3
t=0:.03:2;

traj1 = jtraj(q, q1, t);

plot(t, traj1)

%Rbt.plot(traj1)









