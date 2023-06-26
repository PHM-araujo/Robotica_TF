clear; clc;

% Parametros Robô (comprimentos de cada elo)
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

%Angulos de junta da posição inicial:
q = [0 0 pi/2 0 pi/2];

%ponto de inicio - local que pegará o objeto
p1 = [2.5, 4, 1];
%orientacao de inicio:

%ponto de entrega - local que deixará o objeto
p2 = [0, 5, 0];

% Cinematica inversa - Base até p1:
% Gerar matriz de posição da ferramenta
pos_tool = [1 0 0 p1(1); 0 1 0 p1(2); 0 0 1 p1(3); 0 0 0 1];
q1 = Rbt.ikcon(pos_tool);

% Cinematica inversa - p2 até p2:
% Gerar matriz de posição da ferramenta
pos_tool = [1 0 0 p2(1); 0 1 0 p2(2); 0 0 1 p2(3); 0 0 0 1];
q2 = Rbt.ikcon(pos_tool);

% Gerar Trajetória por interpolação
% Vetor de tempo 0 - 2 em passos de 0.3
t=0:.03:2;
traj1 = jtraj(q, q1, t);
traj2 = jtraj(q1, q2, t);
%plot(t, traj1)

Rbt.plot(traj1)
Rbt.plot(traj2)









