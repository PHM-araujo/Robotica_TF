clear; clc;

% Parametros Robô (comprimentos de cada elo)
l0 = 5;
l1 = 4;
l2 = 4;
l3 = 0;
l4 = 0;
l5 = 0;
%L = Link([theta d a alpha]);

%    theta  d  a  alpha:
%    theta1 0  3  pi/2
%    theta2
%    theta3
%    theta4


L1=Link([0 0 l0 pi/2]); 
L2=Link([0 0 l1 -pi/2]); L2.qlim = [-2*pi/3, 2*pi/3];
L3=Link([0 0 l2 pi/2]);  L3.qlim = [-2*pi/3, 2*pi/3];
L4=Link([0 0 l3 pi/2]);  L4.qlim = [-2*pi/3, 2*pi/3];
L5=Link([0 0 l4 pi/2]);  L5.qlim = [-2*pi/3, 2*pi/3];
L6=Link([0 0 l5 pi/2]);  L6.qlim = [-2*pi/3, 2*pi/3];

%geração do robô com base na definição dos elos e juntas:
Rbt=SerialLink([L1 L2 L3 L4 L5 L6]);
%Angulos de junta da posição inicial:
q = [0 0 0 0 0 0];

% Mostrando cinematica direta obtida:
H = fkine(Rbt,q);
%ponto de inicio - local de coleta do objeto
% p1 = [2, 3,3];

p1 = [0, 3,3];
%ponto de entrega - local que deixará o objeto

% p2 = [2, 2, 7];
p2 = [-10, 3, 4];

% Cinematica inversa - Base até p1:
% Gerar matriz de posição da ferramenta
pos_tool_1 = [1 0 0 p1(1); 0 1 0 p1(2); 0 0 1 p1(3); 0 0 0 1];
ik1 = Rbt.ikine(pos_tool_1);
% ik1 = Rbt.ikine3(pos_tool);

% Cinematica inversa - p1 até p2:
% Gerar matriz de posição da ferramenta
pos_tool_2 = [1 0 0 p2(1); 0 1 0 p2(2); 0 0 1 p2(3); 0 0 0 1];
ik2 = Rbt.ikine(pos_tool_2);

% Gerar Trajetória por interpolação
% Vetor de tempo 0 - 2 em passos de 0.3
t=0:.06:2;
traj1 = jtraj(q, ik1, t);
traj2 = jtraj(ik1, ik2, t);
%plot(t, traj1)

plot2(p1(1,:),'r.')
Rbt.plot(traj1)
plot2(p2(1,:),'r.')
Rbt.plot(traj2)






%{
l0 = 5;
l1 = 4;
l2 = 4;
l3 = 4;
l4 = 4;

L1=Link([0 0 l0 pi/2]);
L2=Link([0 0 l1 -pi/2]);
L3=Link([0 0 l2 pi/2]);
L4=Link([0 0 l3 pi/2]);
L5=Link([0 0 l4 0]);
Rbt=SerialLink([L1 L2 L3 L4]);


%Angulos de junta da posição inicial:
q = [0 0 pi/2 0];

% Mostrando cinematica direta obtida:
H = fkine(Rbt,q);
%ponto de inicio - local de coleta do objeto
p1 = [2, 3,3];

%ponto de entrega - local que deixará o objeto
p2 = [-5, 1, 5];

% Cinematica inversa - Base até p1:
% Gerar matriz de posição da ferramenta
pos_tool = [1 0 0 p1(1); 0 1 0 p1(2); 0 0 1 p1(3); 0 0 0 1];
ik1 = Rbt.ikcon(pos_tool);
% ik1 = Rbt.ikine3(pos_tool);

% Cinematica inversa - p1 até p2:
% Gerar matriz de posição da ferramenta
pos_tool = [1 0 0 p2(1); 0 1 0 p2(2); 0 0 1 p2(3); 0 0 0 1];
ik2 = Rbt.ikcon(pos_tool);

% Gerar Trajetória por interpolação
% Vetor de tempo 0 - 2 em passos de 0.3
t=0:.06:2;
traj1 = jtraj(q, ik1, t);
traj2 = jtraj(ik1, ik2, t);
%plot(t, traj1)

Rbt.plot(traj1)
Rbt.plot(traj2)
%}


