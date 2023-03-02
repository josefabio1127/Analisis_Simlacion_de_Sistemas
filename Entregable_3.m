clear variables
clc
close all
% Ganancia del controlador
k = 5

% Constantes del sistema
R = 1;
L = 0.1;
kA = 0.6;
kf = 0.6;
r = 0.5;
J = 5;
b = 3;
m = 4;
kp = 0.5;
kt = 1;

% Integrador y ecuaciones que se repiten varias veces
I = tf(1, [1 0]);
den_x2 = J + m*r^2;
den_x3 = L*kt + kA*kf;

% Parte mecanica del motor
M = tf(1, [den_x2 b*r^2]);

% Simplificacion de parte electrica del motor
E = tf(kt, den_x3);
P1 = E*I;
P2 = P1/(1 + P1*R);

% Simplificacion de la planta
P3 = P2*kf*M;
P4 = P3/(1 + P3*kA)*I;

% Funciones de trasferencia del sistema
G = k*P4*r;
H = kp;
LA = G*H;
LC = G/(1+G*H);
G1H1= LA/k;


%----------------------------------------------Lugar de las raices---------------------------------------------%
figure('Name','Lugar de las raices','NumberTitle','off');
rlocus(G1H1);

%------------------------------------Función de tranferencia a lazo abierto------------------------------------%
% Respuesta ante una entrada escalón y un impulso.
figure('Name','Función de tranferencia a lazo abierto con escalón unitario','NumberTitle','off');
step(LA);
figure('Name','Función de tranferencia a lazo abierto con un impulso','NumberTitle','off');
impulse(LA);

%--------------------------------Función de tranferencia de trayectoria directa--------------------------------%
% Respuesta ante una entrada escalón y un impulso.
figure('Name','Función de tranferencia de trayectoria directa con escalón unitario','NumberTitle','off');
step(G);
figure('Name','Función de tranferencia de trayectoria directa con un impulso','NumberTitle','off');
impulse(G);

%------------------------------------Función de tranferencia a lazo cerrado------------------------------------%
% Respuesta ante una entrada escalón y un impulso.
figure('Name','Función de tranferencia a lazo cerrado con escalón unitario','NumberTitle','off');
step(LC);
figure('Name','Función de tranferencia a lazo cerrado con un impulso','NumberTitle','off');
impulse(LC);

%---------------------------------------Respuesta de variables de estado---------------------------------------%
% Matrices de funciones dinamicas
C_3 = [r 0 0; 0 r 0; 0 0 r];
C = [r 0 0];
D = 0;

% Para el calculo de los limites de k, se utilizo el punto de ruptura como
% referencia para el valor critico, para el limite superior del caso 
% subamortiguado se utilizo el valor de k en el limite de estabilidad y
% para el limite inferior del caso sobreamortiguado se tiene el valor
% minimo de k, que en este caso es 0

% Para que el sistema sea subamortiguado el valor de k debe ser mayor a 0.337 y
% menor a 17.01
k_sub = 8;
A_sub = [0 1 0 ; 0 -(b*r^2)/den_x2 kf/den_x2 ; -(k_sub*kt*kp*r)/den_x3 -(kA*kt)/den_x3 -(kt*R)/den_x3];
B_sub = [0 ; 0 ; (k_sub*kt)/den_x3];
sys_sub = ss(A_sub, B_sub, C_3, D);
figure('Name','Respuesta de variables de estado caso subamortiguado','NumberTitle','off');
step(sys_sub);

% Para que el sistema sea críticamente amortiguado el valor de k debe ser
% igual a 0.337
k_crit = 0.337;
A_crit = [0 1 0 ; 0 -(b*r^2)/den_x2 kf/den_x2 ; -(k_crit*kt*kp*r)/den_x3 -(kA*kt)/den_x3 -(kt*R)/den_x3];
B_crit = [0 ; 0 ; (k_crit*kt)/den_x3];
sys_crit = ss(A_crit, B_crit, C_3, D);
figure('Name','Respuesta de variables de estado caso críticamente amortiguado','NumberTitle','off');
step(sys_crit);

% Para que el sistema sea sobreamortiguado el valor de k debe ser mayor a 0
% y menor a 0.337
k_sobre = 0.2;
A_sobre = [0 1 0 ; 0 -(b*r^2)/den_x2 kf/den_x2 ; -(k_sobre*kt*kp*r)/den_x3 -(kA*kt)/den_x3 -(kt*R)/den_x3];
B_sobre = [0 ; 0 ; (k_sobre*kt)/den_x3];
sys_sobre = ss(A_sobre, B_sobre, C_3, D);
figure('Name','Respuesta de variables de estado caso sobreamortiguado','NumberTitle','off');
step(sys_sobre);

% Respuesta ante una entrada escalón con diferentes valores de k.
vark = [0.2 0.4 0.6 0.8 1 1.2 1.4 1.6 1.8];
figure('Name','Respuesta del sistema en lazo cerrado para variaciones de 𝐾','NumberTitle','off');
hold on;
for k1 = vark
    % Matrices de funciones dinamicas faltantes
    A = [0 1 0 ; 0 -(b*r^2)/den_x2 kf/den_x2 ; -(k1*kt*kp*r)/den_x3 -(kA*kt)/den_x3 -(kt*R)/den_x3];
    B = [0 ; 0 ; (k1*kt)/den_x3];
    sys = ss(A, B, C, D);
    step(sys);
end
legend("k = 0.2", "k = 0.4", "k = 0.6", "k = 0.8", "k = 1", "k = 1.2", "k = 1.4", "k = 1.6", "k = 1.8", 'Location', 'southeast');
hold off;
