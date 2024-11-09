clear all;
close all;

%% Parámetros de Simulación
Ts = 0.001;
Tf = 3;
Nsim = Tf/Ts;
t = 0:Ts:Tf;

%% Señal de Entrada - Set Point
r_sp = 2.5*ones(size(t));

%% Espacio de estados identificado
A = [0 1;-1.914e5 -3744];
B = [2.214; -7000];
C = [1 0];
D = 0;

sys_ss = ss(A, B, C, D);
SS_disc = c2d(sys_ss, Ts);

%% Funcion de transferencia circuito RC
% R = 10e3;
% C = 10e-6;
% 
% A = [(-1/(R*C)-1/(R*C)) (1/(R*C));(1/(R*C)) (-1/(R*C))];
% B = [1/(R*C);0];
% C = [0 1];
% D = 0;
% sys_ss = ss(A, B, C, D);
% SS_disc = c2d(sys_ss, Ts);

%% Tamaño vectores Espacio de Estados
nx = length(sys_ss.A);

%% Determinación de la matriz K - LQR
% Definir matrices de costo para LQR
Q = diag([1000000 0.001 1000]);
R = 0.000000001; % Tamaño de las columna de B

K_hat = lqi(SS_disc, Q, R);
K_new = K_hat(1:nx);
ki = K_hat(end);

%% Observador
x0 = zeros(1, nx)';

x_hat = ones(nx, Nsim + 1) .* x0;
pole1_obs = 0;
pole2_obs = -0.1;
p_obs = [pole1_obs pole2_obs];
L = place(SS_disc.A', SS_disc.C', p_obs); % Ganancia del observador
L = L';

%% Variables Iniciales
x = ones(nx, Nsim + 1) .* [0; 0];

% Salidas del simulador
yOut  = SS_disc.C*x0; 
tOut = 0;
uOut = 0;
dt = linspace(0, Ts, 10)';
integral_error = zeros(1, Nsim);
e = zeros(1, Nsim);
q = zeros(1, Nsim);
y_feedback = SS_disc.C*x0;

%% Representa el tiempo real, cuanto tiempo está corriendo el
% microcontrolador, el tiempo entre interrupciones el tiempo de muestreo
for k = 1:Nsim
    
    %if k >= 0.5/Ts
        %r_sp(k) = 3;
    %end
    % Microcontrolador %
    e(k) = r_sp(k) - y_feedback;

    % Sin integrador  
%     u(k) = - K*x(:, k);

    % Con integrador, sin referencia
%     e(k) = y_feedback;
%     u(k) = -ki*y_feedback - K_new*x(:, k);

    % Con integrador
    u(k) = -K_new*x_hat(:, k)-ki*q(k) ;

    if u(k) < 0
        u(k) = 0;
    elseif u(k) > 1000
        u(k) = 1000;
    end
    
    x_hat(:, k+1) = (SS_disc.A - L*SS_disc.C)*x_hat(:, k) + SS_disc.B*u(k) + L*y_feedback;
     
    uOut = [uOut u(k)];
    
    u = u(k)*ones(1, numel(dt)); % ZOH de la U - DAC

    % Sistema en la vida real %
    [y, tsim, XssOut] = lsim(sys_ss, u, dt, x(:, k)); % XssOut es la salida del Espacio de Estados. Cambiar
    x(:, k+1) = XssOut(end, :)'; % x es el vector de estados
    q(k+1) = q(k) + e(k) * Ts; % Integrador se obtiene con el modelo discretizado
    y_feedback = y(end, :); % Sampling - ADC

    tOut = [tOut (tsim' + t(k))];
    yOut = [yOut y'];
end

%% Gráficos
figure(1)
plot(tOut, yOut);
hold on
stairs(t, SS_disc.C*x);
plot(t, r_sp);
legend('output', 'sampled', 'set-point')
grid on

figure(2)
hold on
plot(t, uOut)
legend('u signal')
grid on

%% Gráficos de la estimación de los estados
figure(3)
hold on
for i = 1:nx
    plot(t, x_hat(i, 1:Nsim+1));
end
for i = 1:nx
    plot(t, x(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)','x(1)','x(2)');
title('Estimación de los estados');
xlabel('Tiempo [s]');
ylabel('Estados estimados');
grid on
