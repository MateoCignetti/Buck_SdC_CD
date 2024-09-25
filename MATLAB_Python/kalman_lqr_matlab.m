clear all;
close all;
%% Parámetros de Simulación
Ts = 0.001;
Tf = 1;
Nsim = Tf/Ts;
t = 0:Ts:Tf;

%% Señal de Entrada - Set Point
r_sp = 4.5*ones(size(t));

%% Espacio de estados identificado
A = [0 1;-1.914e5 -3744];
B = [2.214; -7000];
C = [1 0];
D = 0;

sys_ss = ss(A, B, C, D);

SS_disc = c2d(sys_ss, Ts);

%% Tamaño vectores Espacio de Estados
nx = length(SS_disc.A);

%% Determinación de la matriz K - LQR
Q = diag([10000 10 10000]);
R = 0.1;

K_hat = lqi(SS_disc, Q, R);
K_new = K_hat(1:nx);
ki = K_hat(end);

%% Observador mediante Kalman
x0 = zeros(1, nx)';
x_hat = ones(nx, Nsim + 1) .* x0; % Estados estimados

% Definir las matrices de covarianza del proceso y de la medición
Q_kalman = 1e-3 * eye(nx);  % Matriz de covarianza del ruido del proceso
R_kalman = 1e-3;            % Covarianza del ruido de medición (ajústalo según el sistema)
P_kalman = eye(nx);               % Matriz de covarianza inicial
P_kalman_pred = eye(nx).*0;

%% Variables Iniciales
x = ones(nx, Nsim + 1) .* x0;

% Salidas del simulador
yOut  = SS_disc.C*x0; 
tOut = 0;
uOut = 0;
dt = linspace(0, Ts, 10)';
integral_error = zeros(1, Nsim);
e = zeros(1, Nsim);
q = zeros(1, Nsim);
y_feedback = SS_disc.C*x0;
lastu = 0;

%% Representa el tiempo real, cuanto tiempo está corriendo el
% microcontrolador, el tiempo entre interrupciones el tiempo de muestreo
for k = 1:Nsim
    
    if k >= 3/Ts
        r_sp(k) = 0;
    end
    if k >= 10/Ts
        r_sp(k) = 10;
    end
    % Microcontrolador %
    e(k) = r_sp(k) - y_feedback;

    % Con integrador
    u(k) = -ki*q(k) - K_new*x_hat(:, k);

    
    % Predicción del estado y de la covarianza
    x_hat(:, k) = SS_disc.A * x_hat(:, k) + SS_disc.B * lastu;
    lastu = u(k);
    P_kalman_pred = SS_disc.A * P_kalman * SS_disc.A' + Q_kalman;
    
    % Corrección (actualización) con la medición
    y_feedback = SS_disc.C * x(:, k);
    y_hat = SS_disc.C * x_hat(:, k);

    K_kalman = P_kalman_pred * SS_disc.C' / (SS_disc.C * P_kalman_pred * SS_disc.C' + R_kalman);
    x_hat(:, k+1) = x_hat(:, k) + K_kalman * (y_feedback - y_hat);
    P_kalman = (eye(nx) - K_kalman * SS_disc.C) * P_kalman_pred;
    
    uOut = [uOut u(k)];
    
    u = u(k)*ones(1, numel(dt)); % ZOH de la U - DAC

    % Sistema en la vida real %
    [y, tsim, XssOut] = lsim(sys_ss, u, dt, x(:, k)); % XssOut es la salida del Espacio de Estados. Cambiar
    x(:, k+1) = XssOut(end, :)'; % x es el vector de estados
    q(k+1) = q(k) + e(k); % Integrador se obtiene con el modelo discretizado
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
subplot(2,1,1)
    hold on
    for i = 1:nx
        plot(t, x(1, 1:Nsim+1));
        plot(t, x_hat(1, 1:Nsim+1));
    end
    legend('x_{(1)}','x_{hat}(1)');
    grid on
    title('Estado x1 real y estimado');
    xlabel('Tiempo [s]');
    ylabel('Estados');
subplot(2,1,2)
    hold on
    for i = 1:nx
        plot(t, x(2, 1:Nsim+1));
        plot(t, x_hat(2, 1:Nsim+1));
    end
    legend('x_{(2)}','x_{hat}(2)');
    grid on
    title('Estado x2 real y estimado');
    xlabel('Tiempo [s]');
    ylabel('Estados');

%% Gráficos de la estimación de los estados junto
figure(4)
hold on
for i = 1:nx
    plot(t, x_hat(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)');
for i = 1:nx
    plot(t, x(i, 1:Nsim+1));
end
legend('x_{hat}(1)','x_{hat}(2)','x(1)','x(2)');
grid on
title('Estados reales x y estimados x_{hat}');
xlabel('Tiempo [s]');
ylabel('Estados estimados');

%% Estados reales por separado
figure(5)
hold on
subplot(2,1,1)
    hold on
    plot(t, x(1, 1:Nsim+1));
    legend('x(1)');
    grid on
    %title('Estados respect');
    xlabel('Tiempo (s)');
    ylabel('Tensión de salida (V)');
subplot(2,1,2)
    hold on
    plot(t, x(2, 1:Nsim+1), 'r');
    legend('x(2)');
    grid on
    %title('Estados reales x');
    xlabel('Tiempo (s)');
    ylabel('Estado x2');