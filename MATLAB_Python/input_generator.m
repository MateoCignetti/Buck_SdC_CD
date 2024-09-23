clear all;
close all;

Ts = 0.01;  % Tiempo de muestreo
s = 3;      % Duración total de la señal
set = [0 20 40 20 60 80 100 40 80 20 10 30 90 80 0];  % Valores deseados
n = length(set);  % Longitud del vector set

u_single = zeros(1, s/Ts);  % Inicializa el vector u para una repetición
segment_length = floor((s/Ts) / n);  % Longitud de cada segmento

% Asignar los valores de 'set' a 'u_single' equitativamente
for i = 1:n
    start_idx = (i-1) * segment_length + 1;
    end_idx = i * segment_length;
    u_single(start_idx:end_idx) = set(i);
end

% Si sobra espacio debido a redondeo, llenar con los valores restantes
if end_idx < length(u_single)
    u_single(end_idx+1:end) = set(end);
end

% Repetir la señal completa 10 veces
num_repeats = 5;
u = repmat(u_single, 1, num_repeats);

% Gráfica de la señal resultante
time = 0:Ts:(length(u)-1)*Ts;
plot(time, u);
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Señal u repetida 10 veces');
grid on;

writematrix(u                                   , 'u_generated.csv');
