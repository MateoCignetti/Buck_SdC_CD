% Punto en el plano s
s = -500+500j;  % Por ejemplo

% Tiempo de muestreo
T = 200e-6;

% Aplicar la transformación bilineal
z = (1 + s * (T/2)) / (1 - s * (T/2));

% Mostrar el punto en el plano z
disp('Punto en el plano z:');
disp(z);