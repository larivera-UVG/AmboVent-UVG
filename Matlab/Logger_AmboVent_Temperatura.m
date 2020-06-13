% Logger de Temperatura para el AmboVent UVG + HUMANA
% Luis Alberto Rivera

%% Inicializaciones
clc;
pause(0.1);

DT = 20;  % Tiempo entre medición, en segundos
T_total = 5/60;  % Tiempo, en horas
N = ceil(T_total*60*60/DT);   % número de muestras

Temperatura = zeros(N,1);
tiempo = zeros(N,1);

n = 1;
while(true)
    nombre_archivo = sprintf('AV_Log_Temp%s_%02d.mat', date, n);
    if isfile(nombre_archivo)
        n = n + 1;
    else
        break;
    end
end

% Inicializar gráficas y handlers
% Tiempos medidos ------------------------------------------------------------------------
figure(1); clf;
h11 = animatedline('Color','r');
xlabel('contador');
ylabel('T (C)');
xlim([0, N]);
%legend('Temperat', 'Medido');
title('Temperatura del motor');
grid on;

pause(0.1);

% Crear objeto serial y abrir el puerto
% instrreset; % sólo si se tiene el Instrument Control Toolbox
delete(instrfind);  % Para evitar problemas al abrir y cerrar.
sObj = serial('COM20','BaudRate',115200);  % REVISAR EL PUERTO
sObj.Timeout = 30;
fopen(sObj);

pause(5);  % Para dar tiempo que el Arduino se reinicie.
fprintf('PRESIONE START...\n\n');

%% Ciclo de recepción de datos
timer_tS = tic;
timer_AV = tic;
n_prev = 2;

for n = 1:N
    Temperatura(n) = fscanf(sObj, '%d');
    
    tiempo(n) = toc(timer_AV);
    timer_AV = tic;

    % Guardar cada minuto, por cualquier cosa
    if(mod(n, 60/DT) == 0)
        save(nombre_archivo, 'Temperatura', 'tiempo', 'N');
    end

    fprintf('Iteración: %d/%d\n', n, N);
    
    addpoints(h11, n, Temperatura(n,1));
    
    if(toc(timer_tS) > 15)  % Esperar a que pase el intervalo mínimo de ThingSpeak
        datos_tS = mean(Temperatura((n_prev+1):n));
        thingSpeakWrite(1070900,datos_tS,'Fields',1,'WriteKey','4NCXKCP5CQGVSJJJ',...
            'Timeout',60);
        timer_tS = tic;
        n_prev = n;
    end
    
    drawnow limitrate
    
end

