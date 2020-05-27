% Logger para el AmboVent UVG + HUMANA
% Luis Alberto Rivera

%% Inicializaciones
clc;
pause(0.1);

BPM = 20;      % Ciclos por minuto. Hacer coincidir con el programa de Arduino.

% CAMBIAR: QUE NO SE NECESITE FIJAR EL COMP_PER. QUE EL PROGRAMA DEL ARDUINO MANDE LOS
% MÁXIMOS Y MÍNIMOS TEÓRICOS DEL WANTED_POS, SEGÚN SEAN EL COMPRESSION_PER.
Comp_per = 80; % Porcentaje de compresión. Hacer coincidir con el programa de Arduino.
T_total = 5/60;  % Tiempo, en horas
N = ceil(T_total*60*BPM);   % número de muestras

M = 7;  % número de datos por ciclo. Debe coincidir con el programa del Arduino
datos = zeros(N,M);
tiempo = zeros(N,1);

n = 1;
while(true)
    nombre_archivo = sprintf('AV_Log_%s_%02d.mat', date, n);
    if isfile(nombre_archivo)
        n = n + 1;
    else
        break;
    end
end

% Inicializar gráficas y handlers
% Tiempos medidos ------------------------------------------------------------------------
figure(1); clf;
h11 = animatedline('Color','b');
% h11 = plot(2:N, (60/BPM)*ones(1,N-1), 'b');
hold on;
h12 = animatedline('Color','r');
xlabel('número de ciclo');
ylabel('tiempo (seg)');
xlim([0, N]);
% ylim([60/BPM-1,60/BPM+1]);
ylim([1, 11]);
legend('Teórico', 'Medido');
% title(sprintf('Tiempos entre mediciones. Teórico: %0.2f', 60/BPM));
title('Tiempos entre mediciones');
grid on;

% Posiciones calculadas y medidas del potenciómetro de feedback --------------------------
figure(2); clf;
h21 = animatedline('Color','r');
hold on;
h22 = animatedline('Color','m');
h23 = animatedline('Color','b');
h24 = animatedline('Color','c');
xlabel('número de ciclo');
ylabel('posiciones');
xlim([0, N]);
title(sprintf('wanted-pos y A-pot. Comp-per: %d', Comp_per));
legend('max-wanted-pos', 'max-A-pot', 'min-wanted-pos', 'min-A-pot', 'Location', 'east');
grid on;

% Presiones máximas y mínimas medidas ----------------------------------------------------
figure(3); clf;
h31 = animatedline('Color','r');
hold on;
h32 = animatedline('Color','b');
xlabel('número de ciclo');
ylabel('mbar');
ylim([-1, 30]);
xlim([0, N]);
title('Máximos y Mínimos de Presión');
legend('Máximos', 'Mínimos');
grid on;

pause(0.1);

% Crear objeto serial y abrir el puerto
% instrreset; % sólo si se tiene el Instrument Control Toolbox
delete(instrfind);  % Para evitar problemas al abrir y cerrar.
sObj = serial('COM28','BaudRate',115200);  % REVISAR EL PUERTO
sObj.Timeout = 30;
fopen(sObj);

fprintf('PRESIONE START...\n\n');
pause(0.1);

%% Ciclo de recepción de datos
for n = 1:N
    tic
    for m = 1:M
        datos(n,m) = fscanf(sObj, '%d');
    end
    tiempo(n) = toc;
    
    % Guardar cada minuto, por cualquier cosa
    if(mod(n, BPM) == 0)
        save(nombre_archivo, 'datos', 'tiempo', 'BPM', 'T_total', 'N', 'M', 'Comp_per');
    end

    fprintf('Iteración: %d/%d\n', n, N);
    
    if(n > 2)
        addpoints(h11, n, 60/datos(n,7));
        addpoints(h12, n, tiempo(n));
        
        addpoints(h21, n, datos(n,2));
        addpoints(h22, n, datos(n,4));
        addpoints(h23, n, datos(n,1));
        addpoints(h24, n, datos(n,3));
        
        addpoints(h31, n, datos(n,6));
        addpoints(h32, n, datos(n,5));
        
        drawnow limitrate
    end
end

figure(3);
ylim([(min([0; datos(2:N,5)])-1), (max([9; datos(2:N,6)])+1)]);

save(nombre_archivo, 'datos', 'tiempo', 'BPM', 'T_total', 'N', 'M', 'Comp_per');

fclose(sObj);

%% Gráficas
% figure(1); clf;
% plot(2:N, (60/BPM)*ones(1,N-1), 'b');
% hold on;
% plot(2:N, tiempo(2:N), 'r');
% xlabel('número de ciclo');
% ylabel('tiempo (seg)');
% ylim([60/BPM-1,60/BPM+1]);
% legend('Teórico', 'Medido');
% title(sprintf('Tiempos entre mediciones. Teórico: %0.2f', 60/BPM));
% grid on;

% figure(2); clf;
% plot(2:N, datos(2:N,2), 'r');
% hold on;
% plot(2:N, datos(2:N,4), 'm');
% plot(2:N, datos(2:N,1), 'b');
% plot(2:N, datos(2:N,3), 'c');
% xlabel('número de ciclo');
% ylabel('posiciones');
% title(sprintf('wanted-pos y A-pot. Comp-per: %d', Comp_per));
% legend('max-wanted-pos', 'max-A-pot', 'min-wanted-pos', 'min-A-pot', 'Location', 'east');
% grid on;

% figure(3); clf;
% plot(2:N, datos(2:N,6), 'r');
% hold on;
% plot(2:N, datos(2:N,5), 'b');
% xlabel('número de ciclo');
% ylabel('mbar');
% ylim([(min([0; datos(2:N,5)])-1), (max([9; datos(2:N,6)])+1)]);
% title('Máximos y Mínimos de Presión');
% legend('Máximos', 'Mínimos');
% grid on;

% figure(3);
% ylim([(min([0; datos(2:N,5)])-1), (max([9; datos(2:N,6)])+1)]);
