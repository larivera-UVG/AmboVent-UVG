% Logger para el AmboVent UVG + HUMANA
% Luis Alberto Rivera

load('Resultados\AV_Log_04-Aug-2020_02.mat');

Nfinal = N;

% Tiempos medidos ------------------------------------------------------------------------
figure(1); clf;
plot(2:Nfinal, 60./datos(2:Nfinal,7), 'b');
hold on;
plot(2:Nfinal, tiempo(2:Nfinal), 'r');
xlabel('n�mero de ciclo');
ylabel('tiempo (seg)');
xlim([2, Nfinal]);
ylim([1, 12]);
legend('Te�rico', 'Medido');
title('Duraci�n de los Ciclos');
grid on;

% Posiciones calculadas y medidas del potenci�metro de feedback --------------------------
figure(2); clf;
plot(2:Nfinal, datos(2:Nfinal,2), 'r');
hold on;
plot(2:Nfinal, datos(2:Nfinal,4), 'm');
plot(2:Nfinal, datos(2:Nfinal,1), 'b');
plot(2:Nfinal, datos(2:Nfinal,3), 'c');
xlabel('n�mero de ciclo');
ylabel('Valor ADC');
xlim([2, Nfinal]);
legend('m�x deseada', 'm�x medida', 'm�n deseada', 'm�n medida', 'Location', 'east');
title('Posiciones Extremas del Brazo');
grid on;

% Presiones m�ximas y m�nimas medidas ----------------------------------------------------
figure(3); clf;
plot(2:Nfinal, datos(2:Nfinal,6), 'r');
hold on;
plot(2:Nfinal, datos(2:Nfinal,5), 'b');
xlabel('n�mero de ciclo');
ylabel('mbar');
ylim([(min([0; datos(2:Nfinal,5)])-1), (max([9; datos(2:Nfinal,6)])+1)]);
xlim([2, Nfinal]);
legend('M�ximos', 'M�nimos');
title('M�ximos y M�nimos de Presi�n');
grid on;
