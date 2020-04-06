% Pruebas para el sistema AmboVent

%% Perfiles de posici�n y velocidad originales
pos = [  0,  0,  0,  0,  1,  1,  1,  2,  2,  3,  3,  4,  5,  5,  6,  7,  8,  9, 10, 11,...
        12, 14, 15, 16, 17, 19, 20, 22, 23, 25, 27, 28, 30, 32, 33, 35, 37, 39, 41, 43,...
        45, 47, 49, 51, 53, 55, 57, 59, 62, 64, 66, 68, 71, 73, 75, 78, 80, 82, 85, 87,...
        90, 92, 95, 97,100,102,105,107,110,112,115,117,120,122,125,128,130,133,135,138,...
       140,143,145,148,150,153,155,158,160,163,165,168,170,173,175,177,180,182,184,187,...
       189,191,193,196,198,200,202,204,206,208,210,212,214,216,218,220,222,223,225,227,...
       228,230,232,233,235,236,238,239,240,241,243,244,245,246,247,248,249,250,250,251,...
       252,252,253,253,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,...
       255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,253,...
       253,252,252,251,250,250,249,248,248,247,246,245,244,243,242,241,239,238,237,236,...
       234,233,232,230,229,227,225,224,222,220,219,217,215,213,211,209,207,205,203,201,...
       199,197,195,193,191,188,186,184,182,179,177,175,172,170,167,165,163,160,158,155,...
       153,150,148,145,143,140,138,135,133,130,128,125,123,120,118,115,113,110,108,105,...
       103,101, 98, 96, 94, 91, 89, 87, 85, 83, 80, 78, 76, 74, 72, 70, 68, 66, 65, 63,...
        61, 59, 58, 56, 54, 53, 51, 50, 48, 47, 46, 45, 43, 42, 41, 40, 39, 38, 37, 36,...
        35, 34, 33, 32, 31, 31, 30, 29, 28, 27, 26, 26, 25, 24, 23, 22, 22, 21, 20, 20,...
        19, 18, 18, 17, 16, 16, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10,  9,  9,...
         9,  8,  8,  7,  7,  7,  6,  6,  6,  5,  5,  5,  5,  4,  4,  4,  4,  3,  3,  3,...
         3,  3,  3,  2,  2,  2,  2,  2,  2,  2,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,...
         1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,...
         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0];

vel = [129,129,130,130,131,131,132,133,133,134,135,135,136,136,137,138,138,138,139,140,...
       140,141,141,141,142,142,143,143,144,144,145,145,145,146,146,146,147,147,147,148,...
       148,148,149,149,149,150,150,150,150,151,151,151,151,151,152,152,152,152,152,152,...
       153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,...
       153,153,153,153,153,153,153,153,153,153,152,152,152,152,152,152,151,151,151,151,...
       151,150,150,150,150,149,149,149,148,148,148,147,147,147,146,146,146,145,145,145,...
       144,144,143,143,142,142,141,141,141,140,140,139,138,138,138,137,136,136,135,135,...
       134,133,133,132,131,131,130,130,129,129,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,127,127,126,126,126,125,125,124,124,...
       123,123,122,122,121,121,120,120,120,119,118,118,118,117,117,116,116,115,115,115,...
       114,114,113,113,113,112,112,111,111,111,110,110,109,109,109,108,108,108,107,107,...
       107,107,106,106,106,105,105,105,105,105,104,104,104,104,104,104,103,103,103,103,...
       103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,104,104,104,104,...
       104,105,105,105,105,105,106,106,106,107,107,107,108,108,108,109,109,109,110,110,...
       111,111,112,112,113,113,114,114,114,115,116,116,116,117,117,118,118,118,118,118,...
       119,119,119,119,119,119,119,120,120,120,120,120,120,120,121,121,121,121,121,121,...
       121,122,122,122,122,122,122,122,123,123,123,123,123,123,123,124,124,124,124,124,...
       124,124,124,124,125,125,125,125,125,125,125,125,125,126,126,126,126,126,126,126,...
       126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,...
       127,127,127,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,...
       128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128];
   
figure(1); clf;
subplot(2,1,1);
plot(pos);
xlabel('Index');
ylabel('Promiles of full range');
title('Position');
grid on;
subplot(2,1,2);
plot(vel-128);
xlabel('Index');
ylabel('position change per 0.2 sec');
title('Velocity');
grid on;
sgtitle('Position and Velocity Profiles of AmboVent System');