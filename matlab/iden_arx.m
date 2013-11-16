clear all; close all; clc;
line_widht = 1;
%% Toolbox de identificacion mediante comandos Matlab
%% DATA experimental proviene desde el circuito RC Serie

% % RC
% data = load('data_rc.lvm');
% y = data(:, 2);
% u = data(:, 4);

% OPAMS
data = load('../data/data_rc.lvm');
y = data(:, 4);
u = data(:, 6);

subplot(211);
plot(y, 'r'), ylabel('Amplitud')

subplot(212);
plot(u, 'k'), ylabel('Amplitud'), xlabel('N(muestras)')

N = length(u);
disp('# de muestras'), disp(N)

tini = 0;
tfin = 10;
t = linspace(tini, tfin, N);

% tt = data(:,1);


% t = 0:N-1;
% u = ones(1, N);

%% Toolbox de identificacion

%% 1st. Porceso de la data-objeto - iddata DAT = iddata(Y,U,Ts)
% Tener cuidado de este valor, ya que debe ser la frecuencia de muestreo
% (fs) de la experiencia con la DAQ.
% Ts = 0.03; % 33.3333ms de labview
Ts =  1/30; 
% Ts = 1e-3; 
idata = iddata(y, u, Ts);

%% 2nd Estructura parametrica ARX(na, nb, nx=nc)
% na = 2; nb = 2; nc = 2; nk = 1;
na = 1; nb = 1; nc = 1; 

% th = arx(idata, [na, nb, nc]);
th = arx(idata, [na, nb, nc]);
% En versiones anteriores se tiena a 'q' en vez de 'z'
% B numerador, A denominador
% FPE(funcion de prediccion de error)
th

%% 3rd discreta - funcion de transerencia D(z)
D = tf(th.b, th.a, Ts)
De = tf(th.c, th.a, Ts)
% cmd: d2c

%% 4th Funcion de transferencia G(s)
Gs = d2c(D, 'zoh');
Ge = d2c(De, 'zoh');
disp('funcion de trans'), Gs

% % Son diferentes Gs y G: th2th es obsoleta
% [n, d] = th2tf(th); % de formato theta a tf
% % th2th es obsoleta, se debe usar TFDATA
% G = tf(n, d);
% disp('funcion de trans'), G

[n, d] = tfdata(D, 'v'); 
Gs = d2c(D, 'zoh');

% white noise
e = wgn(1,N,0);

yc = lsim(Gs, u, t);
ye = lsim(Ge, 0.001*e, t);
plot(t, yc+ye, 'b', 'LineWidth', line_widht);

legend('y_{exp}', 'y_{iden}', 4);



%% Indentificaision usando GUIDE
% ident




