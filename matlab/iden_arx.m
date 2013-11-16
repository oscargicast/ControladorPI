clear all; close all; clc;
line_widht = 2;
%% Toolbox de identificacion mediante comandos Matlab
%% DATA experimental proviene desde el circuito RC Serie
data = load('../data/data_rc.lvm');
y = data(:, 4);
u = data(:, 6);

figure
subplot(211)
plot(y,'b','LineWidth',2); 
xlabel('\bf t(seg)'); ylabel('\bf y(volts)');
subplot(212)
plot(u,'r','LineWidth',2); 
xlabel('\bf t(seg)'); ylabel('\bf u(volts)');

N = length(u);
disp('# de muestras'), disp(N)

tini = 0;
tfin = 10;
t = linspace(tini, tfin, N);

%% Toolbox de identificacion

%% 1st. Proceso de la data-objeto
Ts =  1/30; % De la adquisicion de labview (33.3333ms)
idata = iddata(y, u, Ts);

%% 2nd Estructura parametrica ARX(na, nb, nx=nc)
na = 1; nb = 1; nc = 1; 
th = arx(idata, [na, nb, nc])

%% 3rd discreta - funcion de transerencia D(z)
D = tf(th.b, th.a, Ts)

%% 4th Funcion de transferencia G(s)
Gs = d2c(D, 'zoh')

[n, d] = tfdata(D, 'v'); 

yc = lsim(Gs, u, t);

figure; hold on;
plot(t, y, 'r', 'LineWidth', line_widht);
plot(t, yc, 'b--', 'LineWidth', line_widht);
xlabel('\bf t(seg)'); ylabel('\bf y(volts)');
legend('y_{exp}', 'y_{iden}');





