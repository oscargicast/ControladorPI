clear all; close all; clc
%% Programa para calcular el controlador
% -------------------------------------------------------------------------
% Cargando la DATA
% -------------------------------------------------------------------------
dataLeida = load('../data/data_rc.lvm');
T=1/30; 
y1=dataLeida(:,4);
u1=dataLeida(:,6);
figure
subplot(211)
plot(y1,'m')
subplot(212)
plot(u1,'m')
% -------------------------------------------------------------------------
% Identificación ARX
% -------------------------------------------------------------------------
data=iddata(y1,u1,T);
th=arx(data,[1 1 1]);
present(th)
thc=d2c(th);
[num,den]=tfdata(thc);
Gp=tf(num,den);
% -------------------------------------------------------------------------
% Polos de la planta 
% -------------------------------------------------------------------------
Pp=pole(Gp);
ip=abs(imag(Pp));
rp=abs(real(Pp));
% -------------------------------------------------------------------------
% Especificaciones de diseño polos deseados
% -------------------------------------------------------------------------
ts=1;
Mp=0.1;
zeta=-log(Mp)/sqrt((log(Mp))^2+pi^2);
wn=4.6/(zeta*ts);
s1=-zeta*wn+1j*wn*sqrt(1-zeta^2);
sd=s1;
id=abs(imag(sd));
rd=abs(real(sd));
% -------------------------------------------------------------------------
% Diseño del control PID continuo
% -------------------------------------------------------------------------
qp1=pi-atan(id/rd); 
qp2=atan((ip-id)/(rp-rd)); 
qc=-pi+(qp1+qp2);   % condición de fase
qb=qc/100;          % consideración de diseño para qb
qa=qc/1000;         % consideración de diseño para qa
if abs(qa)>pi/2
    a=rd+id/tan(qa);
else
    a=rd-id/tan(qa);
end
if abs(qb)>pi/2
    b=rd+id/tan(qb);
else
    b=rd-id/tan(qb);
end
% b = 0;
% -------------------------------------------------------------------------
% Simulación del Controlador PID continuo
% -------------------------------------------------------------------------
Gc=tf(conv([1 a],[1 b]),[1 0]);
FLA=series(Gc,Gp);
K=rlocfind(FLA,sd);
Kp=K*(a+b);
Ki=K*(a*b);
Kd=K;
Ti=Kp/Ki;
Td=Kd/Kp;
Td = 0;

Gc=tf([Kp*Ti*Td Kp*Ti Kp],[Ti 0]);
L=series(Gc,Gp);
H=L/(L+1);
figure
subplot(211)
t=0:0.001:5;
u=ones(size(t));
yp=lsim(H,u,t);
plot(t,u,'r')
hold
plot(t,yp,'LineWidth',2)
xlabel('\bf t(seg)')
ylabel('\bf y(t)')
% -------------------------------------------------------------------------
% Re-diseño por tustin del Control en Tiempo Discreto
% -------------------------------------------------------------------------
tau=(1/den{1}(2));
T=tau/5;
[Nt,Dt] = tfdata(Gc,'v');
Nt = poly2sym(Nt,'s');
Dt = poly2sym(Dt,'s');
syms z
Gdt = Nt/Dt;
Gdt = subs(Gdt,{'s'},(2*(z-1))/(T*(z+1)));
Gdt = simplify(Gdt);
Gdt = vpa(Gdt,4); 
[NDt DDt] = numden(Gdt);
NDt = sym2poly(NDt);
DDt = sym2poly(DDt);
% -------------------------------------------------------------------------
% FT del Controlador digital D(z)
% -------------------------------------------------------------------------
GDt = tf(NDt,DDt,T);
printsys(NDt,DDt,'z')
[Np,Dp]=tfdata(Gp,'v');
datos=[Np Dp NDt DDt];
% -------------------------------------------------------------------------
% Coeficientes para lectura de LabVIEW
% -------------------------------------------------------------------------
save coef_rc.lvm datos -ascii -tabs
