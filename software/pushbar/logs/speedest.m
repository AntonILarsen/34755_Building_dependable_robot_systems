clc; close all;
log = load('speedlogbigmotor.log');
T_s = 0.01;
u = log(:,1)*1/255*11.3;
y = log(:,2)/(4*800)*1/T_s*2*pi;
t = 0:T_s:(length(y)-1)*T_s;
figure;
plot(t,y)
idx = find(u < 8,1,'last');
ta = t(idx-10:end);
ua = u(idx-10:end);
ya = y(idx-10:end);
ta = ta - ta(1);
idd = iddata(ya,ua,T_s); 
G = tfest(idd,1,0); 
[y_sim,t_sim] = lsim(G,u,t);
figure;
plot(t_sim,y_sim)
hold on;
plot(t,y)
%%
close all
s = tf('s');
tau_f = 0.001;
G_filt = 1/(tau_f*s+1);
G_ol = G*1/s*G_filt;
Ni = 3;
phi_m = 65;
phi_i = -atan(1/Ni)*180/pi;
phi_G = -180+phi_m-phi_i
wlo = 0;
whi = 300;
wdelta = 0.001;
Pm = 110;
[m,p,w] = bode(G_ol, wlo:wdelta:whi);
wc = interp1(p(:),w, phi_G)
% mp0 = interp1(w, m(:), wp0)
% K_P = 1/mp0
tau_i = Ni/wc;
C = (tau_i*s+1)/(tau_i*s);
Kp = abs(1/(evalfr(C,wc*1i)*evalfr(G_ol,wc*1i)));
G_ol = Kp*C*G*1/s*G_filt;
G_cl = G_ol/(1+G_ol);
figure;
margin(G_ol)
figure;
step(G_cl)
T_s = 10e-3;
C_disc = c2d(Kp*C,T_s,'tusin');
syms z
[num, den] = tfdata(C_disc,'v');
% sys_sym = poly2sym(num, z) / poly2sym(den, z);
% vpa(sys_sym)

H = c2d(G*1/s,T_s,'zoh');
tau = 1/wc
z = tf('z',T_s)
I_z = (tau*(2/T_s*(z-1)/(z+1)) +1)/(tau*2/T_s*(z-1)/(z+1));
I_z = minreal(I_z);
wlo = 0;
whi = 150;
wdelta = 0.00001;
[m,p,w] = bode(I_z*H,  wlo:wdelta:whi); %  wlo:wdelta:whi
bode(I_z*H)


p = p(:);
p = p(2:end); % something is wrong with the first sample. It is set to 0 degrees
m = m(:);
m = m(2:end);
w = w(2:end);
Pm = max(p) + 180
wp0 = interp1(p,w, Pm-180)
m_wp0 = interp1(w, m, wp0);
P = 1/m_wp0
disp('PI =  ')
[num, den] = tfdata(minreal(P*I_z),'v');


C_disc.Variable = 'z^-1'
% syms x y yp xp
% eq = vpa(y == 1/den(2)*(num(2)*x+num(1)*xp-den(1)*yp))
% path = '../lib/servo_ctrl/servo_ctrl';
% if  (1)
%     fp = fopen(strcat(path,'.cpp'), "w");
%     fprintf(fp,'#include "servo_ctrl.h"\n');
%     fprintf(fp,'float pictrl(float e){\n');
%     fprintf(fp,'  static float x = e, xp = e, y = 0, yp = 0;\n');
%     fprintf(fp,'  x = e;\n');
%     fprintf(fp,'  y = %s;\n',rhs(eq));
%     fprintf(fp,'  yp = y;\n');
%     fprintf(fp,'  xp = x;\n');
%     fprintf(fp,'  return y;\n');
%     fprintf(fp,'}');
%     fclose(fp);
%     fp = fopen(strcat(path,'.h'), "w");
%     fprintf(fp,'float pictrl(float e);\n');
%     fclose(fp);
% end