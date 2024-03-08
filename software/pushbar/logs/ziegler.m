log = load('osc2.log');
T_s = 10e-3;
y = log(:,2)*1/(2000);
u = log(:,4);
u(u>11.3) = 12;
t = 0:T_s:(length(y)-1)*T_s;
figure;
plot(t,y)
syms z
tau = 0.85*0.11;
I = (tau*(2/T_s*(z-1)/(z+1)) +1)/(tau*2/T_s*(z-1)/(z+1));
I = simplifyFraction(vpa(I))
Kp = 0.45*0.5;
C = Kp*I
num = [0.0012032085561497326203208556149733*197, -0.0012032085561497326203208556149733*177];
den = [1 -1]
syms x y yp xp
eq = vpa(y == 1/den(2)*(num(2)*x+num(1)*xp-den(1)*yp))
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


syms s tau Kp z Ts
G = Kp*((tau*s+1)/(tau*s));
H = subs(G,s,2/Ts*(z-1)/(z+1))
H = simplifyFraction(vpa(H))
Gf = 1/(tau*s+1);
Hf = subs(Gf,s,2/Ts*(z-1)/(z+1));
Hf = simplifyFraction(vpa(Hf))
%%

Ts = 10e-3;
tau = 0.1;
Kp = 0.1;
b0 = Ts/tau-1;
b1 = Ts/tau+1;
a1 = 1;
a0 = -1;
z = tf('z',Ts);
Gd = Kp*(b1+b0*z^(-1))/(a1+a0*z^(-1));
t = 0:Ts:1;
[ys,ts] = step(Gd,t);
e = 1;
x = e;
xp = 0;
y = 0;
yp = 0;
Y = zeros(1,length(t));
for i = 1:length(t)
    x = e;
    y = 1/a1*(Kp*(b1*x+b0*xp)-a0*yp);
    xp = x;
    yp = y;
    Y(i) = y;
end
figure;
plot(ts,ys);
hold on;
plot(t,Y,'--k');
legend('step','sim')