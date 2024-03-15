close all; clear all; clc;
log = load('device-monitor-240301-150652.log')
x = log(:,1);
y = log(:,2);
z = log(:,3);
% figure;
% scatter3(x,y,z)

[A,b] = magcal([x,y,z]);

D = [x(:),y(:),z(:)];

[A,b,expmfs] = magcal(D); % Calibration coefficients
expmfs % Display the expected magnetic field strength in uT

expmfs = 31.0723

C = (D-b)*A; % Calibrated data
figure(1)
plot3(x(:),y(:),z(:),"LineStyle","none","Marker","X","MarkerSize",8)
hold on
grid(gca,"on")
plot3(C(:,1),C(:,2),C(:,3),"LineStyle","none","Marker","o", ...
      "MarkerSize",8,"MarkerFaceColor","r")
axis equal
xlabel("uT")
ylabel("uT")
zlabel("uT")
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off

syms mx my mz
mout = ([mx, my, mz]-b)*A

Ts = 1/100;
F_kf = [1 -Ts;0 1];
G_kf = [Ts;0];
C_kf = [1 0];
R1 = [0.0005 0;0 0.05];
R2 = 0.00005;
Bv_kf = eye(2);
Gv_kf = eye(2);
V1 = Bv_kf*R1*Bv_kf'*Ts;
V2 = R2/Ts;
[L_kf,P,Q,lbd] = dlqe(F_kf,Gv_kf,C_kf,V1,V2)
syms gz m;
syms y;
syms xhat0;
syms xhat1;
xhat = [xhat0;xhat1];
xhat_next = G_kf*gz+F_kf*L_kf*(m-C_kf*xhat)+F_kf*xhat
path = '../lib/my_ekf/my_ekf';
if  (1)
    fp = fopen(strcat(path,'.cpp'), "w");
    fprintf(fp,'#include <math.h>\n#include "my_ekf.h"\n')
    fprintf(fp,'float kalman(float mx, float my, float mz, float gz){\n');
    fprintf(fp,'  float mx_adj = %s;\n',vpa(mout(1)));
    fprintf(fp,'  float my_adj = %s;\n',vpa(mout(2)));
    fprintf(fp,'  float mz_adj = %s;\n',vpa(mout(3)));
    fprintf(fp,'  float m = atan2(mx_adj, my_adj);\n');
    fprintf(fp,'  static float xhat0 = m;\n');
    fprintf(fp,'  static float xhat1 = 0;\n');
    fprintf(fp,'  float y = %s; // yhat(k) = xhat(k)\n', vpa(C_kf*[xhat]));
    fprintf(fp,'  xhat0 = %s; // xhat(k+1) = G*gz+F*L*(m-C*xhat(k))+F*xhat(k)\n',vpa(xhat_next(1)));
    fprintf(fp,'  xhat1 = %s; // xhat(k+1) = G*gz+F*L*(m-C*xhat(k))+F*xhat(k)\n',vpa(xhat_next(2)));
    fprintf(fp,'  return y;\n');
    fprintf(fp,'}');
    fclose(fp);
    fp = fopen(strcat(path,'.h'), "w");
    fprintf(fp,'float kalman(float mx, float my, float mz, float gz);\n');
    fclose(fp);
end
%%
head = load('device-monitor-240301-151245.log');
t = 0:0.01:(length(head)-1)*0.01;
figure;
plot(t,head,'r');
grid on; hold on;
head = load('device-monitor-240301-153341.log');
t = 0:0.01:(length(head)-1)*0.01;
plot(t,head,'b');
head = load('device-monitor-240301-153647.log');
t = 0:0.01:(length(head)-1)*0.01;
plot(t,head,'g');
head = load('device-monitor-240301-154425.log');
t = 0:0.01:(length(head)-1)*0.01;
% plot(t,head,'k');