clear all; clc; close all
%% Question 1.1
% Create equations
syms vu theta thetaw vw
eq1 = 0 == vu*cos(theta) + cos(thetaw)*vw;
eq2 = 0 == vu*sin(theta) + sin(thetaw)*vw;
% Find solutions
sol = solve([eq1 eq2], [theta vu]);
% Test
b1 = isAlways(subs(eq1,[theta, vu], [sol.theta(1), sol.vu(1)]));
b2 = isAlways(subs(eq1,[theta, vu], [sol.theta(2), sol.vu(2)]));
b3 = isAlways(subs(eq2,[theta, vu], [sol.theta(1), sol.vu(1)]));
b4 = isAlways(subs(eq2,[theta, vu], [sol.theta(2), sol.vu(2)]));
if (b1 && b2 && b3 && b4)
    disp('VALID SOLUTIONS:')
    latex(sol.theta(1))
    latex(sol.vu(1))
    latex(sol.theta(2))
    latex(sol.vu(2))
else
    disp('INVALID SOLUTIONS!')
end

%% Question 1.2
vw0 = 15;
thetaw0 = -60*pi/180;
theta0 = subs(sol.theta(2),thetaw0);


