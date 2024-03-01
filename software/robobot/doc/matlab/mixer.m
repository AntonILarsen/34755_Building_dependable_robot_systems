close all
clear
%% turn
Wb = 0.2;
%%
disp('-- tr, vlin, r, v0, v1, va --')
tr = -0.5;
Vlin = 0;
for (tr = -1:0.5:1)
    for (Vlin = -0.3:0.1:0.3)
       v0 = Wb/2*tr + Vlin;
       if abs(v0) < Vlin 
          v0 = Vlin;
       end
       r = abs(v0/tr);
       v1 = v0/r*(r - Wb);
       va = (v0+v1)/2;
       disp([tr, Vlin, r, v0, v1, va])
    end
end

%%
disp('------ post method ----')
disp('---- turnrate   lin-vel  vel-dif radius   turnrate2 turnrad3  left,    right,  vel-avg ----')
for (tr = -1:0.25:1)
    for (Vlin = -0.3:0.05:0.3)
       vd = Wb*tr;
       if (abs(vd) > abs(Vlin))
           % lin vel is too small, but not 0
           v0 = vd/2 + Vlin;
           v1 = v0 - vd;
       else
         if (vd * Vlin > 0)
           v0 = Vlin;
           v1 = v0 - vd;
         else
           v1 = Vlin;
           v0 = v1 + vd;
         end
       end
       va = (v0+v1)/2;
       if (abs(v1 - v0) > 1e-4)
         ra = (v0+v1)/(v0 - v1)*Wb/2;
       else
         ra = 100;
       end
       tr2 = va/ra;
       tr3 = (v0 - v1)/Wb;
       disp([tr, Vlin, vd, ra, tr2, tr3, v1, v0, va])
    end
end


