% Marciano Preciado
clc,clear

data = xlsread('Team23_ProjectileData3');
thetaS = data((1:37),1);
launchAngle = data((1:37),2);
distanceLOW = data((1:37),3)/100;
distanceHIGH = data((1:37),4)/100;

xLOW = launchAngle(10:23);
x = xLOW;
polyNomHandyLOW = @(cLOW) SumOfSquaredErrors(cLOW(1)*(x.^3-cLOW(4)) +cLOW(2)*(x.^2-cLOW(4)) +cLOW(3)*(x-cLOW(4)) + cLOW(5),distanceLOW(10:23));
cLOW = fminsearch(polyNomHandyLOW,  [-0.0000    0.0000    0.0002 -778.9374    1.2576], []);

xHIGH = launchAngle(7:19);
x = xHIGH;
polyNomHandyHIGH = @(cHIGH) SumOfSquaredErrors(cHIGH(1)*(x.^3-cHIGH(4)) +cHIGH(2)*(x.^2-cHIGH(4)) +cHIGH(3)*(x-cHIGH(4)) + cHIGH(5),distanceHIGH(7:19));
cHIGH = fminsearch(polyNomHandyHIGH, [-0.0000    0.0002    0.0065 -582.1094   -2.6723], []);

plot(launchAngle(7:19),distanceHIGH(7:19),'*r')
hold on
plot(launchAngle(10:23),distanceLOW(10:23),'*b')
hold on

x = xHIGH;
SSEHIGH=SumOfSquaredErrors(distanceHIGH(7:19), cHIGH(1)*(x.^3-cHIGH(4)) +cHIGH(2)*(x.^2-cHIGH(4)) +cHIGH(3)*(x-cHIGH(4)) + cHIGH(5));
x = xLOW;
SSELOW=SumOfSquaredErrors(distanceLOW(10:23), cLOW(1)*(x.^3-cLOW(4)) +cLOW(2)*(x.^2-cLOW(4)) +cLOW(3)*(x-cLOW(4)) + cLOW(5));

text(22, 0.5, sprintf('SSELOW %.5f\nSSEHIGH %.5f', SSELOW, SSEHIGH));

x=[0:1:90];
plot(x,cLOW(1)*(x.^3-cLOW(4)) +cLOW(2)*(x.^2-cLOW(4)) +cLOW(3)*(x-cLOW(4)) + cLOW(5),'-b');
plot(x,cHIGH(1)*(x.^3-cHIGH(4)) +cHIGH(2)*(x.^2-cHIGH(4)) +cHIGH(3)*(x-cHIGH(4)) + cHIGH(5),'-r');

legend('ExperimentalDataLOW','ExperimentalDataHIGH','LOWvals','HIGHvals');

disp('cLOW');
for n = 1:5
   fprintf('double c%d = %.16f;\n',n, cLOW(n)); 
end
disp('cHIGH');
for n = 1:5
   fprintf('double c%d = %.16f;\n',n, cHIGH(n)); 
end


