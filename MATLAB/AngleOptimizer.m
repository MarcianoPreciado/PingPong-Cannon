% Marciano Preciado
clc,clear

data = xlsread('Team23_ProjectileData4');
thetaS = data((1:37),1);
launchAngle = data((1:37),3);
distanceLOW = data((1:37),4)/100;
distanceHIGH = data((1:37),5)/100;

xLOW = launchAngle(10:23);
x = xLOW;
polyNomHandyLOW = @(cLOW) SumOfSquaredErrors(cLOW(1)*(x.^3-cLOW(4)) +cLOW(2)*(x.^2-cLOW(4)) +cLOW(3)*(x-cLOW(4)) + cLOW(5),distanceLOW(10:23));
cLOW = fminsearch(polyNomHandyLOW,  1e+02*[-0.000000024612570   0.000000265222832  -0.000001118744286  -2.316058981845771   0.014288004727492], []);

xHIGH = launchAngle(11:19);
x = xHIGH;
polyNomHandyHIGH = @(cHIGH) SumOfSquaredErrors(cHIGH(1)*(x.^3-cHIGH(4)) +cHIGH(2)*(x.^2-cHIGH(4)) +cHIGH(3)*(x-cHIGH(4)) + cHIGH(5),distanceHIGH(11:19));
cHIGH = fminsearch(polyNomHandyHIGH, [-0.000005825129850   0.000279436039160   0.001091236775705  11.438866919503834   1.308920238310206], []);
figure(1)
axis([25,85,0,1.5]);
plot(launchAngle(11:19),distanceHIGH(11:19),'*r')
hold on
plot(launchAngle(10:23),distanceLOW(10:23),'*b')
hold on

x = xHIGH;
SSEHIGH=SumOfSquaredErrors(distanceHIGH(11:19), cHIGH(1)*(x.^3-cHIGH(4)) +cHIGH(2)*(x.^2-cHIGH(4)) +cHIGH(3)*(x-cHIGH(4)) + cHIGH(5));
x = xLOW;
SSELOW=SumOfSquaredErrors(distanceLOW(10:23), cLOW(1)*(x.^3-cLOW(4)) +cLOW(2)*(x.^2-cLOW(4)) +cLOW(3)*(x-cLOW(4)) + cLOW(5));

text(22, 0.5, sprintf('SSELOW %.5f\nSSEHIGH %.5f', SSELOW, SSEHIGH));

x=[0:1:90];
plot(x,cLOW(1)*(x.^3-cLOW(4)) +cLOW(2)*(x.^2-cLOW(4)) +cLOW(3)*(x-cLOW(4)) + cLOW(5),'-b');
plot(x,cHIGH(1)*(x.^3-cHIGH(4)) +cHIGH(2)*(x.^2-cHIGH(4)) +cHIGH(3)*(x-cHIGH(4)) + cHIGH(5),'-r');

xlabel('Launch Angle [deg]');
ylabel('Landing distance [m]');
axis([20,85,0,1.50]);
legend('ExperimentalDataHIGH','ExperimentalDataLOW','LOWvals','HIGHvals','Location','southwest');
grid('on');
disp('cLOW');
for n = 1:5
   fprintf('double c%d = %.16f;\n',n, cLOW(n)); 
end
disp('cHIGH');
for n = 1:5
   fprintf('double c%d = %.16f;\n',n, cHIGH(n)); 
end

figure
plot(thetaS(10:23),distanceLOW(10:23),'*b');
hold on
plot(thetaS(11:19), distanceHIGH(11:19),'*r');


