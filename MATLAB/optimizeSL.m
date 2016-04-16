clc, clear

data = xlsread('Team23_ProjectileData3');
thetaL = data([1:26],3);
thetaS = data([1:26],1);
plot(thetaS,thetaL,'*g');
hold on

funHandy = @(x) SumOfSquaredErrors(x(1)*sin( x(2)*thetaS + x(3)) + x(4),thetaL);
x = fminsearch(funHandy,  [35.295221154570015   0.014270109956176  -0.781326676841765  49.924635161389332], []);
A = x(1)
B = x(2)
C = x(3)
D = x(4)

figure(3)
plot(thetaS,thetaL,'*g');
hold on
thetaL = A*sin( B*thetaS + C) + D;
plot(thetaS,thetaL,'-k');
text(20,60,sprintf('SSE = %.4f',funHandy(x)));
hold off

% A = 34.937863474950177  ;
% B = 0.015053057998110   ; %pi/2(158.383-x)
% C = -0.813352158119725  ; %-x*b
% D = 49.89238602         ;
%inflection point
x = 55.439655033345026;
y = D;
load 'optVec';
thetaS = [25:135];
sinFunc = @(thetaS)  A*sin( B*thetaS + C) + D;

xbounds = [20 140];
ybounds = [30 90];
figure(1)
subplot(2,1,1);
plot(thetaS, sinFunc(thetaS))
axis([xbounds,ybounds]);
title('Initital function');
xlabel('Launch Angle [deg]');
ylabel('Servo Angle [deg]');
grid
hold on
plot(x,y,'*');

%------------------------------------------
a = A/1000;
b = B*1000;
d = D/1000;

phiS = (thetaS-x)/1000; 

sinFuncMod = @(phiS) a*sin(b*phiS) + d;

subplot(2,1,2);
plot(phiS, sinFuncMod(phiS))
axis([(xbounds-x)/1000, ybounds/1000]);
title('Tranlated and Scaled Function')
ylabel('Launch Angle Mod [deg]');
xlabel('Servo Angle Mod [deg]');
grid
hold off
plot(0,y/1000,'*');

thetaS = @(thetaL) 1000/b*asin((thetaL/1000-d)/a)+x;
figure(2)
fplot(thetaS,[0,135],'-b')
hold on
thetaL = data([1:26],3);
thetaS = data([1:26],1);
plot(thetaL,thetaS,'-k');