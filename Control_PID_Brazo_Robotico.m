%Control PID Brazo Robotico%
%Parametros del Sistema%

b = 0.01;%Fricción viscosa
lc = 0.5;%distancia al centro de masa
l = 1; %longitud del brazo 
g = 9.81; %Acc de la gravedad
m = 1; %masa
I = (1/3)*m*l^2; %momento de inercia de la barra
%ganancia del PID 
kp = 100;
ti = 0.7;
td = 0.07;
alfa1= (b*lc^2)/(m*lc^2+I);
alfa0 = m*g*lc/(m*lc^2+I);
gama = 1/(m*lc^2+I);

b2 = gama*kp*ti*td;
b1 = gama*kp*ti; 
b0= gama*kp;

a3= ti;
a2 = ti*alfa1+gama*kp*ti*td;
a1 = ti*alfa0+gama*kp*ti;
a0 = gama*kp; 

%Numerador Función de Transferencia
K = pi/4; 
Numerador = K*[b2,b1,b0];
%Denominador de la Función de Transferencia
Denominador = [a3,a2,a1,a0];
str='Función de Transferencia';
disp(str);
T= tf(Numerador, Denominador);
%Respuesta del Brazo Robótico
figure;
step(T,'r',5)
title('Respuesta del brazo robotico')
xlabel('Tiempo')
ylabel('Theta')
grid on
hold on

