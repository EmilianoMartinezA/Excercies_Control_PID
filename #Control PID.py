#Control PID
####################
import numpy as np
import matplotlib.pyplot as plt
import control
import math
#################
#Parametros del Control
kp = 10
Ti = 0.1
Td = 1 
##################
#Funcion de transferencia
Num_PID = np.array([kp*Td*Ti, kp*Ti, kp])
Den_PID=np.array([Ti, 0])
C = control.tf(Num_PID, Den_PID) #Control PID
print('C(s)=', C)
#####################
#Parametros Motor
Bm = 0.001 #Coefieicnet de friccion viscosa del motor
BL = 0.001 #Coeficiente d friccion de la carga
JL = 0.1 # Movimiento de Inercia de la carga
Jm = 0.0078 # Mov de inercia del motor
Km = 0.83 #Constante del motor
Kb = 0.83 #Constante de la fuerza electromotriz
La = 0.88*(1e-3) #Inductancia
Ra = 4.5 #Resistencia de armadura
n1 = 10 #Numero de dientes del engrane 1
n2 = 50 #Numero de dientes del engrane 2
n = (n1/n2)
Beq=Bm+pow(n,2)*BL #Friccion Equivalente
JEq = Jm+pow(n, 2)*JL #Momento de inercia Equivalente
####################################
alfa0 = Ra * Beq/Km
alfa1 = (La*Beq+Ra*JEq) /Km
alfa2 = La*JEq/Km
#############################################
#Parametros de la funcion de transferencia total
a0 = kp
a1  = Ti*alfa0+kp*Ti
a2 = Ti*alfa1+kp*Td*Ti
a3 = Ti*alfa2
b0 = kp
b1 = kp*Ti
b2 = kp*Td*Ti
#############################################
#Funcion de transferencia del motor
numerador_motor = np.array([1])
denominador_motor = np.array([alfa2, alfa1, alfa0])
G_p = control.tf(numerador_motor, denominador_motor)
print('Gp(s)=',G_p)
############################
#Mult de bloques G(s)=C(s)*Gp(s)
G = control.series(C,G_p)
print('G(s)=',G)
########################
#Funcion de transferencia de Retroalimentacion
Nume_H = np.array([1])
Den_H = np.array([0.1])
H = control.tf(Nume_H, Den_H)
########################
#Funcion de Transferencia Total
T = control.feedback(G, H)
print('T(s)', T)
#Polos de la funcion de transferencia
polos = control.pole(T)
print('Polos', polos)
zeros = control.zeros(T)
print('Zeros', zeros)
control.pzmap(T)
plt.grid()
plt.show()
######################33
t, y = control.step_response(T)
plt.plot(t,y)
plt.title('Velocidad Angular del Motor (rad/s)')
plt.grid()
plt.show()