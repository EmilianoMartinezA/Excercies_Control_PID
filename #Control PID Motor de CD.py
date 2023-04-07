#Control PID Motor de CD
import numpy as np 
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import math
import control
#####################################
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
####################
#Tiempo de Simulacion
start = 0
stop = 0
step = 1e-3
t = np.arange(start, stop, step)
########################
def f(x,t):
    dx_dt=[0,0,0]
    va =12 #Entrada de voltaje
    n = (n1/n2)
    Beq=Bm+pow(n,2)*BL #Friccion Equivalente
    JEq = Jm+pow(n, 2)*JL #Momento de inercia Equivalente
    dx_dt[0]=x[1]
    dx_dt[1]=-(Beq/JEq)*x[1]+(km/JEq)*x[2]
    dx_dt[2]=-(Kb/La)*x[1]-(Ra/La)*x[2]+(1/La)*va
    return dx_dt
##################################
Sol = odeint(f, y0=[0, 0, 0], t = t)
print = Sol
#####################################
#Graficar
#Velocidad Angular del Motor
plt.plot(t, Sol[:,1], 'b', label = 'x1' )
plt.xlabel('T(Segundos)')
plt.ylabel('x2(Rad/S)')
plt.title('Velocidad Angular')
plt.grid()
plt.show()
#Corriente del Motor
plt.plot(t, Sol[:,2], 'b', label = 'x3' )
plt.xlabel('T(Segundos)')
plt.ylabel('x3(Amp)')
plt.title('Corriente del Motor')
plt.grid()
plt.show()
