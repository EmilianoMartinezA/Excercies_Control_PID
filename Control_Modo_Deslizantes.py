###############################
#Control por Modos Deslizantes
###############################
#Importación de librerías
import numpy as np
import math as math
from scipy.integrate import odeint
import matplotlib.pyplot as plt
###############################
#Parametros del Robot
Ib = 1/3
Bb = 0.01
m = 1
lc = 0.5
l = 1
Jm = 0.008
Bm = 0.04
k = 50
g = 9.81
####################################################
#Vector de Tiempo de Simulación
start = 0
stop = 30
step = 1e-3
t = np.arange(start, stop, step)
####################################################
a = 1
def f(x,t):
    #Superficie Deslizante
    dx_dt = [0, 0, 0, 0]
    r = math.sin(t)
    dr = math.cos(t)
    d1 = math.exp(-3*t)
    d2 = 5 * math.exp(-3*t)
    #Ganancias de la ley de Control
    k1 = 0.49
    k2 = 0.1
    k3 = 0.1
    K = 1
    # Superficie deslizante    
    s = k1*(x[0] - r) + k2*x[1] + k3*x[2] + x[3]
    #Control Equivalente
    WEq = k1*(x[1] - dr) + k2*(-(Bb/Ib)*x[1] - ((m*g*lc)/Ib)*math.sin(x[0])  - (k/Ib)*(x[0] - x[2]) + (1/Ib)*d1) + k3*x[3]
    #Control por modos  deslizantes
    w = WEq - K * pow(math.fabs(s), a) * np.sign(s)
    #ley de Control
    u = Jm*w + Bm*x[3] + k*(x[2] - x[0]) - d2
    dx_dt[0] = x[1]
    dx_dt[1] = -(Bb/Ib)*x[1] - ((m*g*lc)/Ib)*math.sin(x[0]) - (k/Ib)*(x[0] - x[2]) - (1/Ib)*d1
    dx_dt[2] = x[3]
    dx_dt[3] = -(Bm/Jm)*x[3] - (k/Jm)*(x[2] - x[0]) + (1/Jm)*u + (1/Jm)*d2 
    return dx_dt

####################################################
#Solución de las ecuaciones diferenciales
Solucion = odeint(f, y0 =[0,0,0,0],t=t)
print('Solución', Solucion)

#Graficas
#Posición Angular del Robot
plt.plot(t,Solucion[:, 0], 'b', label='x1(t)')
plt.plot(t, np.sin(t), 'r', label='Referencia') #Referencia
plt.xlabel('Tiempo (seg)')
plt.ylabel('x1(t)')
plt.title('Angulo del Robot (rad)')
plt.legend()
plt.grid()
plt.show()

#Velocidad Angular del Robot
plt.plot(t,Solucion[:, 1], 'r', label='x2(t)')
plt.xlabel('Tiempo (seg)')
plt.ylabel('x2(t)')
plt.title('Velocidad Angular del Robot (rad/s)')
plt.grid()
plt.show()
"""
#Posición Angular del Motor
plt.plot(t,Solucion[:, 2], 'b', label='x3(t)')
plt.plot(t, np.sin(t), 'r', label='Referencia') #Referencia
plt.xlabel('Tiempo (seg)')
plt.ylabel('x1(t)')
plt.title('Angulo del Motor (rad)')
plt.legend()
plt.grid()
plt.show()

#Velocidad Angular del Motor
plt.plot(t,Solucion[:, 3], 'r', label='x4(t)')
plt.xlabel('Tiempo (seg)')
plt.ylabel('x2(t)')
plt.title('Velocidad Angular del Motor (rad/s)')
plt.grid()
plt.show()
"""