Dise�o Anal�tico de un Controlador PI Anal�gico y por Redise�o Digital
======================================================================

> Dise�o anal�tico de un controlador PI anal�gico y digital para un sistema
de primer orden (circuito RC).

planta: **Gp(s)=gain/(s+1/tau)**

Nota: **fs = 30Hz**.
 
Se resuelven en el siguiente repo:
- [x] Dise�o anal�gico de un controlador PI.
- [x] Redise�o digital de un controlador PI por Tust�n.
- [x] Comparaci�n entre control PI y PID.

MATLAB 
------
Se identifica el sistema usando la estructura parametrica ARX.

```matlab
>> iden_arx.m
```

Hallamos los controladores PI y exportamos los coeficientes obtenidos del 
controlador digital al directorio */data* para su procesamiento en labview.

```matlab
>> control_pi.m
```

Hallamos los controladores PID y exportamos los coeficientes obtenidos del
controlador digital al directorio */data* para su procesamiento en labview.

```matlab
>> control_pid.m
```

LABVIEW 
-------
Identificaci�n de nuestra plata (Para comparar con el resultado de matlab
obtenido en *iden_arx.m*)
```labview
>> arx.vi
```

Validamos nuestro controlador digital el modelos de nuestra planta 
identificada.
```labview
>> simulado.vi
```

Validaci�n en tiempo real con nuestra planta experimental.
```labview
>> tiempo_real.vi
```

**Advertencia** that to run all the labview files. First you need to change the 
absolute paths to the ones that belong you.


DATA 
----
Data adquirida con una DAQ.
```labview
>> data_rc.lvm
```

Coeficientes generados, representan el modelo de la planta identificada.
```labview
>> coef_planta.lvm
```

Numerador del controlador digital.
```labview
>> num_controller.lvm
```

Denominador del controlador digital.
```labview
>> den_controller.lvm
```

*Nota*
Dependiendo que script ejecutemos:
* control_pi.m
* control_pid.m

, obtendremos los coeficientes generados 
en nuestra */data* para nuestro controlador digital.


Resultados Obtenidos 
====================

Data Obtenida
-------------
![data](https://raw.github.com/oskargicast/ControladorPI/master/imagenes/data.png
 "data")

Identificaci�n ARX
------------------
![identificacion_arx](https://raw.github.com/oskargicast/ControladorPI/master/imagenes/identificacion_arx.png "identificacion_arx")

Planta controlada usando un controlador PI
------------------------------------------
![pi_sistema_controlado](https://raw.github.com/oskargicast/ControladorPI/master/imagenes/pi_sistema_controlado.png "pi_sistema_controlado")

Planta controlada usando un controlador PID
-------------------------------------------
![pid_sistema_controlado](https://raw.github.com/oskargicast/ControladorPI/master/imagenes/pid_sistema_controlado.png "pid_sistema_controlado")

Comparaci�n entre PI y PID
--------------------------
![compara_pi_pid](https://raw.github.com/oskargicast/ControladorPI/master/imagenes/compara_pi_pid.png "compara_pi_pid")



***


