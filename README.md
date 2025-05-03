# Práctica 2: Filtro de Kalman en ROS 2


En esta práctica se ha tenido por objetivo la implementación del filtro de Kalman para la estimación del estado de un robot móvil en un entorno simulado con Gazebo. Para ello, se han implementado dos versiones de este filtro:

**1. KalmanFilter:** Estima la posición y orientación, (x,y,θ), del robot en el entorno.

**2. KalmanFilter_2:** Además de estimar la posición del robot, también estima su velocidad y velocidad angular, (x, y, θ, vx, vy, ω).


## 1. KalmanFilter
En primer lugar, se definen las matrices A y B (de transición y de control respectivamente) en *motion_models.py* y la matriz C (de observación) en *observation_models.py*; estas matrices serán variables en el tiempo. A continuación, se definen las matrices R y Q, que representan, respectivamente, el error en (x,y,θ) referente al proceso y a las observaciones. Este filtro rompe ligeramente las especificaciones de un filtro de kalman convencional ya que el modelo no es completamente lineal al estimar la posición y orientación a través de la velocidad y la velocidad angular, pero se busca su linealización a través de las matrices A y B.

El primer paso del filtro consiste en la **predicción**. Para ello, se hace uso de las matrices A, B y R, de la señal de control 'u' aplicada (que estará conformada en nuestro caso por las velocidades lineal y angular del robot más un ruido), y el tiempo transcurrido desde el último ciclo de ejecución del filtro. Con estos elementos, se predice un nuevo valor de μ, valor estimado de (x,y,θ) a partir del anterior, además de calcular una matriz de covarianza Σ que representará la fiabilidad del valor de μ. Este proceso se lleva a cabo mediante las siguientes ecuaciones:

$μ_{pred} = A * μ_{ant} + B * u$

$Σ_{pred} = A * Σ_{ant} * A^T + R$

El segundo paso consiste el la **actualización**, que emplea las matrices C y Q, las observaciones 'z' (que tendrán también un cierto ruido) y los valores de μ y Σ predichos en el paso anterior, y que sigue las siguientes ecuaciones:

$K = Σ_{pred} * C^T * (C * Σ_{pred} * C^T + Q)^-1$

$μ = μ_{pred} + K * (z - C * μ_{pred})$

$Σ = (I - K * C) * Σ_{pred}$

Como resultado de este segundo paso, se obtiene el valor final de μ y Σ para cada ciclo de ejecución del filtro. En el primer ciclo de ejecución, se toman como μ_ant y Σ_ant un vector y una matriz nulos respectivamente. 

A continuación, se explicará la experimentación que se ha llevado a cabo con este filtro.

### Experimentación
Para analizar este filtro, se han llevado a cabo tres simulaciones con diferentes configuraciones de ruido:

**Ruido bajo.** 
Tanto la matriz R como la matriz Q tienen en su diagonal los valores [0.02, 0.02, 0.02] (el resto de valores de ambas matrices es nulo).
Dado que le hemos especificado al filtro que el ruido tanto del modelo como de las observaciones es bajo, da aproximadamente la misma importancia a la predicción (que hace uso del ruido del modelo) y a la actualización (que usa el de las observaciones). Se observa que la μ resultante del filtro tiene una varianza apreciable, pero el error no es demasiado grande.

**Ruido alto en la medida.** 
La matriz de ruido en la medida Q tiene ahora en su diagonal los valores [0.2, 0.2, 0.2], mientras que la matriz de ruido en el proceso R es idéntica al caso de ruido bajo.
En este caso, la información que tiene el filtro es que el modelo es bastante más fiable que las observaciones, por lo que da más importancia a la predicción que a la actualización. Como resultado, al tener en este caso menos peso las observaciones (que tienen un error aleatorio), la curva que representa los valores de μ del filtro es mucho más suave que en el caso anterior, pero también tiene un error mucho mayor en los tramos en los que el robot describe una curva más cerrada, aunque acaba convergiendo a errores pequeños en tramos rectos.

**Ruido alto en el proceso.** 
La matriz de ruido en el proceso R tiene ahora en su diagonal los valores [0.2, 0.2, 0.2], mientras que la matriz de ruido en la medida Q es idéntica al caso de ruido bajo.
En este último caso, la información que se le da al filtro es que los datos obtenidos mediante las observaciones son más fiables que los resultados del modelo, por lo que tiene más peso en la estimación de la posición del robot la actualización que la predicción. De este modo, dado que las medidas tienen un error aleatorio y el filtro da menos importancia al modelo en este caso, los valores de μ resultantes del filtro tienen una varianza mayor que en los casos anteriores; sin embargo, reacciona mejor a cambios bruscos en la dirección que la configuración anterior.


## 2. KalmanFilter_2
En este segundo filtro las únicas diferencias respecto del primero residen en los valores y las dimensiones de las matrices A, B, C, R y Q y en las dimensiones de μ y Σ, pues este segundo filtro añade a la estimación los valores de velocidad (en x e y) y la velocidad angular, por lo que se duplican las dimensiones. Además, este filtro, al contrario que el anterior sí es completamente lineal. Dado que los pasos que sigue el filtro son idénticos (a excepción del cambio en las dimensiones comentado anteriormente), se puede pasar directamente a la experimentación.

### Experimentación
Para analizar este filtro, se han llevado a cabo tres simulaciones con diferentes configuraciones de ruido:

**Ruido bajo.** 
Tanto la matriz R como la matriz Q tienen en su diagonal los valores [0.02, 0.02, 0.02, 0.02, 0.02, 0.02] (el resto de valores de ambas matrices es nulo).
De igual modo que para el filtro anterior, le hemos asignado un ruido bajo al modelo y a las observaciones, dando así la misma importancia a la predicción y a la actualización. Igual que con el filtro anterior, la μ que se obtiene del filtro tiene una varianza apreciable cuyo error se debe principalmente a esa varianza. Sin embargo, dado que este modelo es más completo que el de la primera versión del filtro, puede observarse que el error es ligeramente menor.

**Ruido alto en la medida.** 
La matriz de ruido en la medida Q tiene ahora en su diagonal los valores [0.2, 0.2, 0.2, 0.2, 0.2, 0.2], mientras que la matriz de ruido en el proceso R es idéntica al caso de ruido bajo.
En esta segunda configuración, se da al filtro la información de que la predicción del modelo es más fiable que las medidas recibidas, por lo que se da más valor a esta que a la actualización. Es por ello que, al tener menos peso las observaciones y del mismo modo que con el primer filtro, la curva de los valores de μ del filtro es mucho más suave que con la configuración de ruidos anterior, aunque con un error más destacable en los tramos con un giro cerrado, que converge lentamente a valores más correctos en tramos más uniformes.

**Ruido alto en el proceso.** 
La matriz de ruido en el proceso R tiene ahora en su diagonal los valores [0.2, 0.2, 0.2, 0.2, 0.2, 0.2], mientras que la matriz de ruido en la medida Q es idéntica al caso de ruido bajo.
Con esta configuración, de igual forma que con el primer filtro, se especifica al filtro es que las observaciones dan mejores aproximaciones que el modelo, dando así más relevancia a la actualización que a la predicción. Así, como con el filtro que no estima la velocidad, las posiciones que estima el filtro tienen mayor varianza que con las otras configuraciones de los ruidos a cambio de asegurar una mejor reacción frente a cambios bruscos en la velocidad.



