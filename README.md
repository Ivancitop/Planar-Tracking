
def make_object_points(self):
    w = self.target_width_m
    h = self.target_height_m

    return np.array([
        [-w / 2,  h / 2, 0.0],
        [ w / 2,  h / 2, 0.0],
        [ w / 2, -h / 2, 0.0],
        [-w / 2, -h / 2, 0.0]
    ], dtype=np.float64)
Planar-Tracking

Nombre legal completo: Iván Delgado Ramos
Correo universitario: A01645788@tec.mx
Número de matrícula / Student ID: A01645788
Descripción del proyecto

Paquete de ROS 2 en Python que segmenta un objeto rectangular físico del fondo utilizando información de profundidad y operaciones morfológicas. Después de la segmentación, el sistema estima la pose 3D del objeto y publica la transformación correspondiente en el árbol TF de ROS 2.

El objeto objetivo es un rectángulo de dimensiones físicas conocidas:

    Ancho (X): 105.6 mm (0.1056 m).
    Alto (Y): 74.0 mm (0.0740 m).
    Plano del objeto: Z = 0.
    Origen del sistema de coordenadas del objeto: centro geométrico del rectángulo.

1. Calibración de la cámara

La matriz intrínseca de la cámara, K, se obtiene del tópico CameraInfo grabado en el rosbag. Esta matriz contiene las distancias focales en píxeles (fx y fy) y las coordenadas del punto principal (cx y cy). Los coeficientes de distorsión se obtienen por separado de la información de calibración y también se proporcionan a solvePnP.

El vector de traslación, tvec, se estima al encontrar la rotación y la traslación que relacionan las coordenadas 3D conocidas de las esquinas del rectángulo con sus posiciones 2D detectadas en la imagen. Una calibración inexacta introduce errores sistemáticos en esta estimación. Por ejemplo, un error en las distancias focales modifica la relación entre el tamaño aparente del objeto y la distancia estimada. Si la distancia focal se sobreestima, la profundidad estimada también puede sobreestimarse, especialmente cuando el objeto está aproximadamente de frente y centrado en la imagen. Los errores en el punto principal o en los coeficientes de distorsión también pueden afectar la traslación y la rotación estimadas.

Por esta razón, utilizar la matriz intrínseca y los coeficientes de distorsión proporcionados por CameraInfo es importante para obtener una estimación de pose fiable.
2. Morfología y segmentación

La máscara binaria se refina mediante una apertura morfológica seguida de una clausura, utilizando un elemento estructurante de 3 × 3 píxeles.

La apertura (MORPH_OPEN), que consiste en una erosión seguida de una dilatación, elimina pequeñas regiones aisladas generadas por el ruido tipo speckle de la profundidad. La clausura (MORPH_CLOSE), que consiste en una dilatación seguida de una erosión, rellena pequeños huecos y mejora la continuidad de la región segmentada del objetivo.

El kernel de 3 × 3 se seleccionó experimentalmente después de compararlo con kernels más grandes de 5 × 5 y 7 × 7. En las pruebas realizadas, el kernel más pequeño ofreció el mejor equilibrio entre la eliminación del ruido y la conservación de los límites y esquinas del rectángulo. Los kernels más grandes pueden eliminar más ruido, pero también pueden deformar la geometría del objetivo y afectar la extracción posterior del contorno y la estimación de pose.

La segmentación también excluye explícitamente los valores de profundidad iguales a cero, ya que representan mediciones inválidas y no una distancia real al objeto.
3. Error de reproyección

El error de reproyección se evalúa proyectando las coordenadas 3D estimadas sobre la imagen mediante cv2.projectPoints y comparando las coordenadas proyectadas con las esquinas detectadas. Para cada frame se calcula el RMS de los errores de las cuatro esquinas:
RMS=14∑i=14ei2
RMS=41​i=1∑4​ei2​
​

donde e_i es la distancia euclidiana, expresada en píxeles, entre la posición detectada y la posición proyectada de la esquina i.

En la secuencia reportada, los frames 2 a 6 presentaron errores RMS de 1.37, 1.50, 1.73, 1.36 y 1.58 píxeles, respectivamente. El promedio de estos cinco frames es aproximadamente 1.51 píxeles, lo que indica una buena concordancia entre las esquinas detectadas y las proyecciones del modelo 3D durante ese intervalo.

Si el error de reproyección se concentra en una sola esquina mientras las otras tres presentan errores bajos, puede indicar una localización incorrecta de esa esquina por artefactos de segmentación, oclusión o una aproximación imprecisa del contorno. Geométricamente, esa esquina no coincide con la proyección que predice la pose estimada. Esto puede indicar un error local en la correspondencia entre puntos 2D y 3D o una diferencia entre el modelo planar asumido y el objeto observado.

Un RMS bajo no garantiza por sí solo que la pose 3D sea físicamente exacta: también depende de la calibración, las dimensiones utilizadas y la correspondencia correcta entre las esquinas.
4. Análisis de fallos durante una inclinación pronunciada

Cuando el rectángulo rota hacia una orientación casi de canto, aproximadamente a 80° respecto a la vista frontal, su ancho aparente disminuye considerablemente. Al acercarse a 90°, el área proyectada de una superficie plana puede hacerse muy pequeña, dificultando la segmentación y la detección fiable de sus esquinas.

Esta condición puede hacer que la máscara de profundidad pierda píxeles válidos del objetivo o que el contorno deje de representar el rectángulo completo. Si la máscara pierde el objeto antes de la extracción del contorno, la primera etapa que falla es la segmentación por profundidad. Si la máscara permanece intacta, pero la aproximación del contorno deja de identificar las cuatro esquinas físicas, la primera etapa que falla es la extracción de esquinas.

Cuando las cuatro esquinas ya no se detectan de forma fiable, las correspondencias 2D–3D proporcionadas a solvePnP dejan de ser confiables y la estimación de rotación y traslación puede volverse inexacta. En consecuencia, la pose publicada y la transformación TF pueden dejar de representar correctamente la posición física del objetivo.

Este comportamiento evidencia una limitación geométrica del seguimiento planar monocular: cuando un objeto plano se aproxima a una orientación de canto, su área proyectada puede resultar

