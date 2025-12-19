import cv2
import numpy as np

# Variables globales para almacenar los puntos seleccionados
puntos = []
blob_count = 0  # Contador para los blobs
nblob = 4
# Función para guardar cada blob


def guardar_blob():
    global puntos, blob_count
    if len(puntos) >= 3:  # Asegurarse de que haya al menos 3 puntos
        # Convertir la lista de puntos en un array de numpy
        puntos_np = np.array(puntos, np.int32)

        # Crear una máscara (blanco y negro) para el área del polígono
        mascara = np.zeros_like(img)
        cv2.fillPoly(mascara, [puntos_np], (255, 255, 255))

        # Hacer un AND entre la imagen original y la máscara
        imagen_recortada = cv2.bitwise_and(img, mascara)

        # Mostrar la imagen recortada
        cv2.imshow(f'Imagen Recortada {blob_count}', imagen_recortada)

        # Ahora, extraer los valores RGB que no tengan 0 ni 255 en ningún canal
        no_0_255 = set()  # Usamos un set para almacenar valores únicos

        for i in range(imagen_recortada.shape[0]):
            for j in range(imagen_recortada.shape[1]):
                # Obtener el valor del píxel (BGR)
                pixel = imagen_recortada[i, j]
                # Verificar si ninguno de los componentes es 0 ni 255
                if np.all((pixel != [0, 0, 0]) &
                          (pixel > 0) & (pixel <= 255)):
                    # Convertimos el pixel a tupla y lo añadimos al set
                    no_0_255.add(tuple(pixel))

        # Guardar los valores RGB únicos en el archivo txt
        with open('valores_rosa.txt', 'a') as f:  # Usar 'a' para agregar a los anteriores
            for rgb in no_0_255:
                f.write(f'{rgb[0]}, {rgb[1]}, {rgb[2]}\n')

        print(
            f"Se han guardado {len(no_0_255)} valores RGB unicos en el archivo 'valores_rgb_unicos.txt'.")

# Función para manejar la selección de puntos con el mouse


def seleccionar_puntos(event, x, y, flags, param):
    global puntos, blob_count

    if event == cv2.EVENT_LBUTTONDOWN:
        # Añadir punto
        puntos.append((x, y))
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)  # Dibujar un círculo rojo
        if len(puntos) > 1:
            # Dibujar el polígono
            cv2.polylines(img, [np.array(puntos, np.int32)],
                          isClosed=False, color=(0, 255, 0), thickness=2)
        cv2.imshow('Imagen', img)

    # Cerrar el polígono con botón derecho
    if event == cv2.EVENT_RBUTTONDOWN and len(puntos) >= 3:
        if blob_count < nblob:
            blob_count += 1
            guardar_blob()
            puntos = []  # Limpiar los puntos para el siguiente blob


# Ruta completa de la imagen
imagen_path = r'9.png'  # Actualiza esta línea con la ruta correcta

# Leer la imagen
img = cv2.imread(imagen_path)

# Crear una ventana para mostrar la imagen
cv2.imshow('Imagen', img)
cv2.setMouseCallback('Imagen', seleccionar_puntos)

# Esperar hasta que se hayan seleccionado 3 blobs
while blob_count < nblob:
    cv2.waitKey(1)

# Esperar hasta que se presione una tecla para cerrar
cv2.waitKey(0)
cv2.destroyAllWindows()
