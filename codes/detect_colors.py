import cv2
import numpy as np
import sys
from typing import Tuple, Dict


def read_values(filename: str) -> np.ndarray:
    values = []
    with open(filename, 'r') as f:
        for line in f:
            rgb = tuple(map(int, line.strip().split(', ')))
            values.append(rgb)
    return np.array(values)


def compute_stats(matriz: np.ndarray) -> Dict[str, np.ndarray]:
    media = np.mean(matriz, axis=0)
    covarianza = np.cov(matriz, rowvar=False)
    # usar solo la diagonal (independencia entre canales)
    covarianza = np.diag(np.diag(covarianza))
    deter = np.linalg.det(covarianza)
    inversa = np.linalg.inv(covarianza)
    constante_pk = 1 / np.sqrt(((2 * np.pi) ** 3) * deter)
    return {
        'mean': media,
        'cov': covarianza,
        'det': deter,
        'inv': inversa,
        'const': constante_pk,
    }


def compute_pk_for_image(imagen: np.ndarray, stats: Dict[str, np.ndarray]) -> np.ndarray:
    imagen_float = np.float64(imagen)
    imagen_rgb_float = imagen_float.reshape((-1, 3))
    dif = imagen_rgb_float - stats['mean']
    sk = np.sum(dif @ stats['inv'] * dif, axis=1)
    pk = stats['const'] * np.exp(-sk / 2)
    return pk


def segment_by_thresholds(imagen: np.ndarray, pks: Tuple[np.ndarray, np.ndarray, np.ndarray],
                          thresholds: Tuple[float, float, float]) -> np.ndarray:
    imagen_rgb_float = np.float64(imagen).reshape((-1, 3))
    new_img = np.zeros_like(imagen_rgb_float, dtype=np.uint8)
    pk_verde, pk_azul, pk_rosa = pks
    umbral_verde, umbral_azul, umbral_rosa = thresholds
    new_img[pk_verde > umbral_verde] = [0, 255, 0]
    new_img[pk_azul > umbral_azul] = [255, 0, 0]
    new_img[pk_rosa > umbral_rosa] = [0, 0, 255]
    return new_img.reshape(imagen.shape)


def morphological_processing(segmented: np.ndarray) -> np.ndarray:
    kernel_cierre = np.ones((3, 3), np.uint8)
    imangen_erosionada = cv2.erode(
        segmented, np.ones((1, 1), np.uint8), iterations=1)
    imagen_open = cv2.morphologyEx(
        imangen_erosionada, cv2.MORPH_OPEN, kernel_cierre)

    kernel_diamante = np.array([[0, 0, 1, 0, 0],
                                [0, 1, 1, 1, 0],
                                [1, 1, 1, 1, 1],
                                [0, 1, 1, 1, 0],
                                [0, 0, 1, 0, 0]], dtype=np.uint8)
    imagen_erosionada = cv2.erode(imagen_open, kernel_diamante, iterations=1)
    imagen_final = cv2.dilate(
        imagen_erosionada, kernel_diamante, iterations=2)

    return imagen_final


def filter_keep_large_circles(imagen_final: np.ndarray, min_radius: int = 12,
                              circularity_thresh: float = 0.6) -> np.ndarray:
    """Mantiene únicamente las regiones que parecen círculos grandes.

    Para cada canal (azul, verde, rosa) encuentra contornos y conserva
    sólo aquellos cuyo radio mínimo encerrador >= min_radius y cuya
    relación area_contorno / area_círculo >= circularity_thresh.
    """
    h, w = imagen_final.shape[:2]
    out = np.zeros_like(imagen_final)
    # colores por canal en BGR
    channel_colors = {
        0: (255, 0, 0),
        1: (0, 255, 0),
        2: (0, 0, 255),
    }
    for ch in (0, 1, 2):
        mask = (imagen_final[:, :, ch] == 255).astype(np.uint8) * 255
        if mask.sum() == 0:
            continue
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 0:
                continue
            (x, y), r = cv2.minEnclosingCircle(cnt)
            if r < min_radius:
                continue
            circle_area = np.pi * (r ** 2)
            circularity = area / circle_area if circle_area > 0 else 0
            if circularity >= circularity_thresh:
                # dibujar círculo relleno en la salida
                center = (int(round(x)), int(round(y)))
                radius = int(round(r))
                cv2.circle(out, center, radius, channel_colors[ch], -1)
    return out


def create_masks(imagen_final: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    mask_azul = imagen_final[:, :, 0] == 255
    mask_verde = imagen_final[:, :, 1] == 255
    mask_rosa = imagen_final[:, :, 2] == 255
    return mask_azul, mask_verde, mask_rosa


def keep_only_nearest_mask(mask_from: np.ndarray, mask_to: np.ndarray) -> np.ndarray:
    """Devuelve una nueva máscara booleana basada en `mask_to` donde se conserva
    únicamente el círculo (o contorno) cuya distancia al círculo mayor en
    `mask_from` es mínima.
    """
    # convertir a uint8
    mf = (mask_from.astype(np.uint8) * 255)
    mt = (mask_to.astype(np.uint8) * 255)

    # encontrar contornos en from y to
    cnts_from, _ = cv2.findContours(
        mf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts_from:
        return mask_to
    cnts_to, _ = cv2.findContours(
        mt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts_to:
        return mask_to

    # seleccionar el contorno 'from' de mayor área y calcular su centro
    largest_from = max(cnts_from, key=cv2.contourArea)
    (fx, fy), fr = cv2.minEnclosingCircle(largest_from)
    fx, fy = float(fx), float(fy)

    # para cada contorno 'to' calcular su centro y distancia al from
    best_idx = None
    best_dist = float('inf')
    for i, cnt in enumerate(cnts_to):
        (tx, ty), tr = cv2.minEnclosingCircle(cnt)
        d = np.hypot(tx - fx, ty - fy)
        if d < best_dist:
            best_dist = d
            best_idx = i

    if best_idx is None:
        return mask_to

    # crear nueva máscara: dibujar sólo el círculo mínimo encerrador del contorno elegido
    chosen = cnts_to[best_idx]
    (cx, cy), cr = cv2.minEnclosingCircle(chosen)
    cx_i, cy_i, cr_i = int(round(cx)), int(round(cy)), int(round(cr))
    new_mask = np.zeros_like(mt)
    cv2.circle(new_mask, (cx_i, cy_i), max(1, cr_i), 255, -1)
    return new_mask.astype(bool)


def draw_arrow_between_masks(img: np.ndarray, mask_from: np.ndarray, mask_to: np.ndarray) -> None:
    """Dibuja en `img` una flecha desde el centro de `mask_from` hasta el centro de `mask_to`.
    También dibuja pequeños círculos en los centros detectados.
    Modifica `img` in-place.
    """
    # obtener lista de círculos en cada máscara (x,y,r)
    def find_circles_in_mask(mask: np.ndarray):
        mask_u8 = (mask.astype(np.uint8) * 255)
        blur = cv2.GaussianBlur(mask_u8, (9, 9), 2)
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                                   param1=50, param2=15, minRadius=5, maxRadius=0)
        res = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype(int)
            for c in circles:
                res.append((int(c[0]), int(c[1]), int(c[2])))
            return res
        # fallback a contornos
        contours, _ = cv2.findContours(
            mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 0:
                continue
            (x, y), r = cv2.minEnclosingCircle(cnt)
            res.append((int(round(x)), int(round(y)), int(round(r))))
        return res

    circles_from = find_circles_in_mask(mask_from)
    circles_to = find_circles_in_mask(mask_to)
    if not circles_from or not circles_to:
        return

    # seleccionar el círculo 'from' más grande
    from_circle = max(circles_from, key=lambda c: c[2])
    fx, fy, fr = from_circle

    # en 'to' elegir el círculo más grande y cercano al 'from'
    # estrategia: tomar todos los círculos a distancia <= 1.5 * min_dist y elegir el de mayor radio;
    # si no hay ninguno en ese rango, elegir el de mayor radio global.
    dists = [np.hypot(c[0] - fx, c[1] - fy) for c in circles_to]
    min_dist = min(dists)
    thresh = min_dist * 1.5
    candidates = [c for c, d in zip(circles_to, dists) if d <= thresh]
    if candidates:
        to_circle = max(candidates, key=lambda c: c[2])
    else:
        to_circle = max(circles_to, key=lambda c: c[2])

    from_center = (int(fx), int(fy))
    to_center = (int(to_circle[0]), int(to_circle[1]))

    # dibujar flecha (de azul hacia rosa). Color negro, grosor 2
    cv2.arrowedLine(img, from_center, to_center, (0, 0, 0), 2, tipLength=0.4)


def apply_masks(original: np.ndarray, masks: Tuple[np.ndarray, np.ndarray, np.ndarray]) -> np.ndarray:
    mask_azul, mask_verde, mask_rosa = masks
    imagen_azul = np.zeros_like(original)
    imagen_verde = np.zeros_like(original)
    imagen_rosa = np.zeros_like(original)
    imagen_azul[:, :] = [255, 0, 0]
    imagen_verde[:, :] = [0, 255, 0]
    imagen_rosa[:, :] = [128, 0, 255]
    resultado = original.copy()
    resultado[mask_azul] = imagen_azul[mask_azul]
    resultado[mask_verde] = imagen_verde[mask_verde]
    resultado[mask_rosa] = imagen_rosa[mask_rosa]
    return resultado


def main(video_path: str = 'fulbito.mp4') -> None:
    cap = cv2.VideoCapture(video_path)
    out = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 20.0,
                          (int(cap.get(3)), int(cap.get(4))))
    if not out.isOpened():
        print("Error: No se pudo abrir el archivo de salida para escritura.")
        return
    # Leer valores de los archivos
    matriz_verde = read_values('valores_verde.txt')
    matriz_azul = read_values('valores_azul.txt')
    matriz_rosa = read_values('valores_rosa.txt')

    stats_verde = compute_stats(matriz_verde)
    stats_azul = compute_stats(matriz_azul)
    stats_rosa = compute_stats(matriz_rosa)

    # Umbrales originales
    umbral_verde = 0.00000018
    umbral_azul = 0.00000001
    umbral_rosa = 0.000000003

    while True:
        ret, imagen = cap.read()
        if not ret:
            break

        pk_verde = compute_pk_for_image(imagen, stats_verde)
        pk_azul = compute_pk_for_image(imagen, stats_azul)
        pk_rosa = compute_pk_for_image(imagen, stats_rosa)

        segmented = segment_by_thresholds(imagen, (pk_verde, pk_azul, pk_rosa),
                                          (umbral_verde, umbral_azul, umbral_rosa))
        cv2.imshow('Imagen Segmentada 2', segmented)
        imagen_final = morphological_processing(segmented)
        # Filtrar y quedarnos sólo con círculos grandes y relativamente circulares
        cv2.imshow('Debug', imagen_final)
        imagen_final = filter_keep_large_circles(
            imagen_final, min_radius=5, circularity_thresh=0.63)
        masks = create_masks(imagen_final)
        # mantener únicamente el círculo rosa más cercano al azul
        mask_azul, mask_verde, mask_rosa = masks
        mask_rosa = keep_only_nearest_mask(mask_azul, mask_rosa)
        masks = (mask_azul, mask_verde, mask_rosa)
        resultado = apply_masks(imagen, masks)
        # Dibujar flecha desde el círculo azul hacia el círculo rosa (si ambos existen)
        mask_azul, mask_verde, mask_rosa = masks
        draw_arrow_between_masks(resultado, mask_azul, mask_rosa)

        out.write(resultado)
        cv2.imshow('Imagen Segmentada', resultado)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        main()
