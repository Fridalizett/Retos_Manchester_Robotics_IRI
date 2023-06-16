from ultralytics import YOLO
import cv2

# Cargar el modelo YOLO
model = YOLO('modelo_0.pt')

# Configurar la cámara
video_capture = cv2.VideoCapture(0)  # Cambiar el índice para usar otra cámara

while True:
    # Capturar un frame de la cámara
    ret, frame = video_capture.read()

    # Realizar la detección con el modelo en el frame
    results = model(frame)

    # Iterar sobre los resultados individuales
    for result in results:
        # Verificar si hay detecciones en el resultado actual
        if result.boxes is not None and len(result.boxes) > 0:
            # Obtener las coordenadas de las cajas detectadas
            predictions = result.boxes.xyxy

            # Iterar sobre las predicciones y dibujar las bounding boxes en el frame
            for box in predictions:
                x1, y1, x2, y2 = box[0:4].tolist()  # Obtener coordenadas x e y de la caja

                # Obtener la confianza y el ID de clase de la predicción
                confidence = result.boxes.conf[0].item()  # La confianza está en result.boxes.conf
                class_id = result.boxes.cls[0].item()  # El ID de clase está en result.boxes.cls

                label = model.names[class_id]
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.putText(frame, f'{label}: {confidence:.2f}', (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

    # Mostrar el frame con las detecciones
    cv2.imshow('Detección de señales', frame)

    # Salir del bucle si se presiona 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar las ventanas
video_capture.release()
cv2.destroyAllWindows()
