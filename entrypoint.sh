#!/bin/bash
set -e

# Configurar entorno de ROS
source /opt/ros/jazzy/setup.bash

# Verificar si existe la carpeta montada y ejecutar rosdep
# Esto se ejecutará cada vez que inicies el contenedor (docker compose up)
if [ -d "/home/ubuntu/ros_ws/src/Stage" ]; then
    echo "Volumen detectado. Verificando dependencias..."
    # Actualizamos rosdep y ejecutamos install
    # Usamos sudo porque instalar dependencias requiere root
    rosdep update
    sudo rosdep install --from-paths /home/ubuntu/ros_ws/src/Stage --ignore-src -r -y
    sudo rosdep install --from-paths /home/ubuntu/ros_ws/src/stage_ros2 --ignore-src -r -y
else
    echo "ADVERTENCIA: No se detectó el código fuente en /home/ubuntu/ros_ws/src"
fi

echo "🚀 Contenedor iniciado correctamente: $(hostname)"
echo "Usuario actual: $(whoami)"
echo "Directorio de trabajo: $(pwd)"
echo "Fecha de inicio: $(date)"
echo "--------------------------------------"

# Si se pasa un comando, ejecutarlo
if [ "$#" -gt 0 ]; then
    exec "$@"
else
    # Si no hay comando, mantener una shell interactiva abierta
    exec bash
fi