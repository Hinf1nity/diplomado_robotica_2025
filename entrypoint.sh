#!/bin/bash
set -e

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