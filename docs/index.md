# Introducción e instalación

Los robots ejecutan múltiples algoritmos de control simultáneamente y gestionar todos estos algoritmos de control puede ser complicado. Ros2_control permite ayudar a los desarrolladores a implementar controladores de manera rápida y escalable, además de ofrecer muchas funciones integradas adicionales. Así, ros2_control es un conjunto de paquetes y herramientas que permiten, básicamente, enviar comandos y comunicarse con las articulaciones de un robot con el objetivo de controlarlas. El principal objetivo de esta práctica es mostrar como integrar ros2_control en un entorno simulado, de forma que pueda usarse esta herramienta para controlar las articulaciones de un robot.

Para desarrollar esta práctica debe estar instalado ros2_control así como una simulación de un robot RR de dos grados de libertad rotacionales. Para hacer esta práctica puede utilizarse la siguiente máquina virtual:

También es posible utilizar otra máquina virtual o una instalación nativa de ubuntu que cumpla lo siguiente:

* Ubuntu 24.04
* Ros 2 Jazzy Jalisco. Para instalarlo seguir los pasos indicados [aquí](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
* Instalar ros2_ control:
```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```
* Crear una carpeta ros2_ws en el directorio home y dentro de esa carpeta crear otra llamada src:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

* Descomprimir el siguiente zip y copiar la carpeta en la carpeta ros2_robot_sca en el directorio ~/ros2_ws/src que acabas de crear:
```
https://drive.google.com/drive/folders/11q3OVL2HK4nRVVNhmQg8WSTV5-36kYaW?usp=drive_link
```
Este zip contiene la carpeta ros2_robot_sca que contiene la simulación de un robot de dos grados de libertad que se utilizará a lo largo de la práctica.

* Ahora ya se podrá compilar el repositorio:
```
cd ~/ros2_ws/
. /opt/ros/jazzy/setup.sh
colcon build
```
