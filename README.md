## Описание

Репозиторий содержит ряд пакетов ROS, созданных в реализации системы управления для робота РМ-01 (PUMA 560). Для активной разработки на данный момент используется ветка [develop](https://github.com/skbmir/puma01/tree/develop).

**Версия ROS:** Noetic  
**ОС:** Ubuntu 20.04

Для сборки настоятельно рекомендуется применять `catkin build`. Установка системы сборки `catkin`: 
```
sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon
``` 

Чтобы установить все необходимые пакеты:
```
cd ~/your_moveit_ws/src 
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```
<!-- 
Докачка необходимых пакетов, но они должны автоматически скачаться, если правильно заполнены файлы package.xml: 

ros_control_boilerplate:
sudo apt-get install ros-noetic-ros-control-boilerplate 

rosparam shortcuts:
sudo apt-get install ros-$ROS_DISTRO-rosparam-shortcuts

moveit:
sudo apt-get install ros-noetic-moveit
-->

<!--
## Описание пакетов:
**1. puma01_description:** URDF робота;   
**2. puma01_control:** кастомные контроллеры ros_control, а также файл с их конфигурацией;  
**3. puma01_gazebo:** launch-файлы для запуска симуляции Gazebo;  
**4. puma01_moveit:** пакет создан автоматически при помощи MoveIt Setup Assistant;  
**5. puma01_hardware_interface:** интерфейс робота, реализованный на основе ros_control_boilerplate;   
**6. puma01_trajectory_executer:** специфичная ROS-нода, реализующая примитивный алгоритм выполнения траектории;  
-->

<!--
## TO-DO
- добавить команды, используемые для запуска всего
- по-английски всё писать может??
-->
