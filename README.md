## Описание

Репозиторий содержит ряд пакетов ROS, относящихся к проекту `puma01`. 

**Версия ROS:** Noetic  
**ОС:** Ubuntu 20.04

Для сборки рекомендуется применять `catkin build`

## ROS-пакеты:
**1. puma01_description:** содержит URDF робота;   
**2. puma01_control:** содержит код кастомных контроллеров, а также файл с конфигурацией;  
**3. puma01_gazebo:** содержит launch-файлы для загрузки описания робота в Parameter Server и запуска симуляции Gazebo;  
**4. puma01_moveit:** пакет создан автоматически при помощи MoveIt Setup Assistant;  
**5. puma01_trajectory_executer:** содержит ROS-ноду, реализующую примитивный механизм выполнения заранее спланированной траектории;  

<!--
## TO-DO
- добавить команды, используемые для запуска всего
- по-английски всё писать может??
-->
