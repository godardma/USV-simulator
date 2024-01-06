# USV simulator

Package ROS2 simulant la dynamique d'un catamaran motorisé (type Otter de Maritime Robotics), basé sur [ce repo](https://github.com/godardma/PythonVehicleSimulator)

## Auteur :

:student: Maël GODARD <mael.godard@ensta-bretagne.org> (STIC/ROB)

## Git Structure :

* :file_folder: [/foxglove](foxglove) : **dossier contenant un layout conseillé pour Foxglove Studio**
* :file_folder: [/launch](launch) : **dossier contenant les launcher**
* :file_folder: [/mesh](mesh) : **dossier contenant le mesh d'un USV**
* :file_folder: [/msg](msg) : **dossier contenant les messages custom**
* :file_folder: [/rviz2_config](rviz2_config) : **dossier contenant la config rviz2 conseillée**
* :file_folder: [/usv_simulator](usv_simulator) : **dossier contenant les sources**
* :spiral_notepad: [package.xml](package.xml)
* :spiral_notepad: [CMakeLists.txt](CMakeLists.txt)    **fichier CMake**
* :spiral_notepad: [README.md](README.md)

## Technologies :

* Ubuntu 22.04
* Python
* ROS2 Humble


## Building the package

* Installer le package python_vehicle_simulator comme décrit [ici](https://github.com/godardma/PythonVehicleSimulator)
* Cloner le repo dans le dossier src d'un workspace ROS2 Humble
* Se placer à la racine du workspace ROS2 Foxy
* Build le package :
```bash
colcon build --packages-select usv_simulator
. install/setup.zsh # pour les terminaux zsh
. install/setup.bash # pour les terminaux bash

```

## Lancement :

### Simulateur seul :

```bash
ros2 run usv_simulator boat_simulator.py
```

### Simulateur et contrôleur :

```bash
ros2 launch usv_simulator launcher.launch.py
```

L'USV simulé peut ainsi être contrôlé par un message de type Twist, par exemple en utilisant rqt_robot_steering.


## Visualisation

    Dans un terminal, lancer rviz2 et ouvrir la config jointe dans le dossier [/rviz2_config](rviz2_config) pour visualiser l'USV.