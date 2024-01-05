# USV simulator

USV simulator, based on [this repo](https://github.com/godardma/PythonVehicleSimulator)

## Auteur :

:student: Maël GODARD <mael.godard@ensta-bretagne.org> (STIC/ROB)

## Git Structure :

* :file_folder: [/helios_ros2](usv_simulator) : **dossier contenant les sources**
* :file_folder: [/mesh](mesh) : **dossier contenant le mesh d'un USV**
* :file_folder: [/rviz2_config](rviz2_config) : **dossier contenant la config rviz2 conseillée**
* :spiral_notepad: [package.xml](package.xml)
* :spiral_notepad: [setup.py](setup.py)    **fichier de setup ROS2 python**
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
colcon build --symlink-install --packages-select usv_simulator
. install/setup.zsh # pour les terminaux zsh
. install/setup.bash # pour les terminaux bash

```

Le package étant en python, le paramètre --symlink-install permet de ne pas avoir à recompiler le package après un changement.

## Lancement :
```bash
ros2 launch usv_simulator simulator
```

* Visualisation

    Dans un terminal, lancer rviz2 et ouvrir la config jointe dans le dossier [/rviz2_config](rviz2_config) pour visualiser l'USV.