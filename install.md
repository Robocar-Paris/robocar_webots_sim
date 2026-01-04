# Guide d'Installation : ROS 2 Jazzy + Webots + IA sur Fedora 43
**Méthode : Distrobox (Conteneur Ubuntu 24.04)**

Ce guide détaille l'installation d'un environnement de robotique complet sur Fedora en utilisant **Distrobox**. Cela permet de bénéficier de la stabilité officielle de ROS 2 sur Ubuntu tout en gardant Fedora comme système hôte.

---

## 1. Prérequis (Sur Fedora)
Installation de l'outil de conteneurisation.

```bash
sudo dnf install distrobox

# Créer le conteneur
distrobox create --name robocar_sim --image ubuntu:24.04

# Entrer dans le conteneur (toutes les commandes suivantes se font ICI)
distrobox enter robocar_sim

# 1. Outils de base
sudo apt update && sudo apt install curl gnupg2 lsb-release nano -y

# 2. Ajouter la clé GPG ROS
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. Ajouter le dépôt
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Installer ROS 2 Desktop
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions -y

# 1. Télécharger la version officielle .deb
wget [https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb](https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb)

# 2. Installer Webots (gère les dépendances automatiquement)
sudo apt install ./webots_2025a_amd64.deb -y

# 3. Configurer la variable d'environnement (Important !)
export WEBOTS_HOME=/usr/local/webots

sudo apt install ros-jazzy-webots-ros2 -y

# 1. Installer le module venv
sudo apt install python3-venv -y

# 2. Créer l'environnement (avec accès system-site-packages pour ROS)
python3 -m venv --system-site-packages ~/ros2_ai_env

# 3. Installer les librairies d'IA
source ~/ros2_ai_env/bin/activate
pip install torch stable-baselines3 shimmy gymnasium

# 1. Entrer dans le conteneur
distrobox enter robocar_sim

# 2. Activer ROS 2
source /opt/ros/jazzy/setup.bash

# 3. (Optionnel) Si non configuré dans le .bashrc
export WEBOTS_HOME=/usr/local/webots

# 4. Lancer la simulation (Exemple Tesla)
ros2 launch webots_ros2_tesla robot_launch.py

# 1. Entrer dans le conteneur
distrobox enter robocar_sim

# 2. Activer ROS 2 ET l'environnement Python IA
source /opt/ros/jazzy/setup.bash
source ~/ros2_ai_env/bin/activate

# 3. Vérifier que tout communique
ros2 topic list

# 4. Lancer ton script
python3 mon_script_ia.py

