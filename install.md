# Robocar Webots Sim - Sprint 0 : Installation et Prise en Main

## 📋 Table des matières

1. [Introduction](#1-introduction)
2. [Quel système d'exploitation ?](#2-quel-système-dexploitation-)
3. [Installation sur Ubuntu](#3-installation-sur-ubuntu)
4. [Installation sur Fedora](#4-installation-sur-fedora)
5. [Installation sur Windows](#5-installation-sur-windows)
6. [Configuration du projet (Tous OS)](#6-configuration-du-projet-tous-os)
7. [Tutoriel Webots (Tous OS)](#7-tutoriel-webots-tous-os)
8. [Tutoriel ROS 2 (Tous OS)](#8-tutoriel-ros-2-tous-os)
9. [Vérification finale](#9-vérification-finale)
10. [Dépannage](#10-dépannage)

---

# 1. Introduction

## Objectif du Sprint 0

À la fin de ce sprint, tu auras :
- ✅ Un environnement de développement fonctionnel
- ✅ Webots installé et testé
- ✅ ROS 2 Humble installé et testé
- ✅ Le projet Robocar cloné et compilé
- ✅ Les bases de Webots et ROS 2 maîtrisées

## Ce que tu vas installer

| Logiciel | Version | Description |
|----------|---------|-------------|
| **Webots** | R2023b | Simulateur de robot 3D |
| **ROS 2** | Humble | Framework robotique |
| **Python** | 3.10+ | Langage de programmation |
| **Git** | Dernière | Gestion de versions |

---

# 2. Quel système d'exploitation ?

## Choisis ton OS et suis la section correspondante :

| OS | Section | Difficulté | Recommandé pour |
|----|---------|------------|-----------------|
|  **Ubuntu 22.04** | [Section 3](#3-installation-sur-ubuntu) | 🟢 Facile | Débutants en robotique |
|  **Fedora 38/39/40** | [Section 4](#4-installation-sur-fedora) | 🟡 Moyen | Utilisateurs Fedora |
|  **Windows 10/11** | [Section 5](#5-installation-sur-windows) | 🟡 Moyen | Utilisateurs Windows |

### Recommandation

**Si tu as le choix, utilise Ubuntu 22.04** car :
- ROS 2 Humble est officiellement supporté
- Plus de documentation disponible
- Moins de problèmes de compatibilité

**Alternative :** Utilise une machine virtuelle Ubuntu ou Docker.

---

# 3. Installation sur Ubuntu

##  Versions supportées
- **Ubuntu 22.04 LTS** (Jammy) -  Recommandé
- **Ubuntu 24.04 LTS** (Noble) - ️ Utiliser ROS 2 Jazzy

## 3.1 Préparation du système

### Étape 1 : Mettre à jour Ubuntu

```bash
# Ouvre un terminal (Ctrl+Alt+T) et tape :
sudo apt update
sudo apt upgrade -y
```

### Étape 2 : Installer les outils de base

```bash
sudo apt install -y \
    git \
    curl \
    wget \
    nano \
    htop \
    build-essential \
    python3 \
    python3-pip \
    python3-venv \
    software-properties-common
```

### Étape 3 : Vérifier les versions

```bash
python3 --version   # Doit afficher 3.10.x ou plus
git --version       # Doit afficher 2.x.x
```

---

## 3.2 Installation de Webots (Ubuntu)

### Méthode recommandée : APT

```bash
# Ajouter la clé GPG de Cyberbotics
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -

# Ajouter le dépôt
sudo apt-add-repository 'deb https://cyberbotics.com/debian binary-amd64/'

# Mettre à jour et installer
sudo apt update
sudo apt install -y webots
```

### Méthode alternative : Téléchargement direct

```bash
# Télécharger le .deb
cd ~/Downloads
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb

# Installer
sudo apt install -y ./webots_2023b_amd64.deb
```

###  Vérification

```bash
# Lancer Webots
webots &

# Ou depuis le menu Applications
```

**Test :**
1. Va dans `File > Open Sample World`
2. Choisis `robots > boston_dynamics > spot > spot.wbt`
3. Clique sur (Play)
4. Si le robot bouge → Webots fonctionne !

---

## 3.3 Installation de ROS 2 Humble (Ubuntu)

### Étape 1 : Configurer les locales

```bash
locale  # Vérifier que UTF-8 est présent

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Étape 2 : Ajouter les sources ROS 2

```bash
# Installer les outils nécessaires
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Ajouter la clé GPG
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Ajouter le dépôt
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Étape 3 : Installer ROS 2 Humble

```bash
# Mettre à jour les paquets
sudo apt update

# Installer la version Desktop (complète)
sudo apt install -y ros-humble-desktop

# Installer les outils de développement
sudo apt install -y ros-dev-tools
```

### Étape 4 : Configurer l'environnement

```bash
# Ajouter le source automatique au démarrage du terminal
echo '' >> ~/.bashrc
echo '# ROS 2 Humble' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Appliquer immédiatement
source ~/.bashrc
```

### Vérification

```bash
# Vérifier l'installation
ros2 --help

# Test avec le démo talker/listener
# Terminal 1 :
ros2 run demo_nodes_cpp talker

# Terminal 2 (nouveau terminal) :
ros2 run demo_nodes_py listener
```

Si tu vois des messages "Publishing" et "I heard" →  ROS 2 fonctionne !

---

## 3.4 Installation du bridge Webots-ROS2 (Ubuntu)

```bash
# Installer le package webots_ros2
sudo apt install -y ros-humble-webots-ros2
```

###  Vérification

```bash
# Tester avec l'exemple Universal Robot
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

Webots doit s'ouvrir avec des bras robotiques →  Bridge fonctionnel !

---

## 3.5 Installation des outils Python ML (Ubuntu)

```bash
# Installer pip si nécessaire
sudo apt install -y python3-pip

# Installer les bibliothèques ML
pip3 install --user \
    numpy \
    opencv-python \
    matplotlib \
    gymnasium==0.29.1 \
    stable-baselines3==2.2.1 \
    tensorboard \
    torch \
    torchvision
```

###  Vérification

```bash
python3 -c "import gymnasium; print('Gymnasium OK')"
python3 -c "from stable_baselines3 import PPO; print('SB3 OK')"
python3 -c "import torch; print(f'PyTorch OK, CUDA: {torch.cuda.is_available()}')"
```

---

# 4. Installation sur Fedora

##  Versions supportées
- **Fedora 38** - Supporté
- **Fedora 39** - Supporté
- **Fedora 40** - Supporté

## 4.1 Préparation du système

### Étape 1 : Mettre à jour Fedora

```bash
# Ouvre un terminal et tape :
sudo dnf update -y
```

### Étape 2 : Installer les outils de base

```bash
sudo dnf install -y \
    git \
    curl \
    wget \
    nano \
    htop \
    gcc \
    gcc-c++ \
    make \
    cmake \
    python3 \
    python3-pip \
    python3-devel
```

### Étape 3 : Installer Snap (nécessaire pour Webots et ROS 2)

```bash
# Installer snapd
sudo dnf install -y snapd

# Créer le lien symbolique
sudo ln -s /var/lib/snapd/snap /snap

# Activer le service
sudo systemctl enable --now snapd.socket

# IMPORTANT : Redémarrer ou se déconnecter/reconnecter
sudo reboot
```

** Après le redémarrage, ouvre un nouveau terminal pour continuer.**

---

## 4.2 Installation de Webots (Fedora)

### Méthode recommandée : Snap

```bash
# Installer Webots via Snap
sudo snap install webots
```

### Méthode alternative : Tarball

```bash
# Créer le dossier
mkdir -p ~/Applications
cd ~/Applications

# Télécharger Webots
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b-x86-64.tar.bz2

# Extraire
tar xjf webots-R2023b-x86-64.tar.bz2

# Configurer l'environnement
echo '' >> ~/.bashrc
echo '# Webots' >> ~/.bashrc
echo 'export WEBOTS_HOME=$HOME/Applications/webots' >> ~/.bashrc
echo 'export PATH=$WEBOTS_HOME:$PATH' >> ~/.bashrc

source ~/.bashrc
```

### ✅ Vérification

```bash
# Lancer Webots
webots &
```
---
## 4.3 Installation de ROS 2 Humble (Fedora)

### Méthode 1 : Via Snap (Recommandé pour débutants)

```bash
# Installer ROS 2 Humble via Snap
sudo snap install ros-humble-desktop --classic

# Configurer l'environnement
echo '' >> ~/.bashrc
echo '# ROS 2 Humble (Snap)' >> ~/.bashrc
echo 'export PATH=/snap/bin:$PATH' >> ~/.bashrc
echo 'source /snap/ros-humble-desktop/current/opt/ros/humble/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

### Méthode 2 : Via Copr (Plus intégré à Fedora)

```bash
# Activer le dépôt Copr
sudo dnf copr enable tavie/ros2 -y

# Installer ROS 2 Humble
sudo dnf install -y ros-humble-desktop

# Configurer l'environnement
echo '' >> ~/.bashrc
echo '# ROS 2 Humble (Copr)' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

### Méthode 3 : Compilation depuis les sources (Avancé)

<details>
<summary>Cliquer pour voir les instructions détaillées</summary>

```bash
# Installer les dépendances
sudo dnf install -y \
    cmake cppcheck eigen3-devel gcc-c++ liblsan libXaw-devel \
    libyaml-devel make opencv-devel patch python3-colcon-common-extensions \
    python3-coverage python3-devel python3-empy python3-nose python3-pip \
    python3-pydocstyle python3-pyparsing python3-pytest python3-pytest-cov \
    python3-pytest-mock python3-pytest-runner python3-rosdep python3-setuptools \
    python3-vcstool poco-devel poco-foundation python3-flake8 \
    python3-flake8-import-order redhat-rpm-config uncrustify wget

# Installer les dépendances Python
pip3 install --user \
    flake8-blind-except flake8-builtins flake8-class-newline \
    flake8-comprehensions flake8-deprecated flake8-quotes \
    "pytest>=5.3" pytest-repeat pytest-rerunfailures

# Créer le workspace
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Télécharger le code source
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Installer les dépendances avec rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Compiler (peut prendre 1-2 heures)
colcon build --symlink-install

# Configurer l'environnement
echo '' >> ~/.bashrc
echo '# ROS 2 Humble (compilé)' >> ~/.bashrc
echo 'source ~/ros2_humble/install/local_setup.bash' >> ~/.bashrc

source ~/.bashrc
```

</details>

### Vérification

```bash
# Vérifier l'installation
ros2 --help

# Test talker/listener
# Terminal 1 :
ros2 run demo_nodes_cpp talker

# Terminal 2 :
ros2 run demo_nodes_py listener
```

---

## 4.4 Installation du bridge Webots-ROS2 (Fedora)

### Si tu as utilisé Copr :

```bash
sudo dnf install -y ros-humble-webots-ros2
```

### Si tu as utilisé Snap ou compilation :

```bash
# Créer un workspace pour le bridge
mkdir -p ~/webots_ws/src
cd ~/webots_ws/src

# Cloner le package
git clone --branch humble https://github.com/cyberbotics/webots_ros2.git

# Compiler
cd ~/webots_ws
source /opt/ros/humble/setup.bash  # ou le setup de ta méthode
colcon build --symlink-install

# Configurer
echo 'source ~/webots_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Vérification

```bash
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

---

## 4.5 Installation des outils Python ML (Fedora)

```bash
pip3 install --user \
    numpy \
    opencv-python \
    matplotlib \
    gymnasium==0.29.1 \
    stable-baselines3==2.2.1 \
    tensorboard \
    torch \
    torchvision
```

# 6. Configuration du projet (Tous OS)

## 6.1 Créer le workspace ROS 2

### Ubuntu / Fedora / WSL

```bash
# Créer le dossier workspace
mkdir -p ~/robocar_ws/src
cd ~/robocar_ws/src
```

## 6.2 Cloner le projet

### Ubuntu / Fedora / WSL

```bash
git clone https://github.com/Robocar-Paris/robocar_webots_sim.git
```

---

## 6.3 Installer les dépendances Python du projet

```bash
# Tous OS :
cd robocar_webots_sim  # ou le chemin approprié
pip install -r requirements.txt
```

---

## 6.4 Installer les dépendances ROS 2

### Ubuntu / Fedora / WSL

```bash
cd ~/robocar_ws
rosdep install --from-paths src --ignore-src -y
```

### Windows Natif

```powershell
cd C:\dev\robocar_ws
# Note: rosdep n'est pas disponible sur Windows
# Installe les dépendances manuellement si nécessaire
```

---

## 6.5 Compiler le projet

### Ubuntu / Fedora / WSL

```bash
cd ~/robocar_ws
colcon build --symlink-install
```

### Windows Natif

```powershell
cd C:\dev\robocar_ws
colcon build --merge-install
```

---

## 6.6 Configurer l'environnement

### Ubuntu / Fedora / WSL

```bash
echo '' >> ~/.bashrc
echo '# Robocar Workspace' >> ~/.bashrc
echo 'source ~/robocar_ws/install/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

### Windows Natif

Ajoute à ton script de setup :
```batch
call C:\dev\robocar_ws\install\local_setup.bat
```

---

## 6.7 Vérification

```bash
# Liste les packages ROS 2 disponibles
ros2 pkg list | grep robocar

# Si le package apparaît → ✅ Configuration réussie !
```

---

# 7. Tutoriel Webots (Tous OS)

## 🎯 Objectif

Apprendre les bases de Webots pour pouvoir créer et modifier notre simulation.

## ⏱️ Durée estimée : 2 heures

---

## 7.1 Découverte de l'interface

### Lancer Webots

```bash
webots &
```

### L'interface Webots

```
┌─────────────────────────────────────────────────────────────────────┐
│  Menu Bar                                                           │
├──────────────┬──────────────────────────────┬───────────────────────┤
│              │                              │                       │
│  Scene Tree  │     Vue 3D                   │   Text Editor        │
│  (gauche)    │     (centre)                 │   (droite)           │
│              │                              │                       │
│  Liste des   │     Visualisation            │   Code du            │
│  objets      │     du monde                 │   controller         │
│              │                              │                       │
├──────────────┴──────────────────────────────┴───────────────────────┤
│  Console (messages, erreurs)                                        │
└─────────────────────────────────────────────────────────────────────┘
```

### Éléments importants

| Élément | Description |
|---------|-------------|
| **Scene Tree** | Arborescence de tous les objets dans le monde |
| **Vue 3D** | Visualisation interactive de la simulation |
| **Text Editor** | Éditeur de code intégré |
| **Console** | Messages de log et erreurs |
| **Boutons de contrôle** | ▶️ Play, ⏸️ Pause, ⏹️ Stop, 🔄 Reset |

---

## 7.2 Ouvrir un monde exemple

1. `File > Open Sample World`
2. Navigue vers `robots > gctronic > e-puck > e-puck.wbt`
3. Clique sur `Open`

### Explorer la scène

- **Rotation de la vue** : Clic droit + glisser
- **Zoom** : Molette de la souris
- **Déplacement** : Clic molette + glisser

---

## 7.3 Comprendre les concepts de base

### Vocabulaire Webots

| Terme | Définition | Exemple |
|-------|------------|---------|
| **World** | L'environnement de simulation (.wbt) | training_world.wbt |
| **Node** | Un objet dans le monde | Robot, Solid, Light |
| **Field** | Une propriété d'un Node | translation, rotation, color |
| **PROTO** | Un modèle réutilisable | Robocar.proto |
| **Controller** | Le code qui contrôle un robot | robocar_driver.py |
| **Supervisor** | Un controller avec pouvoirs spéciaux | Peut déplacer des objets |

### Types de Nodes courants

```
WorldInfo          → Paramètres du monde (gravité, temps)
Viewpoint          → Position de la caméra
Background         → Couleur du ciel
DirectionalLight   → Source de lumière
RectangleArena     → Sol rectangulaire
Robot              → Un robot contrôlable
Solid              → Un objet physique
Shape              → Apparence visuelle
```

---

## 7.4 Créer ton premier monde

### Étape 1 : Nouveau monde

1. `File > New > New Project Directory...`
2. Nom du projet : `mon_premier_monde`
3. Coche "Add a rectangle arena"
4. `Create`

### Étape 2 : Explorer le monde créé

Regarde le Scene Tree :
```
WorldInfo
Viewpoint
TexturedBackground
TexturedBackgroundLight
RectangleArena
```

### Étape 3 : Ajouter un robot

1. Dans le Scene Tree, clic droit sur le dernier élément
2. `Add New > PROTO nodes (Webots Projects) > robots > gctronic > e-puck > E-puck`
3. Le robot apparaît au centre

### Étape 4 : Lancer la simulation

1. Clique sur ▶️ (Play)
2. Le robot ne bouge pas (pas de controller)

---

## 7.5 Créer ton premier controller

### Étape 1 : Créer le fichier

1. `Wizards > New Robot Controller...`
2. Nom : `mon_controller`
3. Langage : `Python`
4. `Finish`

### Étape 2 : Écrire le code

Un éditeur s'ouvre avec le code de base. Remplace par :

```python
"""
Mon premier controller Webots.
Fait avancer le robot en ligne droite.
"""

from controller import Robot

# Crée l'instance du robot
robot = Robot()

# Récupère le timestep du monde
timestep = int(robot.getBasicTimeStep())

# Récupère les moteurs des roues
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Configure les moteurs en mode vitesse
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Définit la vitesse (rad/s)
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)

# Boucle principale
while robot.step(timestep) != -1:
    # Le robot avance tout seul grâce aux vitesses définies
    pass
```

### Étape 3 : Assigner le controller au robot

1. Dans le Scene Tree, clique sur `E-puck`
2. Dans le panneau de droite, trouve le champ `controller`
3. Change la valeur en `mon_controller`

### Étape 4 : Lancer

1. Sauvegarde le monde (`Ctrl+S`)
2. Clique sur ▶️
3. Le robot avance ! 🎉

---

## 7.6 Exercices pratiques

### Exercice 1 : Faire tourner le robot

Modifie le controller pour que le robot tourne sur lui-même :

```python
# Vitesses différentes = le robot tourne
left_motor.setVelocity(2.0)
right_motor.setVelocity(-2.0)  # Inversé !
```

### Exercice 2 : Détecter un obstacle

Ajoute la détection d'obstacle avec les capteurs de proximité :

```python
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Moteurs
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Capteurs de proximité (l'e-puck en a 8)
sensors = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    sensors.append(sensor)

# Boucle principale
while robot.step(timestep) != -1:
    # Lit les capteurs avant (ps0 et ps7)
    front_left = sensors[7].getValue()
    front_right = sensors[0].getValue()
    
    # Si obstacle devant (valeur > 80)
    if front_left > 80 or front_right > 80:
        # Tourne à droite
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(-2.0)
    else:
        # Avance
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
```

### Exercice 3 : Ajouter un obstacle

1. Dans le Scene Tree, clic droit
2. `Add New > PROTO nodes (Webots Projects) > objects > obstacles > OilBarrel`
3. Place le baril devant le robot
4. Lance la simulation

Le robot doit contourner l'obstacle !

---

## 7.7 Ressources supplémentaires

- **Documentation officielle** : https://cyberbotics.com/doc/guide/index
- **Tutoriels Webots** : https://cyberbotics.com/doc/guide/tutorials
- **API Python** : https://cyberbotics.com/doc/reference/index

---

# 8. Tutoriel ROS 2 (Tous OS)

## 🎯 Objectif

Comprendre les concepts fondamentaux de ROS 2 pour pouvoir créer nos nodes.

## ⏱️ Durée estimée : 3 heures

---

## 8.1 Concepts fondamentaux

### Qu'est-ce que ROS 2 ?

ROS 2 (Robot Operating System 2) est un **framework** pour créer des applications robotiques. Ce n'est pas un système d'exploitation, mais un ensemble d'outils et de bibliothèques.

### Vocabulaire ROS 2

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SYSTÈME ROS 2                               │
│                                                                     │
│   ┌─────────┐         Topic: /camera         ┌─────────┐           │
│   │  Node   │  ─────────────────────────────► │  Node   │           │
│   │ Caméra  │       (Publisher)               │   IA    │           │
│   │         │                                 │         │           │
│   └─────────┘                                 └────┬────┘           │
│                                                    │                │
│                                                    │ Topic: /cmd_vel│
│                                                    ▼                │
│                                               ┌─────────┐           │
│                                               │  Node   │           │
│                                               │ Moteurs │           │
│                                               └─────────┘           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

| Terme | Définition | Analogie |
|-------|------------|----------|
| **Node** | Un programme qui fait une tâche | Un employé dans une entreprise |
| **Topic** | Canal de communication | Un groupe WhatsApp |
| **Message** | Données envoyées | Un message dans le groupe |
| **Publisher** | Node qui envoie des messages | Quelqu'un qui poste |
| **Subscriber** | Node qui reçoit des messages | Quelqu'un qui lit |
| **Service** | Communication requête/réponse | Appeler quelqu'un au téléphone |
| **Action** | Tâche longue avec feedback | Commander un Uber (suivi en temps réel) |

---

## 8.2 Commandes de base

### Lister les nodes actifs

```bash
ros2 node list
```

### Lister les topics

```bash
ros2 topic list
```

### Voir les messages d'un topic

```bash
ros2 topic echo /nom_du_topic
```

### Publier un message manuellement

```bash
ros2 topic pub /nom_du_topic type_message "{champ: valeur}"
```

### Informations sur un topic

```bash
ros2 topic info /nom_du_topic
ros2 topic type /nom_du_topic
```

### Lister les services

```bash
ros2 service list
```

### Appeler un service

```bash
ros2 service call /nom_du_service type_service "{champ: valeur}"
```

---

## 8.3 Premier exercice : Talker/Listener

### Étape 1 : Lancer le talker

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker
```

Tu verras :
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

### Étape 2 : Lancer le listener

```bash
# Terminal 2 (nouveau)
ros2 run demo_nodes_py listener
```

Tu verras :
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

### Étape 3 : Explorer

```bash
# Terminal 3
ros2 node list
# Affiche : /talker, /listener

ros2 topic list
# Affiche : /chatter, /parameter_events, /rosout

ros2 topic echo /chatter
# Affiche les messages en temps réel
```

---

## 8.4 Créer ton premier package ROS 2

### Étape 1 : Créer le package

```bash
cd ~/robocar_ws/src  # ou ton workspace

# Créer un package Python
ros2 pkg create --build-type ament_python mon_premier_package
```

Structure créée :
```
mon_premier_package/
├── package.xml              # Métadonnées du package
├── setup.py                 # Configuration Python
├── setup.cfg
├── resource/
│   └── mon_premier_package
└── mon_premier_package/     # Ton code ici
    └── __init__.py
```

### Étape 2 : Créer un Publisher

Crée le fichier `mon_premier_package/mon_publisher.py` :

```python
#!/usr/bin/env python3
"""
Mon premier publisher ROS 2.
Publie un compteur sur le topic /mon_compteur.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MonPublisher(Node):
    """Node qui publie un compteur."""
    
    def __init__(self):
        super().__init__('mon_publisher')
        
        # Crée un publisher sur le topic '/mon_compteur'
        # Type de message : Int32
        # Queue size : 10
        self.publisher = self.create_publisher(Int32, '/mon_compteur', 10)
        
        # Timer qui appelle la fonction toutes les 0.5 secondes
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Compteur
        self.compteur = 0
        
        self.get_logger().info('Publisher démarré !')
    
    def timer_callback(self):
        """Appelé toutes les 0.5 secondes."""
        msg = Int32()
        msg.data = self.compteur
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publication : {self.compteur}')
        
        self.compteur += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = MonPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Étape 3 : Créer un Subscriber

Crée le fichier `mon_premier_package/mon_subscriber.py` :

```python
#!/usr/bin/env python3
"""
Mon premier subscriber ROS 2.
Écoute le topic /mon_compteur.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MonSubscriber(Node):
    """Node qui écoute le compteur."""
    
    def __init__(self):
        super().__init__('mon_subscriber')
        
        # Crée un subscriber sur le topic '/mon_compteur'
        self.subscription = self.create_subscription(
            Int32,
            '/mon_compteur',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('Subscriber démarré !')
    
    def listener_callback(self, msg):
        """Appelé à chaque message reçu."""
        self.get_logger().info(f'Reçu : {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    node = MonSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Étape 4 : Configurer le package

Modifie `setup.py` :

```python
from setuptools import find_packages, setup

package_name = 'mon_premier_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ton Nom',
    maintainer_email='ton@email.com',
    description='Mon premier package ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mon_publisher = mon_premier_package.mon_publisher:main',
            'mon_subscriber = mon_premier_package.mon_subscriber:main',
        ],
    },
)
```

### Étape 5 : Compiler et tester

```bash
# Compile
cd ~/robocar_ws
colcon build --packages-select mon_premier_package

# Source
source install/setup.bash

# Terminal 1 : Lance le publisher
ros2 run mon_premier_package mon_publisher

# Terminal 2 : Lance le subscriber
ros2 run mon_premier_package mon_subscriber
```

---

## 8.5 Les types de messages courants

### Messages standards

| Package | Message | Utilisation |
|---------|---------|-------------|
| `std_msgs` | `String`, `Int32`, `Float64`, `Bool` | Données simples |
| `geometry_msgs` | `Twist`, `Pose`, `Point` | Géométrie, mouvements |
| `sensor_msgs` | `Image`, `LaserScan`, `Imu` | Capteurs |
| `nav_msgs` | `Odometry`, `Path` | Navigation |

### Exemple : Twist (vitesse)

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 0.5   # Vitesse avant (m/s)
cmd.linear.y = 0.0
cmd.linear.z = 0.0
cmd.angular.x = 0.0
cmd.angular.y = 0.0
cmd.angular.z = 0.2  # Vitesse de rotation (rad/s)
```

### Exemple : Image (caméra)

```python
from sensor_msgs.msg import Image

# L'image est généralement fournie par un driver de caméra
# Tu la reçois via un subscriber
def image_callback(msg):
    print(f"Image reçue : {msg.width}x{msg.height}")
```

---

## 8.6 Exercice : Contrôler Turtlesim

### Étape 1 : Lancer turtlesim

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node
```

Une fenêtre avec une tortue apparaît.

### Étape 2 : Explorer les topics

```bash
# Terminal 2
ros2 topic list
# /turtle1/cmd_vel   → Commandes de vitesse
# /turtle1/pose      → Position de la tortue
```

### Étape 3 : Contrôler la tortue manuellement

```bash
# Faire avancer la tortue
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 0.0}}"

# Faire tourner la tortue
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

### Étape 4 : Créer un controller pour la tortue

Crée `mon_premier_package/turtle_controller.py` :

```python
#!/usr/bin/env python3
"""
Controller pour la tortue turtlesim.
Fait faire des carrés à la tortue.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TurtleController(Node):
    
    def __init__(self):
        super().__init__('turtle_controller')
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(4.0, self.move_square)
        
        self.step = 0
        self.get_logger().info('Turtle Controller démarré !')
    
    def move_square(self):
        cmd = Twist()
        
        if self.step % 2 == 0:
            # Avance
            cmd.linear.x = 2.0
            cmd.angular.z = 0.0
            self.get_logger().info('Avance...')
        else:
            # Tourne 90°
            cmd.linear.x = 0.0
            cmd.angular.z = 1.57  # ~90° en 1 seconde
            self.get_logger().info('Tourne...')
        
        self.publisher.publish(cmd)
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

N'oublie pas d'ajouter l'entry point dans `setup.py` et de recompiler !

---

## 8.7 Services ROS 2

### Créer un service simple

Les services sont utiles pour les commandes "one-shot" comme reset, spawn, etc.

```python
#!/usr/bin/env python3
"""
Service qui additionne deux nombres.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionService(Node):
    
    def __init__(self):
        super().__init__('addition_service')
        
        # Crée le service
        self.service = self.create_service(
            AddTwoInts,
            'additionner',
            self.additionner_callback
        )
        
        self.get_logger().info('Service prêt !')
    
    def additionner_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AdditionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Appeler le service

```bash
ros2 service call /additionner example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

---

## 8.8 Ressources supplémentaires

- **Tutoriels officiels ROS 2** : https://docs.ros.org/en/humble/Tutorials.html
- **API Python rclpy** : https://docs.ros2.org/latest/api/rclpy/
- **Types de messages** : https://github.com/ros2/common_interfaces

---

# 9. Vérification finale

## 🔍 Script de vérification complète

Exécute ce script pour vérifier que tout est bien installé :

```bash
#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║         VÉRIFICATION SPRINT 0 - Robocar Project               ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

score=0
total=8

echo "═══════════════════════════════════════════════════════════════"
echo "1. Système d'exploitation"
echo "═══════════════════════════════════════════════════════════════"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "   OS : $NAME $VERSION"
else
    echo "   OS : $(uname -s)"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "2. Python"
echo "═══════════════════════════════════════════════════════════════"
if command -v python3 &> /dev/null; then
    version=$(python3 --version)
    echo "   ✅ $version"
    ((score++))
else
    echo "   ❌ Python3 non installé"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "3. Git"
echo "═══════════════════════════════════════════════════════════════"
if command -v git &> /dev/null; then
    version=$(git --version)
    echo "   ✅ $version"
    ((score++))
else
    echo "   ❌ Git non installé"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "4. Webots"
echo "═══════════════════════════════════════════════════════════════"
if command -v webots &> /dev/null; then
    echo "   ✅ Webots installé"
    ((score++))
else
    echo "   ❌ Webots non installé ou pas dans le PATH"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "5. ROS 2"
echo "═══════════════════════════════════════════════════════════════"
if command -v ros2 &> /dev/null; then
    echo "   ✅ ROS 2 installé"
    ((score++))
    
    # Vérifier la version
    distro=$(printenv ROS_DISTRO)
    if [ -n "$distro" ]; then
        echo "   Distribution : $distro"
    fi
else
    echo "   ❌ ROS 2 non installé"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "6. Bridge Webots-ROS2"
echo "═══════════════════════════════════════════════════════════════"
if ros2 pkg list 2>/dev/null | grep -q webots_ros2; then
    echo "   ✅ webots_ros2 installé"
    ((score++))
else
    echo "   ❌ webots_ros2 non installé"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "7. Bibliothèques Python ML"
echo "═══════════════════════════════════════════════════════════════"
ml_ok=true
python3 -c "import gymnasium" 2>/dev/null && echo "   ✅ Gymnasium" || { echo "   ❌ Gymnasium"; ml_ok=false; }
python3 -c "from stable_baselines3 import PPO" 2>/dev/null && echo "   ✅ Stable-Baselines3" || { echo "   ❌ Stable-Baselines3"; ml_ok=false; }
python3 -c "import torch" 2>/dev/null && echo "   ✅ PyTorch" || { echo "   ❌ PyTorch"; ml_ok=false; }
python3 -c "import numpy" 2>/dev/null && echo "   ✅ NumPy" || { echo "   ❌ NumPy"; ml_ok=false; }
python3 -c "import cv2" 2>/dev/null && echo "   ✅ OpenCV" || { echo "   ❌ OpenCV"; ml_ok=false; }

if [ "$ml_ok" = true ]; then
    ((score++))
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "8. Projet Robocar"
echo "═══════════════════════════════════════════════════════════════"
if [ -d "$HOME/robocar_ws/src/robocar_webots_sim" ]; then
    echo "   ✅ Projet cloné"
    ((score++))
    
    if [ -d "$HOME/robocar_ws/install" ]; then
        echo "   ✅ Projet compilé"
        ((score++))
    else
        echo "   ⚠️  Projet non compilé"
    fi
else
    echo "   ❌ Projet non cloné"
    echo "   Exécute : cd ~/robocar_ws/src && git clone https://github.com/Robocar-Paris/robocar_webots_sim.git"
fi
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "RÉSULTAT"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "   Score : $score / $total"
echo ""

if [ $score -eq $total ]; then
    echo "   🎉 FÉLICITATIONS ! Ton environnement est prêt !"
    echo ""
    echo "   Prochaines étapes :"
    echo "   1. Faire le tutoriel Webots (Section 7)"
    echo "   2. Faire le tutoriel ROS 2 (Section 8)"
    echo "   3. Passer au Sprint 1"
elif [ $score -ge 6 ]; then
    echo "   👍 Presque terminé ! Corrige les éléments manquants."
else
    echo "   ⚠️  Plusieurs éléments manquent. Relis la documentation."
fi
echo ""
```

---

## ✅ Checklist du Sprint 0

### Installation

| Étape | Ubuntu | Fedora | Windows | Fait ? |
|-------|--------|--------|---------|--------|
| Mise à jour système | `apt update` | `dnf update` | Windows Update | ☐ |
| Outils de base | `apt install git...` | `dnf install git...` | Chocolatey | ☐ |
| Python 3.10+ | ✅ Inclus | ✅ Inclus | Télécharger | ☐ |
| Webots | APT | Snap | Installeur | ☐ |
| ROS 2 Humble | APT | Snap/Copr | Binaire/WSL | ☐ |
| Bridge webots_ros2 | APT | Compilation | Compilation | ☐ |
| Libs Python ML | pip | pip | pip | ☐ |

### Configuration

| Étape | Fait ? |
|-------|--------|
| Workspace créé (`~/robocar_ws`) | ☐ |
| Projet cloné | ☐ |
| Dépendances installées | ☐ |
| Projet compilé | ☐ |
| `.bashrc` configuré | ☐ |

### Tutoriels

| Tutoriel | Durée | Fait ? |
|----------|-------|--------|
| Webots - Interface | 30 min | ☐ |
| Webots - Premier monde | 30 min | ☐ |
| Webots - Premier controller | 1h | ☐ |
| ROS 2 - Concepts de base | 30 min | ☐ |
| ROS 2 - Talker/Listener | 30 min | ☐ |
| ROS 2 - Premier package | 1h | ☐ |
| ROS 2 - Turtlesim | 1h | ☐ |

---

# 10. Dépannage

## Problèmes courants par OS

### 🐧 Ubuntu

| Problème | Solution |
|----------|----------|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `webots: command not found` | Réinstaller via APT |
| Erreur OpenGL dans Webots | `sudo apt install mesa-utils` |
| `rosdep` échoue | `sudo rosdep init && rosdep update` |

### 🎩 Fedora

| Problème | Solution |
|----------|----------|
| Snap ne fonctionne pas | `sudo systemctl restart snapd` + reboot |
| `ros2: command not found` | Vérifier le source dans `.bashrc` |
| Webots ne démarre pas | Installer les drivers GPU |
| Package manquant | Essayer `dnf search nom_package` |

### 🪟 Windows

| Problème | Solution |
|----------|----------|
| WSL pas installé | `wsl --install` en admin |
| Pas d'affichage graphique | Installer VcXsrv (Windows 10) |
| Python pas dans PATH | Réinstaller en cochant "Add to PATH" |
| `colcon` échoue | Vérifier Visual Studio Build Tools |

## Commandes de diagnostic

```bash
# Vérifier ROS 2
ros2 doctor

# Vérifier l'environnement ROS 2
printenv | grep ROS

# Tester la communication ROS 2
ros2 run demo_nodes_cpp talker &
ros2 topic echo /chatter

# Vérifier Webots
webots --version

# Tester Python
python3 -c "import rclpy; print('rclpy OK')"
```

## Obtenir de l'aide

1. **Consulte la console** : Les erreurs y sont affichées
2. **Cherche sur Google** : Copie-colle le message d'erreur
3. **Forum ROS** : https://discourse.ros.org/
4. **GitHub Issues** : https://github.com/Robocar-Paris/robocar_webots_sim/issues
5. **Demande à l'équipe** : N'hésite pas !

---

# 📚 Récapitulatif

## Ce que tu as appris dans le Sprint 0

1. ✅ Installer Webots, ROS 2 et les outils nécessaires
2. ✅ Comprendre l'interface de Webots
3. ✅ Créer un monde et un controller Webots
4. ✅ Comprendre les concepts ROS 2 (nodes, topics, messages)
5. ✅ Créer un package ROS 2 avec publisher/subscriber
6. ✅ Configurer le projet Robocar

## Prochaine étape : Sprint 1

Tu es maintenant prêt pour le **Sprint 1 : Le Digital Twin** !

Dans le Sprint 1, tu vas :
- Créer le modèle 3D du Robocar dans Webots
- Créer le driver ROS 2 pour contrôler le robot
- Intégrer la caméra

**Bonne continuation !** 🚗💨