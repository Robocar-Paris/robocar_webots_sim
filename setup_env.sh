#!/bin/bash

# Couleurs pour les messages
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. Détecter la version de ROS 2 installée
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "  ${GREEN}✅${NC} ROS 2 Jazzy activé"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "  ${GREEN}✅${NC} ROS 2 Humble activé"
else
    echo -e "  ${YELLOW}⚠️  ROS 2 non trouvé dans /opt/ros/${NC}"
    echo -e "     Installez ROS 2 ou vérifiez le chemin"
fi

# 2. Workspace Robocar
if [ -f "$HOME/robocar_ws/install/setup.bash" ]; then
    source ~/robocar_ws/install/setup.bash
    echo -e "  ${GREEN}✅${NC} Workspace Robocar activé"
else
    echo -e "  ${YELLOW}⚠️  Workspace non compilé${NC}"
    echo -e "     Lancez : cd ~/robocar_ws && colcon build"
fi

# 3. Webots
if [ -d "/usr/local/webots" ]; then
    export WEBOTS_HOME=/usr/local/webots
    echo -e "  ${GREEN}✅${NC} WEBOTS_HOME configuré (/usr/local/webots)"
elif [ -d "/snap/webots/current/usr/share/webots" ]; then
    export WEBOTS_HOME=/snap/webots/current/usr/share/webots
    echo -e "  ${GREEN}✅${NC} WEBOTS_HOME configuré (Snap)"
else
    echo -e "  ${YELLOW}⚠️  Webots non trouvé${NC}"
fi

# 4. Environnement Python IA (optionnel)
if [ "$1" == "--ai" ] || [ "$1" == "-a" ]; then
    if [ -f "$HOME/ros2_ai_env/bin/activate" ]; then
        source ~/ros2_ai_env/bin/activate
        echo -e "  ${GREEN}✅${NC} Environnement Python IA activé"
    else
        echo -e "  ${YELLOW}⚠️  Environnement IA non trouvé (~ros2_ai_env)${NC}"
    fi
fi

echo ""
echo -e "${GREEN}🎉 Environnement prêt !${NC}"
echo ""
echo "Commandes utiles :"
echo "  ros2 run robocar_driver <node>   - Lancer un node"
echo "  ros2 topic list                  - Lister les topics"
echo "  ros2 launch <pkg> <launch.py>    - Lancer un launch file"
echo "  webots                           - Ouvrir Webots"
echo "  ./build.sh                       - Recompiler le projet"
echo ""