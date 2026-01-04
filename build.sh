#!/bin/bash

set -e  # Arrête si une commande échoue

# Couleurs
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Variables
WORKSPACE="$HOME/robocar_ws"
CLEAN=false
SYMLINK=false
PACKAGE=""

# Parser les arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean|-c)
            CLEAN=true
            shift
            ;;
        --symlink|-s)
            SYMLINK=true
            shift
            ;;
        --help|-h)
            echo "Usage: ./build.sh [options] [package_name]"
            echo ""
            echo "Options:"
            echo "  --clean, -c     Nettoie avant de compiler"
            echo "  --symlink, -s   Utilise des liens symboliques (dev mode)"
            echo "  --help, -h      Affiche cette aide"
            echo ""
            echo "Exemples:"
            echo "  ./build.sh                    # Compile tout"
            echo "  ./build.sh robocar_driver     # Compile un package"
            echo "  ./build.sh --clean            # Nettoie et recompile"
            echo "  ./build.sh -s robocar_driver  # Dev mode pour un package"
            exit 0
            ;;
        *)
            PACKAGE=$1
            shift
            ;;
    esac
done

# Vérifier que le workspace existe
if [ ! -d "$WORKSPACE/src" ]; then
    echo -e "${RED}❌ Workspace non trouvé : $WORKSPACE${NC}"
    echo "Créez-le avec :"
    echo "  mkdir -p ~/robocar_ws/src"
    echo "  cd ~/robocar_ws/src"
    echo "  ln -s /chemin/vers/robocar_webots_sim ."
    exit 1
fi

# Aller dans le workspace
cd "$WORKSPACE"

# Sourcer ROS 2 si pas déjà fait
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}❌ ROS 2 non trouvé${NC}"
        exit 1
    fi
fi
echo -e "  ${GREEN}✅${NC} ROS 2 $ROS_DISTRO"

# Nettoyer si demandé
if [ "$CLEAN" = true ]; then
    echo -e "\n${YELLOW}🧹 Nettoyage des dossiers de compilation...${NC}"
    rm -rf build/ install/ log/
    echo -e "  ${GREEN}✅${NC} Nettoyé"
fi

# Construire les arguments colcon
COLCON_ARGS=""

if [ "$SYMLINK" = true ]; then
    COLCON_ARGS="$COLCON_ARGS --symlink-install"
    echo -e "  ${GREEN}✅${NC} Mode symlink activé"
fi

if [ -n "$PACKAGE" ]; then
    COLCON_ARGS="$COLCON_ARGS --packages-select $PACKAGE"
    echo -e "  ${GREEN}✅${NC} Package : $PACKAGE"
else
    echo -e "  ${GREEN}✅${NC} Compilation de tous les packages"
fi

# Compiler
echo -e "\n${BLUE}🔨 Compilation en cours...${NC}\n"

colcon build $COLCON_ARGS

# Sourcer le résultat
source install/setup.bash

# Afficher les packages compilés
echo "Packages disponibles :"
ros2 pkg list 2>/dev/null | grep -E "robocar" | while read pkg; do
    echo -e "  • $pkg"
done

echo ""
echo "Prochaine étape : ros2 run robocar_driver <node>"
echo ""