#SAMXL setup shell file
#! /bin/bash
set -e # exit script on non-zero return code

SCRIPT_VERSION="2.1.10" # Toggle PREFERRED_REMOTE_DISPLAY and overriding locale
# 2.1.9 Toggle for venv; add local python location to path, nicer help formatting
# 2.1.8 change location of ENV settings from ~/.bashrc to using $DEV_ENV_SETTINGS_FILE
# 2.1.7 ssh key juggling added to compose hacks
# 2.1.6 Toggle for nasty SMACC minimal checkout from old script added
# 2.1.5 Toggle for nasty Docker Compose fixes added
# 2.1.4 ROS2 / ROS1 compatibility
# 2.1.3 dynamically get the location of the default dep files. Removed project name.
# 2.1.2 changed default location of dependency files
# 2.1.1 project name extracted as variable
# 2.1 - ROS2 variant

# Default settings for params
BUILD=true
TARGET_WORKSPACE=""
DEPENDENCIES_WORKSPACE="/home/ros/dependencies_ws"
UNDERLAY_WORKSPACE="/home/ros/underlay_ws"
TARGET_ROS_TYPE=$ROS_VERSION
TARGET_ROS_DISTRO=$ROS_DISTRO

NASTY_FIXES_FOR_COMPOSE=false
SMACC_CLIENT_SUBSET=false
USE_PREFERRED_REMOTE_DISPLAY=false
OVERRIDE_LOCALE=false

# Find location of setup.sh regardless of where called from; default files should be in the same directory
# see https://stackoverflow.com/a/4774063/22510343 for explanation
# shellcheck disable=SC2164
SCRIPTPATH="$( cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 ; pwd -P )"

DEPENDENCIES_FILE="${SCRIPTPATH}/.repos"
PYTHON_REQS_FILE="${SCRIPTPATH}/requirements.txt"
PYTHON_USE_VENV=false
APT_PACKAGES_FILE="${SCRIPTPATH}/.pkglist"


ICI_MODE=false

CI_TARGET_WORKSPACE="/home/ros/target_ws"
CI_DEPENDENCIES_WORKSPACE="/home/ros/upstream_ws"

USAGE="
Usage for setup.sh
v${SCRIPT_VERSION}

setup.sh [-t <path>]

Optional flags: [-binsV] [-h] [-v] [-a <file>] [-d <path>] [-f <file>] [-p <file>] [-r <file>] [-u <path>] [-P] [-l]

Options details:

-t <path>   TARGET_WORKSPACE: where this repo will be built. Binary-available dependencies specified in package.xml will be downloaded by rosdep. 
            Default: ${TARGET_WORKSPACE}

-u <path>   UNDERLAY_WORKSPACE: workspace containing pre-built dependencies (ros packages such as tesseract made available on a docker image)
            Default: ${UNDERLAY_WORKSPACE}

-d <path>   DEPENDENCIES_WORKSPACE: upstream workspace containing dependencies that need to be downloaded and built before this repo can be built. E.g. contents of .repos and manual checkouts.
            Default: ${DEPENDENCIES_WORKSPACE}

-f <file>   DEPENDENCIES_FILE: .repos or .rosinstall file containing upstream dependency locations.
            Default: ${DEPENDENCIES_FILE}

-p <file>   PYTHON_REQS_FILE: requirements.txt file containing python packages to install.
            Default: ${PYTHON_REQS_FILE}

-a <file>   APT_PACKAGES_FILE: file containing apt dependency packages to install (that cannot be handled by rosdep).
            Default: ${APT_PACKAGES_FILE}

-b          BUILD: Build packages. Set this flag to SKIP all build steps (useful for testing/debugging)
            Default: ${BUILD}

-i          ICI MODE: switch to predefined settings for ros_industrial CI server. This will omit certain steps that the ros_industrial CI server handles.

-r          ROS TYPE: which version of ROS to use: 1 or 2. Defaults to what is returned by ROS_VERSION
            Default: ${TARGET_ROS_TYPE}

-n          NASTY_FIXES_FOR_COMPOSE: enable temporary fixes for Docker compose containers. true or false.
            Default: ${NASTY_FIXES_FOR_COMPOSE}

-s          SMACC_CLIENT_SUBSET: enable checkout of specific subset of smacc clients. true or false.
            Default: ${SMACC_CLIENT_SUBSET}

-V          PYTHON_USE_VENV: Install python requirements to a new venv. true or false.
            Default: ${PYTHON_USE_VENV}

-P          USE_PREFERRED_REMOTE_DISPLAY: override the default display variable
            Default: $USE_PREFERRED_REMOTE_DISPLAY

-l          OVERRIDE_LOCALE: override the default display variable
            Default: $OVERRIDE_LOCALE

-v          Print current script version.

-h          Print this help text.

"

while getopts a:bd:f:hilnp:Pr:st:u:Vv flag; do
    case "${flag}" in

        a) APT_PACKAGES_FILE=${OPTARG} ;;
        b) BUILD=false ;;
        d) DEPENDENCIES_WORKSPACE=${OPTARG} ;;
        f) DEPENDENCIES_FILE=${OPTARG} ;;
        i) ICI_MODE=true ;;
        l) OVERRIDE_LOCALE=true ;;
        n) NASTY_FIXES_FOR_COMPOSE=true ;;
        p) PYTHON_REQS_FILE=${OPTARG} ;;
        P) USE_PREFERRED_REMOTE_DISPLAY=true ;;
        r) TARGET_ROS_TYPE=${OPTARG} ;;
        s) SMACC_CLIENT_SUBSET=true ;;
        t) TARGET_WORKSPACE=${OPTARG} ;;
        u) UNDERLAY_WORKSPACE=${OPTARG} ;;
        V) PYTHON_USE_VENV=true ;;
        v)
            echo -e "${SCRIPT_VERSION}"
            exit 0
        ;;
        h)
            echo -e "${USAGE}"
            exit 0
        ;;
        *)
            echo 'Unknown parameter' >&2
            echo 'use -h for usage information'
            exit 1
        ;;
    esac
done

if [[ ${ICI_MODE} == true ]]; then

    TARGET_WORKSPACE=${CI_TARGET_WORKSPACE}
    DEPENDENCIES_WORKSPACE=${CI_DEPENDENCIES_WORKSPACE}
    BUILD=false

fi

# Check if target workspace has been set
if [[ -z $TARGET_WORKSPACE ]]; then

    echo "Target workspace was not provided. Aborting."
    exit 1

fi

if [[ $NASTY_FIXES_FOR_COMPOSE == true ]]; then
    # TODO: remove these nasty fixes for compose
    # Make sure that target workspace has the correct permissions
    sudo chown ros:ros "${TARGET_WORKSPACE}"
    sudo chown ros:ros "${TARGET_WORKSPACE}/src"

    # 'secrets' isn't properly implemented in non-swarm compose, so UID, GID and mode cannot be set.
    # what follows is a horrible way to get working ssh keys in a hopefully non-persistent way
    sudo chown ros:ros /home/ros/.ssh*
    sudo cp /home/ros/.ssh_orig/* /home/ros/.ssh/
    sudo chown -R ros:ros "/home/ros/.ssh"
    sudo chmod 600 /home/ros/.ssh/*

    # Turn off strict host checking & known host comparison
nostrict="
Host *
StrictHostKeyChecking no
UserKnownHostsFile=/dev/null
"
    echo $nostrict >> /home/ros/.ssh/config

    # now actually use the ssh keys from the compose
    git config --global url."git@gitlab.tudelft.nl:".insteadof https://gitlab.tudelft.nl/
fi



####################################################################################
# Workspace configs: move any configuration files to the correct location
####################################################################################

# e.g. Move all files, including hidden/dotfiles from directory to the root of the target workspace
# cp -rT "$TARGET_WORKSPACE"/src/workspace_template/container/config/ "$TARGET_WORKSPACE/"

####################################################################################
# Standalone Dependencies: packages and binaries
####################################################################################
# "${DEV_ENV_SETTINGS_FILE}" is set in the 0.0.2 humble_dev_base dockerfile.
# Default location is "/etc/profile.d/dev_env_tweaks.sh"
if [[ -z ${DEV_ENV_SETTINGS_FILE} ]]; then
    export DEV_ENV_SETTINGS_FILE="/etc/profile.d/dev_env_tweaks.sh"
    sudo touch ${DEV_ENV_SETTINGS_FILE}
    sudo chown ros:ros ${DEV_ENV_SETTINGS_FILE}
fi

if [[ $USE_PREFERRED_REMOTE_DISPLAY == true ]]; then
    echo "export DISPLAY=$PREFERRED_REMOTE_DISPLAY" >> "${DEV_ENV_SETTINGS_FILE}"
fi

if [[ $OVERRIDE_LOCALE == true ]]; then
    echo "export LANG=C" >> "${DEV_ENV_SETTINGS_FILE}"
fi

###############################
# apt packages
sudo apt-get update -qq


if [[ ! -f $APT_PACKAGES_FILE ]]; then

    echo "Specified APT_PACKAGES_FILE not found. Aborting."
    echo "${APT_PACKAGES_FILE}"
    exit 1

fi


if [[ ! -f $APT_PACKAGES_FILE ]]; then

    echo "Specified APT_PACKAGES_FILE not found. Aborting."
    echo "${APT_PACKAGES_FILE}"
    exit 1

fi

# install all packages in the APT_PACKAGES_FILE.
# grep will strip any lines starting with # (comments)
# -r will prevent the apt-get call from being run at all if the file does not contain any non-whitespace characters.

grep -e '^[^#]' "${APT_PACKAGES_FILE}" | xargs -r sudo apt-get install -y -qq --no-install-recommends

###############################
# Install Aravis
###############################

echo "Installing Aravis 0.8.30..."
ARAVIS_VERSION="0.8.30"
wget https://github.com/AravisProject/aravis/releases/download/${ARAVIS_VERSION}/aravis-${ARAVIS_VERSION}.tar.xz && \
    tar xfJ aravis-${ARAVIS_VERSION}.tar.xz && \
    rm aravis-${ARAVIS_VERSION}.tar.xz

cd aravis-${ARAVIS_VERSION} && \
    meson setup build && \
    cd build && \
    ninja && \
    sudo ninja install && \
    sudo ldconfig && \
    cd ../.. && \
    rm -rf aravis-${ARAVIS_VERSION}

###############################
# Install ScanCONTROL Linux SDK
###############################
echo "Installing ScanCONTROL Linux SDK 1.0.0..."
SCANCONTROL_VERSION="1-0-0"
wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-${SCANCONTROL_VERSION}.zip -O scanCONTROL-Linux-SDK.zip && \
    unzip scanCONTROL-Linux-SDK.zip -d scanCONTROL-Linux-SDK/ && \
    rm scanCONTROL-Linux-SDK.zip

cd scanCONTROL-Linux-SDK/libmescan/ && \
    meson builddir && \
    cd builddir && \
    ninja && \
    sudo ninja install && \
    sudo ldconfig && \
    cd ../../..

cd scanCONTROL-Linux-SDK/libllt/include && \
    wget -q https://raw.githubusercontent.com/Pugens/scancontrol/1105de0ea8a28526b03de488d76821e07bada265/micro_epsilon_scancontrol_driver/include/lltlib/llt.h -O llt.h && \
    cd .. && \
    meson builddir && \
    cd builddir && \
    ninja && \
    sudo ninja install && \
    sudo ldconfig && \
    cd ../../..

echo "Aravis and ScanCONTROL Linux SDK installation completed!"

###############################
# update the rosdep database
rosdep -q update  --rosdistro "${TARGET_ROS_DISTRO}"

###############################
# Python packages

if [[ ! -f $PYTHON_REQS_FILE ]]; then

    echo "Specified PYTHON_REQS_FILE not found. Aborting."
    echo "${PYTHON_REQS_FILE}"
    exit 1

fi

# Default image doesn't have .local/bin on the path, so add it
export PATH="/home/ros/.local/bin:$PATH"
echo "export PATH=\"/home/ros/.local/bin:\$PATH\"" >> "${DEV_ENV_SETTINGS_FILE}"

# If true, install the requirements to a venv
if [[ ${PYTHON_USE_VENV} == true ]]; then

    # Remove previous existing venv
    if [[ -d "${TARGET_WORKSPACE}"/venv ]]; then
        rm -rf "${TARGET_WORKSPACE}"/venv
    fi

    virtualenv "${TARGET_WORKSPACE}"/venv
    # shellcheck source=/dev/null
    source "${TARGET_WORKSPACE}"/venv/bin/activate

fi

pip3 install -q -r "${PYTHON_REQS_FILE}"

###############################
# ccache
# improve build speeds on subsequent catkin builds
# not needed in CI mode; handled automatically by industrial_ci (see gitlab-ci.yml)
if [[ ${ICI_MODE} == false ]]; then

    if [[ $(apt -qq list --installed ccache) == ""  ]]; then
        sudo apt install ccache -qq -y
        export PATH="/usr/lib/ccache:$PATH"

        echo "export PATH=\"/usr/lib/ccache:\$PATH\"" >> "${DEV_ENV_SETTINGS_FILE}"
    fi
    export CCACHE_DIR="${TARGET_WORKSPACE}/.ccache"

    # create env vars for ccache that will persist outside this script
    echo "export CCACHE_DIR=\"$CCACHE_DIR\"" >> "${DEV_ENV_SETTINGS_FILE}"

fi #ICI_MODE


####################################################################################
# Upstream Dependencies: packages that must be built
####################################################################################

mkdir -p "${DEPENDENCIES_WORKSPACE}"/src || { echo "Problem with DEPENDENCIES_WORKSPACE. Aborting."; exit 1; }
cd "${DEPENDENCIES_WORKSPACE}"/src || { echo "Problem with DEPENDENCIES_WORKSPACE. Aborting."; exit 1; }


################################
# SMACC clients
# SMACC is a mono repo with many packages; we only want a few targets. vcs doesn't support sparse checkouts, so we must do it manually

if [[ $SMACC_CLIENT_SUBSET == true ]]; then

    "$SCRIPTPATH"/smacc_fix.sh -d "${DEPENDENCIES_WORKSPACE}"

fi

################################
# Standard .rosinstall imports from the target repo
# In CI mode, handled by industrial_ci

if [[ ${ICI_MODE} == false ]]; then

    if [[ ! -f $DEPENDENCIES_FILE ]]; then

        echo "Specified DEPENDENCIES_FILE not found. Aborting."
        echo "${DEPENDENCIES_FILE}"
        echo "${DEPENDENCIES_FILE}"
        exit 1

    fi

    vcs import --shallow < "${DEPENDENCIES_FILE}" "${DEPENDENCIES_WORKSPACE}"/src/

    if [[ ${BUILD} == true ]]; then

        cd "${DEPENDENCIES_WORKSPACE}"  || { echo "Problem with DEPENDENCIES_WORKSPACE. Aborting."; exit 1; }

        if [[ $TARGET_ROS_TYPE == 2 ]]; then
            # shellcheck source=/dev/null
            [ -f "${UNDERLAY_WORKSPACE}/install/setup.bash" ]  && source "${UNDERLAY_WORKSPACE}/install/setup.bash"
        else
            # TODO: ROS1 can use install or devel...
            # shellcheck source=/dev/null
            [ -f "${UNDERLAY_WORKSPACE}/devel/setup.bash" ]  && source "${UNDERLAY_WORKSPACE}/devel/setup.bash"

        fi

        rosdep -q install --from-paths "${DEPENDENCIES_WORKSPACE}"/src --ignore-src -y --rosdistro "${TARGET_ROS_DISTRO}"

        if [[ $TARGET_ROS_TYPE == 2 ]]; then
            # merge install: make one install dir with everything inside. Mandatory if Gazebo build is needed.
            # symlink-install: make symlinks from src to install instead of copying.
            # TODO: symlink-install causes issue in CI. See https://gitlab.tudelft.nl/samxl/toolbox/workspace_template/-/issues/1
            colcon build --parallel-workers 3 --symlink-install --merge-install
        else
            catkin build -c
        fi

    fi # BUILD

    ###########################################################################################
    # Target workspace
    # This segment is not run in CI mode by default; these parts are handled by the ros_industrial scripts.
    ###########################################################################################

    if [[ ${BUILD} == true ]]; then

        cd "${TARGET_WORKSPACE}"  || { echo "Problem with TARGET_WORKSPACE. Aborting."; exit 1; }

        if [[ $TARGET_ROS_TYPE == 2 ]]; then
            # shellcheck source=/dev/null
            source "${DEPENDENCIES_WORKSPACE}/install/setup.bash"
        else
            # TODO: ROS1 can use install or devel...
            # shellcheck source=/dev/null
            source "${DEPENDENCIES_WORKSPACE}/devel/setup.bash"
        fi

        rosdep -q install --from-paths "${TARGET_WORKSPACE}"/src --ignore-src -y

        if [[ $TARGET_ROS_TYPE == 2 ]]; then
            # merge install: make one install dir with everything inside. Mandatory if Gazebo build is needed.
            # symlink-install: make symlinks from src to install instead of copying.
            # TODO: symlink-install causes issue in CI. See https://gitlab.tudelft.nl/samxl/toolbox/workspace_template/-/issues/1
            colcon build --parallel-workers 3 --symlink-install --merge-install
        else
            catkin build -c
        fi

        if [[ $TARGET_ROS_TYPE == 2 ]]; then
            # shellcheck source=/dev/null
            source "${TARGET_WORKSPACE}/install/setup.bash"
        else
            # TODO: ROS1 can use install or devel...
            # shellcheck source=/dev/null
            source "${TARGET_WORKSPACE}/devel/setup.bash"
        fi

    fi # BUILD

    echo "Please source $DEV_ENV_SETTINGS_FILE in this shell to get updated paths and env vars (They will be applied automatically in new shells)."

fi # ICI_MODE
