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
echo "Installing apt packages..."
grep -e '^[^#]' "${APT_PACKAGES_FILE}" | xargs -r sudo apt-get install -y --no-install-recommends

###############################
# update the rosdep database
rosdep -v update  --rosdistro "${TARGET_ROS_DISTRO}"

###############################
# Python packages
echo "Installing python packages..."
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

pip3 install -v -r "${PYTHON_REQS_FILE}"