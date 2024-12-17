###############################
# Install Aravis
###############################

echo "Installing Aravis 0.8.30..."
ARAVIS_VERSION="0.8.30"
sudo wget https://github.com/AravisProject/aravis/releases/download/${ARAVIS_VERSION}/aravis-${ARAVIS_VERSION}.tar.xz && \
    sudo tar xfJ aravis-${ARAVIS_VERSION}.tar.xz && \
    sudo rm aravis-${ARAVIS_VERSION}.tar.xz

cd aravis-${ARAVIS_VERSION} && \
    sudo meson setup build && \
    cd build && \
    sudo ninja && \
    sudo ninja install && \
    sudo ldconfig && \
    cd ../.. && \
    sudo rm -rf aravis-${ARAVIS_VERSION}

###############################
# Install ScanCONTROL Linux SDK
###############################
echo "Installing ScanCONTROL Linux SDK 1.0.0..."
SCANCONTROL_VERSION="1-0-0"
sudo wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-${SCANCONTROL_VERSION}.zip -O scanCONTROL-Linux-SDK.zip && \
    sudo unzip scanCONTROL-Linux-SDK.zip -d scanCONTROL-Linux-SDK/ && \
    sudo rm scanCONTROL-Linux-SDK.zip

cd scanCONTROL-Linux-SDK/libmescan/ && \
    sudo meson builddir && \
    cd builddir && \
    sudo ninja && \
    sudo ninja install && \
    sudo ldconfig && \
    cd ../../..

cd scanCONTROL-Linux-SDK/libllt/include && \
    sudo wget -q https://raw.githubusercontent.com/Pugens/scancontrol/1105de0ea8a28526b03de488d76821e07bada265/micro_epsilon_scancontrol_driver/include/lltlib/llt.h -O llt.h && \
    cd .. && \
    sudo meson builddir && \
    cd builddir && \
    sudo ninja && \
    sudo ninja install && \
    sudo ldconfig && \
    cd ../../..

echo "Aravis and ScanCONTROL Linux SDK installation completed!"
