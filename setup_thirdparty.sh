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
