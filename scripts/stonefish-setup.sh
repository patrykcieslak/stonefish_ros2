#!/bin/bash
set -e

# ----------------------------- GLOBAL VARIABLES -----------------------------
STONEFISH_DIR="$HOME/opt/stonefish"
LOG_PREFIX="[$(date +%T)]"

# ----------------------------- HELPER FUNCTIONS -----------------------------
log_info() {
    echo -e "$LOG_PREFIX [INFO] $1"
}

log_error() {
    echo -e "$LOG_PREFIX [ERROR] $1" >&2
}

# ----------------------------- C++ DEPENDENCIES -----------------------------
install_cpp_dependencies() {
    log_info "Installing required C++ dependencies..."
    sudo apt-get update -qq
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        libglm-dev \
        libsdl2-dev \
        libfreetype6-dev
    log_info "C++ dependencies installed."
}

install_gcc13_compiler() {
    log_info "Installing GCC 13 compiler..."
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends software-properties-common
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
    sudo apt-get update -qq

    sudo apt-get install -y --no-install-recommends gcc-13 g++-13
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
}

# ----------------------------- STONEFISH INSTALLATION -----------------------------
install_stonefish() {
    if [ -d "$STONEFISH_DIR" ]; then
        log_info "Stonefish is already installed at $STONEFISH_DIR. Skipping clone."
    else
        log_info "Cloning Stonefish repository..."
        mkdir -p "$STONEFISH_DIR"
        git clone https://github.com/vortexntnu/stonefish.git "$STONEFISH_DIR"
        sed -i '30i#include <cstdint>' /github/home/opt/stonefish/Library/include/sensors/Sample.h
    fi

    log_info "Building Stonefish..."
    mkdir -p "$STONEFISH_DIR/build"
    cd "$STONEFISH_DIR/build"

    cmake ..
    make -j"$(nproc)"
    sudo make install

    log_info "Stonefish installation complete."
}

# ----------------------------- EXECUTE INSTALLATION -----------------------------
log_info "Starting manual installation of extra dependencies..."
install_cpp_dependencies
install_gcc13_compiler
install_stonefish

log_info "All dependencies installed successfully."