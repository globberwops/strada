FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive

# Install core build engines, compilers, utilities, static analysis tools, and dynamic verification suites
RUN apt-get update && apt-get install -y --no-install-recommends \
    binutils \
    build-essential \
    ca-certificates \
    ccache \
    clang \
    clang-format \
    clang-tidy \
    clang-tools \
    clangd \
    cmake \
    cppcheck \
    curl \
    doxygen \
    fd-find \
    g++ \
    gdb \
    graphviz \
    lcov \
    libgmock-dev \
    libgtest-dev \
    libpugixml-dev \
    libxml2-utils \
    lldb \
    ninja-build \
    pkg-config \
    python3-full \
    python3-pip \
    qt6-base-dev \
    qt6-wayland \
    ripgrep \
    valgrind \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install python-based linting/formatting tools
RUN pip3 install --break-system-packages --no-cache-dir cmakelang gcovr

# Switch to the default ubuntu user to handle user-space configurations and CLI installations
USER ubuntu
ENV HOME=/home/ubuntu

# Create a local bin directory if it doesn't exist, and fix the Ubuntu fdfind -> fd naming issue
RUN mkdir -p ${HOME}/.local/bin \
    && ln -s $(which fdfind) ${HOME}/.local/bin/fd

# Ensure user-local binaries (including agy and our fd symlink) are explicitly on the PATH
ENV PATH="${HOME}/.local/bin:${PATH}"

# Run the native Google Antigravity installation directly into the user space
# The script defaults to extracting the binary to ~/.local/bin/agy
RUN curl -fsSL https://antigravity.google/cli/install.sh | bash

# RUN curl -fsSL https://pi.dev/install.sh | sh
# RUN pi install npm:pi-intercom \
#     && pi install npm:pi-lens \
#     && pi install npm:pi-subagents \
#     && pi install npm:pi-web-access
