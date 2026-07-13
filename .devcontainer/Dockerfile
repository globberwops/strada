FROM ubuntu:26.04 AS base

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
    git \
    doxygen \
    fd-find \
    g++ \
    gcovr \
    gdb \
    graphviz \
    iwyu \
    lcov \
    libgmock-dev \
    libgtest-dev \
    libpugixml-dev \
    libxml2-utils \
    lldb \
    ninja-build \
    nodejs \
    npm \
    pkg-config \
    python3-full \
    python3-pip \
    qt6-base-dev \
    qt6-wayland \
    ripgrep \
    uncrustify \
    valgrind \
    shellcheck \
    shfmt \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install python-based linting/formatting tools
RUN pip3 install --break-system-packages --no-cache-dir cmakelang cpplint gcovr pre-commit

# ==============================================================================
# CI Target (Optimized for CI runner, keeps image lightweight)
# ==============================================================================
FROM base AS ci

# Any CI-specific packages or variables can go here.
# (Currently, CI uses root and only requires the core build and static tools)

# ==============================================================================
# Development Target (Customized for local developer setup)
# ==============================================================================
FROM base AS dev

# Switch to the default ubuntu user to handle user-space configurations and CLI installations
USER ubuntu
ENV HOME=/home/ubuntu

# Create a local bin directory if it doesn't exist, and fix the Ubuntu fdfind -> fd naming issue
RUN mkdir -p ${HOME}/.local/bin \
    && ln -s $(which fdfind) ${HOME}/.local/bin/fd

# Ensure user-local binaries (including agy and our fd symlink) are explicitly on the PATH
ENV PATH="${HOME}/.local/bin:${PATH}"

# Install the native Herdr installation directly into the user space
RUN curl -fsSL https://herdr.dev/install.sh | sh

# Run the native Google Antigravity installation directly into the user space
RUN curl -fsSL https://antigravity.google/cli/install.sh | bash

# Run the native Pi installation directly into the user space
RUN curl -fsSL https://pi.dev/install.sh | sh

# Trigger devcontainer rebuild for shfmt and shellcheck
