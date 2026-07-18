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
    doxygen \
    fd-find \
    g++ \
    gcovr \
    gdb \
    git \
    graphviz \
    iwyu \
    lcov \
    libbenchmark-dev \
    libclang-21-dev \
    libgmock-dev \
    libgtest-dev \
    libpugixml-dev \
    libxml2-utils \
    lldb \
    llvm-21-dev \
    ninja-build \
    nodejs \
    npm \
    pkg-config \
    python3-full \
    python3-pip \
    qt6-base-dev \
    qt6-wayland \
    ripgrep \
    shellcheck \
    shfmt \
    uncrustify \
    valgrind \
    wl-clipboard \
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

# Install Herdr
RUN curl -fsSL https://herdr.dev/install.sh | sh

# Install omp Agent
RUN curl -fsSL https://omp.sh/install | sh
