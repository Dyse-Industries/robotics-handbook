# =============================================================================
# Configure Build: Set parameters and tools for the build
# =============================================================================
ARG BASE_IMAGE=rust:1.90-slim-bullseye
FROM ${BASE_IMAGE}

ENV USERNAME=mdyson
ENV USER_UID=1000
ENV USER_GID=1000
ENV BASE_IMAGE=${BASE_IMAGE}
ENV PROJECT_NAME=robotics-handbook

ENV SHELL=/bin/bash
ENV PROJECT_NAME=${PROJECT_NAME}
ENV USERNAME=mdyson
ENV NODE_VERSION=22.0.0

# =============================================================================
# Development Tools: Essential utilities for development
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    vim \
    curl \
    wget \
    pkg-config \
    libssl-dev \
    ca-certificates \
    build-essential \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# User Configuration: Set up workspace
# =============================================================================
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN echo "source /home/${USERNAME}/${PROJECT_NAME}/scripts/bash_prompt.bash" >> /home/${USERNAME}/.bashrc

COPY ./ /home/${USERNAME}/${PROJECT_NAME}

# =============================================================================
# Project Configuration: instal dependencies
# =============================================================================

# Install Node.js for Prettier formatting
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash - \
    && sudo apt-get install -y nodejs

# install cargo deps
# RUN /home/${USERNAME}/${PROJECT_NAME}/scripts/install_rust.bash
RUN cargo install mdbook --vers 0.4.52 --force && \
    cargo install mdbook-katex --vers 0.9.4 --force && \
    cargo install mdbook-mermaid --vers 0.16.2 --force && \
    mdbook-mermaid install /home/${USERNAME}/${PROJECT_NAME}

WORKDIR /home/${USERNAME}/${PROJECT_NAME}
CMD ["/bin/bash"]
