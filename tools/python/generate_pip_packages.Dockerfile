FROM debian:bookworm

# Install dependencies of the pip packages that we're compiling.
RUN apt-get update && apt-get install -y \
    cmake \
    make \
    git \
    curl \
    clang-13 \
    libcairo2-dev \
    libgirepository1.0-dev \
    libglib2.0-0 \
    libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

# Get latest patchelf for auditwheel.
RUN curl -L https://github.com/NixOS/patchelf/releases/download/0.15.0/patchelf-0.15.0-x86_64.tar.gz > /tmp/patchelf.tar.gz \
    && tar -xaf /tmp/patchelf.tar.gz -C /usr \
    && rm -f /tmp/patchelf.tar.gz

# Get the same Python versions that we're using for actually running Python code.
RUN mkdir -p /opt/python3.10 && \
    curl -SL https://github.com/astral-sh/python-build-standalone/releases/download/20240107/cpython-3.10.13+20240107-x86_64-unknown-linux-gnu-install_only.tar.gz \
    | tar -xz -C /opt/python3.10 --strip-components=1

RUN mkdir -p /opt/python3.13 && \
    curl -SL https://github.com/astral-sh/python-build-standalone/releases/download/20250610/cpython-3.13.4+20250610-x86_64-unknown-linux-gnu-install_only_stripped.tar.gz \
    | tar -xz -C /opt/python3.13 --strip-components=1

# Make some symlinks to satisfy assumptions some of the installer scripts (e.g.
# setup.py files) make about the system.
# /opt/python and /install are linked dynamically inside the build script.
RUN ln -s /usr/bin/clang-13 /usr/bin/clang && \
    ln -s /usr/bin/clang++-13 /usr/bin/clang++
