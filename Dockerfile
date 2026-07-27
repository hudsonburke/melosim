# melosim round-trip environment
# Build:  docker build -t melosim .
# Run:    docker run --rm -v $(pwd):/workspace melosim roundtrip /workspace/model.osim
#
# This image has everything needed to run the PyO3 importer + Rust exporter:
#   - Python 3.13 with opensim package
#   - Rust toolchain (cargo)
#   - The melosim source

FROM python:3.13-slim-bookworm

# Install system deps that OpenSim wheels need
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    pkg-config \
    libssl-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    libx11-6 \
    libxi6 \
    libxmu6 \
    libxt6 \
    libfreetype6 \
    libfontconfig1 \
    libxcursor1 \
    libxrandr2 \
    libxinerama1 \
    && rm -rf /var/lib/apt/lists/*

# Install OpenSim Python package
RUN pip install --no-cache-dir opensim

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /workspace

# Default: show help
CMD ["cargo", "run", "--bin", "roundtrip", "--help"]
