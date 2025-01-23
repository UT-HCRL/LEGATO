# Use NVIDIA's official CUDA image with Ubuntu as the base
FROM nvidia/cuda:12.1.1-base-ubuntu22.04

# Set the working directory
WORKDIR /workspace

# Install required dependencies for building Python
RUN apt update && apt install -y --no-install-recommends \
    build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev libsqlite3-dev wget libbz2-dev lzma liblzma-dev \
    git curl cmake ffmpeg libsm6 libxext6 libgl1-mesa-glx\
    libglfw3-dev libgles2-mesa-dev \
    coreutils \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Download and build Python 3.9.1
RUN wget https://www.python.org/ftp/python/3.9.2/Python-3.9.2.tgz && \
    tar -xzf Python-3.9.2.tgz && \
    cd Python-3.9.2 && \
    ./configure --enable-optimizations && \
    make -j$(nproc) && \
    make altinstall && \
    cd .. && \
    rm -rf Python-3.9.2 Python-3.9.2.tgz

# Set Python 3.9 as the default Python version
RUN ln -sf /usr/local/bin/python3.9 /usr/bin/python && ln -sf /usr/local/bin/python3.9 /usr/bin/python3

# Install pip for Python 3.9
# RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python && \
#     python -m pip install --no-cache-dir "pip<24.1"
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python

# Clone the LEGATO repository
# RUN git clone https://github.com/UT-HCRL/LEGATO.git /workspace/LEGATO

# Set the working directory to the LEGATO folder
# WORKDIR /workspace/LEGATO
COPY requirements.txt .

# Install Python dependencies from the project's requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# RUN git clone https://github.com/ARISE-Initiative/robosuite.git
# WORKDIR /workspace/robosuite
# RUN pip install --no-cache-dir -r requirements.txt

# COPY files
WORKDIR /workspace/LEGATO
COPY . .

# Expose the ports needed by the application
EXPOSE 8888

# Default command to keep the container running
CMD ["bash"]
