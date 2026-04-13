FROM python:3.11-slim

# Dependências do sistema
RUN apt-get update && apt-get install -y \
    git \
    curl \
    gcc \
    g++ \
    make \
    cmake \
    libssl-dev \
    libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# Instala o PlatformIO CLI
RUN pip install platformio
FROM python:3.11-slim

# Dependências do sistema (inclui git para clonar a plataforma do maxgerhardt)
RUN apt-get update && apt-get install -y \
    git \
    curl \
    gcc \
    g++ \
    make \
    cmake \
    libssl-dev \
    libffi-dev \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Instala PlatformIO CLI
RUN pip install platformio

# Cria diretório de trabalho
WORKDIR /firmware

# Copia apenas o platformio.ini primeiro (para cachear o download das deps)
COPY firmware/MotionControllerRP/platformio.ini .

# Pré-baixa a plataforma e dependências (camada cacheada separadamente)
RUN platformio pkg install

# Copia o restante do código-fonte
COPY firmware/MotionControllerRP/ .

# Compila
CMD ["platformio", "run"]
# Diretório de trabalho
WORKDIR /firmware

# Copia o firmware para dentro do container
COPY firmware/MotionControllerRP/ .

# Faz o build ao iniciar o container
CMD ["platformio", "run"]