# ==================== vsis_video_service 开发环境 ====================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Asia/Shanghai

# apt 源开关: APT_MIRROR=none 保持官方源 —— 狗上流量卡实测 ports.ubuntu.com ~5MB/s, 清华源反而 403
#            APT_MIRROR=tuna 换清华源 —— 办公室/家宽网络在 x86 上 build 时用
ARG APT_MIRROR=none
RUN apt-get update && \
    apt-get install -y ca-certificates unzip && \
    if [ "$APT_MIRROR" = "tuna" ]; then \
      sed -i -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
             -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
             -e 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu/ubuntu-ports|g' \
             /etc/apt/sources.list; \
    fi && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    echo $TZ > /etc/timezone && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    gdb \
    git \
    nano \
    libsqlite3-dev \
    libopencv-dev \
    ffmpeg \
    curl \
    libssl-dev \
    iputils-ping \
    net-tools \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# 设置工作目录并拷贝源码
WORKDIR /vsis_video_service
COPY . /vsis_video_service

RUN chmod +x /vsis_video_service/start.sh

# 默认启动命令
CMD ["/bin/bash", "/vsis_video_service/start.sh"]
