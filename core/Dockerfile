FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates cmake g++ python3 python3-dev && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY CMakeLists.txt ./
COPY core/ core/
COPY bindings/ bindings/
COPY scenarios/ scenarios/
COPY tools/ tools/
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DDRIVEBENCH_PYTHON=ON \
    && cmake --build build --parallel 2
ENV PYTHONPATH=/workspace/build/bindings
CMD ["ctest", "--test-dir", "build", "--output-on-failure"]