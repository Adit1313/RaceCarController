FROM espressif/idf:v5.1.1

# Install protocol buffers
RUN apt-get update && apt-get install -y protobuf-compiler python3-pip

# Install pre-commit and requirements for building protobuf messages
RUN . $IDF_PATH/export.sh && pip install protobuf grpcio-tools

# Pre-install the hooks that pre-commit needs. The contents of the .git/hooks
# folder will get overwritten by the volume mount, but the executables for the
# hooks will be there!
RUN pip install pre-commit
COPY .pre-commit-config.yaml /code/.pre-commit-config.yaml
WORKDIR /code
RUN git init . && pre-commit install-hooks

# Make sure idf.py is added to PATH and ignore output
RUN echo "source /opt/esp/idf/export.sh > /dev/null" >> /root/.bashrc
