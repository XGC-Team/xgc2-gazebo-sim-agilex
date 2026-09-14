FROM gazebo@sha256:eba35c7ca3198daa68edf71b4732fc42668ff43d9bfa712d66402c66d3628305
ENV DEBIAN_FRONTEND=noninteractive
# The base is the official gzserver11 image, not a ROS or product runtime.
RUN sed -i 's|http://packages.osrfoundation.org|https://packages.osrfoundation.org|g' /etc/apt/sources.list.d/gazebo-latest.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends build-essential cmake pkg-config libyaml-cpp-dev libgazebo11-dev=11.15.1-1* \
 && test ! -d /opt/ros \
 && rm -rf /var/lib/apt/lists/*
COPY scout/include/scout_gazebo/command_delay.hpp /src/scout/include/scout_gazebo/command_delay.hpp
COPY scout/tools/native_replay /src/scout/tools/native_replay
RUN cmake -S /src/scout/tools/native_replay -B /build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /build -j2 \
 && dpkg-query -W > /build/dependency-packages.txt \
 && sha256sum /build/scout_native_replay > /build/binary-sha256.txt
ENV GAZEBO_MODEL_DATABASE_URI=""
ENTRYPOINT ["/build/scout_native_replay"]
