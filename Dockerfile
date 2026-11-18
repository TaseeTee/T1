FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --uid 10001 --create-home participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /opt/participant
COPY manifest.py ./manifest.py
COPY tasks/ ./tasks/
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh

USER participant
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
