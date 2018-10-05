# Stop all running containers
docker stop $(docker ps -aq)

xhost +local:root

# Run container
nvidia-docker run -it --rm \
    --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/tmp/.XIM-unix:/tmp/.XIM-unix:rw" \
    -p 5900:5900 -p 5554:5554 -p 5555:5555 -p 5557:5557 -p 5558:5558 -p 5559:5559 \
    sandeepgogadi/jderobot-docker-ros
