sudo docker run --privileged \
           -it --init --ipc=host --network=host --volume=$PWD:/app \
           -v "/tmp/.X11-unix:/tmp/.X11-unix" \
           -v "/home/taehoon/lecture:/workspace" \
           -e DISPLAY \
           -e QT_X11_NO_MITSHM=1 \
	   --workdir=/workspace \
	   --name dataset_ra_labeler \
	   point_labeler:v3

