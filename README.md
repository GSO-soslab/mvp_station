# vx_comms
ROS driver for vehicle communication. <br> The `vx_script.py` deals with converting ros msgs into mavlink msgs and sends across the desired transport(Both RF and UDP are supported). <br>
The `gcs_script.py` receives MAVLink msgs and sends the necesary parsed messages to its clients via sockets.<br>
`client.py` is the socket client.
