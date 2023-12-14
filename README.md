# mvp_station
Scalable Robot Dashboard & GroundStation built on MAVLink and QGIS
### Scripts
- `vx_script.py`  : deals with converting ros msgs into mavlink msgs and sends across the desired transport(Both RF and UDP are supported). Runs on the Vehicle. <br>
- `gcs_script.py` : receives MAVLink msgs and sends the necesary parsed messages to its clients via sockets. Runs on the GCS computer. <br>
- `client.py` : is the client-side script. Communicates with the GCS via sockets.<br>
- `interface.py` is the QGIS Interface. 

### Requirements && Dependencies
```pip3 install -r requirements.txt```
<br>
- [QGIS](https://www.qgis.org/en/site/forusers/download.html) for the Client side. Used to visualize vehicle telematics <br>
- Radio Modules for Vehicle side (optional) <br>
- Wi-Fi Router (For communication between GCS and Client)

### System Architecture
There are 3 components- Vx, GCS, and Client side.
GCS distributes Vx data to all the connected clients.
![alt_text](/media/arch.png)


### Usage
- Configure the `config/[component]_param.yaml` to the desired states. <br>
- Launch `roslaunch vx_comms start_comms_vx.launch`  on the vehicle side. <br>
- Configure and `python3 scripts/gcs_script.py` on the GCS side. <br>
- Configure and `python3 scripts/client.py` on the Client side. <br>
- Download QuickMapServices Plugin from the QGIS plugin store. Get more maps by
`Web ► QuickMapServices ► Settings`<br>
Go to the `More services` tab. <br>
Read carefully the message of this tab and if you agree click on the `Get Contributed pack` button. <br>
Click `Save`.<br>
Reopen the `Web ► QuickMapServices` menu you will see that more providers are available.<br>
- Open the QGIS project `dashboard.qgz` and open `interface.py` in the environment.

![alt_text](/media/qgis_updated.gif)
