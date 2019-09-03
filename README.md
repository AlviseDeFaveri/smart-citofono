# Smart citofono

Smart interphone IoT project done with Cooja, ContikiOS, Mosquitto and NodeRed.

**Folder description**

* `src/` contains the Contiki sources
* `node-red/` contains the NodeRed dashboard
* `resources/` contains images and diagrams for the report
* `smart-citofono.csc` is the Cooja simulation file
* `log.txt` is a sample log file

**Simulation instructions**

To for the simulation to work, you must have a running Mosquitto instance. These instructions have been tested on the provided Ubuntu VM.

* move this folder in `contiki/apps`

* open Cooja
* load the simulation `smart-citofono.csc` and check that Cooja is listening on port 6001 and 6002.
* on a terminal: type `sudo ~/contiki/tools/tunslip6 -a 127.0.0.1 aaaa::1/64`
* start the simulation
* on another terminal: type `node-red` and, once it's started, open the browser at *127.0.0.1:1800/ui*