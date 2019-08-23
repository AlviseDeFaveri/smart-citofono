#Smart citofono

IoT project. `common.*` and `protocol.h` contain the code shared between mote
interphone.

To run the simulation:

* open cooja
* add contiki/examples/ipv6/rpl-border-router as cooja mote to the simulation
* right click on the mote > serial port (server) > start
* on a terminal: `sudo ~/contiki/tools/tunslip6 -a 127.0.0.1 aaaa::1/64`
* add contiki/apps/smart-citofono/citofono.c as cooja mote to the simulation
* add contiki/apps/smart-citofono/mote.c as cooja mote to the simulation
* start simulation
* on another terminal, start `mosquitto_sub -h localhost -t "#"` to monitor
published messages