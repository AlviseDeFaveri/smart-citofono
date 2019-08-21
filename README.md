To run the simulation:

* open cooja
* add contiki/examples/ipv6/rpl-border-router as cooja mote
* right click on the mote > serial port (server) > start
* on a terminal: `sudo ~/contiki/tools/tunslip6 -a 127.0.0.1 aaaa::1/64`
* add contiki/apps/smart-citofono/common.c as mote
* start