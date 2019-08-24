/**
 * Smart citofono for IoT project.
 * Author: Alvise de'Faveri Tron
 *
 * This file contains the code for the mote node. It is responsible of signaling
 * to the user when a RING message from the interphone is received and send an
 * OPEN message if the corresponding button is pressed.
 */
#include "common.h"
#include "mqtt.h"
#include "contiki-conf.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "protocol.h"

#define BROKER_IP    "aaaa::1"
#define BROKER_PORT  1883
#define TYPE_ID      "mote"
#define SUB_TOPIC    "iot-2/evt/status/fmt/json"
#define PUB_TOPIC    "iot-2/cmd/mote/fmt/json"

#define MOTE_STATE_INIT         0
#define MOTE_STATE_IDLE         1
#define MOTE_STATE_RINGING      2

#define PING_INTERVAL CLOCK_SECOND

static struct etimer ping_timer;
static int mote_state = 0;


/*-----------------------PROCESS INIT----------------------------------------*/
PROCESS(mqtt_fsm_process, "MQTT Smart Citofono mote");
PROCESS_NAME(mqtt_fsm_process);
AUTOSTART_PROCESSES(&mqtt_fsm_process);


/*-------------------------HW MAPPING----------------------------------------*/
/* For simulation pourposes, the hardware has been mapped has follows:
 * - RINGING  = blue LED
 * - BUTTON   = button sensor
 */

inline void startRinging()
{
  printf("APP [mote] - Ringing\n");
  leds_on(LEDS_BLUE);
}

inline void stopRinging()
{
  leds_off(LEDS_BLUE);
}


/*------------------------PUB HANDLER----------------------------------------*/
/**
 * What to do when a message is received.
 */
void pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
                    uint16_t chunk_len)
{
  DBG("APP [mote] -  RCV SEQ: %c%c%c\n",chunk[SEQ_OFFSET_IN_MSG-1],
                    chunk[SEQ_OFFSET_IN_MSG],chunk[SEQ_OFFSET_IN_MSG+1] );
  DBG("APP [mote] -  RCV ID: %c%c%c\n",chunk[STATE_OFFSET_IN_MSG-1],
                    chunk[STATE_OFFSET_IN_MSG],chunk[STATE_OFFSET_IN_MSG+1] );

  // Check messages id
  switch(chunk[STATE_OFFSET_IN_MSG])
  {
    case CITOFONO_IDLE:
      DBG("APP [mote] - Received IDLE\n");
      mote_state = MOTE_STATE_IDLE;
      break;
    case CITOFONO_RINGING:
      DBG("APP [mote] - Received interphone call\n");
      mote_state = MOTE_STATE_RINGING;
      break;
    case CITOFONO_OPENING:
      DBG("APP [mote] - Gate is opening\n");
      mote_state = MOTE_STATE_IDLE;
      break;
    case CITOFONO_OPENED:
      DBG("APP [mote] - Gate is opened\n");
      mote_state = MOTE_STATE_IDLE;
      break;
    case CITOFONO_NOT_OPENED:
      DBG("APP [mote] - Gate has not been opened\n");
      mote_state = MOTE_STATE_IDLE;
      break;
  }

  process_poll(&mqtt_fsm_process);
  DBG("APP [mote] - Received message!\n");
}


/*-----------------------IDLE HANDLER----------------------------------------*/
/**
 * What to do in idle state.
 */
void idle_state_handler(process_event_t ev, process_data_t data)
{
  switch(mote_state)
  {
    // Ping to receive the current interphone state. This state is exited
    // on the first message received (see pub_handler()).
    case MOTE_STATE_INIT:
      if(ev == PROCESS_EVENT_TIMER && data == &ping_timer)
      {
        etimer_set(&ping_timer, PING_INTERVAL);
        printf("APP [mote] - Pinging interphone\n");
        mqtt_fsm_publish(MOTE_PING);
      }
      break;

    // Wait to receive a call from the interphone
    case MOTE_STATE_IDLE:
      stopRinging();
      DBG("APP [mote] - IDLE state\n");
      if(ev == sensors_event && data == &button_sensor)
      {
        printf("APP [mote] - Button pressed but no incoming call\n");
      }
      break;

    // If ringing, accept the open button
    case MOTE_STATE_RINGING:
      startRinging();
      printf("APP [mote] - RINGING state\n");
      if(ev == sensors_event && data == &button_sensor)
      {
        printf("APP [mote] - Sending OPEN message\n");
        mqtt_fsm_publish(MOTE_OPEN);
        mote_state = MOTE_STATE_IDLE;
      }
      break;
  }
}


/*-------------------------MAIN PROCESS--------------------------------------*/
PROCESS_THREAD(mqtt_fsm_process, ev, data)
{
  PROCESS_BEGIN();

  printf("APP [mote] - Smart-citofono mote started\n");

  /* Initialize the mqtt state machine */
  mqtt_fsm_init(BROKER_IP, BROKER_PORT, TYPE_ID, PUB_TOPIC, SUB_TOPIC,
                &mqtt_fsm_process, &idle_state_handler, &pub_handler);

  /* Initialize the mote */
  SENSORS_ACTIVATE(button_sensor);
  mote_state = MOTE_STATE_INIT;
  etimer_set(&ping_timer, 1);

  /* Main loop */
  while(1)
  {
    PROCESS_WAIT_EVENT();

    /* Advance the state machine */
    mqtt_fsm_run(ev, data);
  }

  PROCESS_END();
}