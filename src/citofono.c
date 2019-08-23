/**
 * Smart citofono for IoT project.
 * Author: Alvise de'Faveri Tron
 *
 * This file contains the code for the interphone node. It is responsible of
 * publishing a RING message when a call is received on the interphone, and
 * open the gate if an OPEN message is received within a time window.
 */
#include "common.h"
#include "protocol.h"

#include "mqtt.h"
#include "contiki-conf.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"

/* Configuration */
#define BROKER_IP    "aaaa::1"
#define BROKER_PORT  1883
#define TYPE_ID      "citofono"
#define PUB_TOPIC    "iot-2/evt/status/fmt/json"
#define SUB_TOPIC    "iot-2/cmd/+/fmt/json"

#define CITOFONO_STATE_IDLE      0
#define CITOFONO_STATE_RINGING   1
#define CITOFONO_STATE_OPEN_RCV  2
#define CITOFONO_STATE_OPENING   3

#define RING_TIMEOUT        (30*CLOCK_SECOND)
#define RING_INTERVAL       (2*CLOCK_SECOND)
#define OPEN_TIME           (2*CLOCK_SECOND)
#define HEARTBEAT_INTERVAL  (5*CLOCK_SECOND)

/* Private variables */
static struct etimer open_timer;
static struct etimer heartbeat_timer;
static struct etimer ring_timer;
static struct etimer ring_timeout;
static int mote_state = 0;
static int ping = 0;

/*-----------------------PROCESS INIT----------------------------------------*/
PROCESS(mqtt_fsm_process, "MQTT Smart Citofono");
PROCESS_NAME(mqtt_fsm_process);
AUTOSTART_PROCESSES(&mqtt_fsm_process);


/*-------------------------HW MAPPING----------------------------------------*/
/* For simulation pourposes, the hardware has been mapped has follows:
 * - INTERPHONE CALL = button press
 * - RINGING         = blue LED
 * - OPENING         = red LED
 */
#define INTERPHONE_CALL_SENSOR button_sensor

inline void openGate()
{
  printf("APP [citofono] - Opening gate\n");
  leds_on(LEDS_GREEN);
}

inline void stopGate()
{
  leds_off(LEDS_GREEN);
}

inline void startRinging()
{
  printf("APP [citofono] - Interphone ringing\n");

  etimer_set(&ring_timer, RING_INTERVAL);
  mqtt_fsm_publish(CITOFONO_RINGING);
  leds_toggle(LEDS_BLUE);
}

inline void stopRinging()
{
  etimer_stop(&ring_timer);
  etimer_stop(&ring_timeout);
  leds_off(LEDS_BLUE);
}

/*-----------------------HEARTBEAT HANDLER-----------------------------------*/
/**
 * If a ping message is received, publish a state message.
 */
void handleHeartbeat()
{
  // Encode the state in a message
  switch(mote_state)
  {
    case CITOFONO_STATE_IDLE:
      DBG("APP [citofono] - Sending IDLE\n");
      mqtt_fsm_publish(CITOFONO_IDLE);
      break;
    case CITOFONO_STATE_RINGING:
      DBG("APP [citofono] - Sending RINGING\n");
      mqtt_fsm_publish(CITOFONO_RINGING);
      break;
    case CITOFONO_STATE_OPEN_RCV:
    case CITOFONO_STATE_OPENING:
      DBG("APP [citofono] - Sending OPENING\n");
      mqtt_fsm_publish(CITOFONO_OPENING);
      break;
  }

  ping = 0;
}

/*------------------------PUB HANDLER----------------------------------------*/
/**
 * What to do when a message is received.
 */
void pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
                    uint16_t chunk_len)
{
  // Check messages id
  switch(chunk[STATE_OFFSET_IN_MSG])
  {
    // change state at next heartbeat
    case MOTE_OPEN:
      if(mote_state == CITOFONO_STATE_RINGING)
      {
        printf("APP [citofono] - Open message received\n");
        mote_state = CITOFONO_STATE_OPEN_RCV;
      }
      break;
    // handle ping at next heartbeat
    case MOTE_PING:
      printf("APP [citofono] - Ping received\n");
      ping = 1;
      break;
  }

  process_poll(&mqtt_fsm_process);
  DBG("APP [citofono] - Received message!");
}


/*-----------------------IDLE HANDLER----------------------------------------*/
/**
 * What to do in MQTT_IDLE state (i.e. the mqtt engine has been initialized and
 * no MQTT-related action is ongoing).
 */
void idle_state_handler(process_event_t ev, process_data_t data)
{

  switch(mote_state)
  {
    // Wait to receive a call on the interphone
    case CITOFONO_STATE_IDLE:
    {
      stopRinging();
      stopGate();

      if(ev == sensors_event && data == &INTERPHONE_CALL_SENSOR)
      {
        startRinging();

        etimer_set(&ring_timeout, RING_TIMEOUT);
        mote_state = CITOFONO_STATE_RINGING;
      }
      break;
    }

    // Ringing. Can be exited by receiving a timeout or an OPEN msg.
    case CITOFONO_STATE_RINGING:
    {
      if(ev == PROCESS_EVENT_TIMER && data == &ring_timer)
      {
        startRinging();
      }
      else if(ev == PROCESS_EVENT_TIMER && data == &ring_timeout)
      {
        printf("APP [citofono] - Timer expired, gate not opened\n");
        mqtt_fsm_publish(CITOFONO_NOT_OPENED);
        mote_state = CITOFONO_STATE_IDLE;
      }
      break;
    }

    // Open message accepted: start opening.
    case CITOFONO_STATE_OPEN_RCV:
    {
      stopRinging();
      openGate();

      mqtt_fsm_publish(CITOFONO_OPENING);
      etimer_set(&open_timer, OPEN_TIME);
      mote_state = CITOFONO_STATE_OPENING;
      break;
    }

    // When open timeout is received, finish opening and return to idle.
    case CITOFONO_STATE_OPENING:
    {
      if(ev == PROCESS_EVENT_TIMER && data == &open_timer)
      {
        stopGate();
        printf("APP [citofono] - Gate opened\n");
        mqtt_fsm_publish(CITOFONO_OPENED);
        mote_state = CITOFONO_STATE_IDLE;
      }
      break;
    }

  }
}


/*-------------------------MAIN PROCESS--------------------------------------*/
PROCESS_THREAD(mqtt_fsm_process, ev, data)
{
  PROCESS_BEGIN();
  printf("APP [citofono] - Smart-citofono interphone started\n");

  /* Initialize the mqtt state machine */
  mqtt_fsm_init(BROKER_IP, BROKER_PORT, TYPE_ID, PUB_TOPIC, SUB_TOPIC,
                &mqtt_fsm_process, &idle_state_handler, &pub_handler);

  /* Initialize citofono */
  SENSORS_ACTIVATE(INTERPHONE_CALL_SENSOR);
  leds_off(LEDS_ALL);
  mote_state = CITOFONO_STATE_IDLE;
  etimer_set(&heartbeat_timer, HEARTBEAT_INTERVAL);

  /* Main loop */
  while(1)
  {
    PROCESS_WAIT_EVENT();

    /* Handle heartbeat and reschedule */
    if(ev == PROCESS_EVENT_TIMER && data == &heartbeat_timer)
    {
      handleHeartbeat();
      etimer_set(&heartbeat_timer, HEARTBEAT_INTERVAL);
    }

    /* Advance the state machine */
    mqtt_fsm_run(ev, data);
  }

  PROCESS_END();
}