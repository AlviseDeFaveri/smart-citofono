/**
 * Smart citofono for IoT project.
 * Author: Alvise de'Faveri Tron
 *
 * This file contains the the mqtt state machine which handles
 * the registration, connection and disconnection phases of the MQTT communication.
 * It is used by both the mote and the interphone node.
 */
#include "common.h"

#include "contiki-conf.h"
#include "mqtt.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"

#define MAX_TCP_SEGMENT_SIZE        32
#define MAX_PAYLOAD_SIZE            50
#define APP_BUFFER_SIZE             512

#define SUBSCRIBE_INTERVAL          (CLOCK_SECOND * 10)
#define PUB_INTERVAL                (CLOCK_SECOND * 10)
#define RECONNECT_INTERVAL          (CLOCK_SECOND * 2)

/*--------------------------VARIABLES----------------------------------------*/
/* Configuration */
static char *broker_ip;
static uint16_t broker_port;
static char* type_id;
static char* pub_topic;
static char* sub_topic;
static struct process* mqtt_fsm_process;
static IdleHandler idle_state_handler;
static PubHandler  pub_handler;

/* Global variables */
static struct etimer fsm_periodic_timer;

static char send_buffer[APP_BUFFER_SIZE];
static struct mqtt_message *rcv_buffer;
static char* org_id  =  "smartcit";
static char client_id[APP_BUFFER_SIZE];

static uint16_t seq_nr_value = 0;
static uint8_t connect_attempt = 0;
static struct mqtt_connection conn;
static uint8_t state;


/*-------------------------INIT FUNC----------------------------------------*/
/**
 * Initialization of the mqtt state-machine.
 */
void mqtt_fsm_init( char* b_ip,
                    uint16_t b_port,
                    char* t_id,
                    char* pub_t,
                    char* sub_t,
                    struct process* p,
                    IdleHandler idle_h,
                    PubHandler  pub_h )
{
  /* Set configuration */
  broker_ip   = b_ip;
  broker_port = b_port;
  type_id     = t_id;
  pub_topic   = pub_t;
  sub_topic   = sub_t;
  mqtt_fsm_process = p;
  idle_state_handler = idle_h;
  pub_handler  = pub_h;

  /* Init client id */
  snprintf(client_id, APP_BUFFER_SIZE, "d:%s:%s:%02x%02x",
           org_id, type_id,
           linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  /* Init fsm  */
  seq_nr_value = 0;
  state = STATE_INIT;
  etimer_set(&fsm_periodic_timer, 0); // schedule first state machine event
}


/*-----------------------PUBLISH FUNC----------------------------------------*/
/**
 * Set the message to be published.
 */
void mqtt_fsm_publish(char payload)
{
  seq_nr_value++;
  snprintf(send_buffer, APP_BUFFER_SIZE,
           "{"
           "\"d\":{"
           "\"id\":\"%c\","
           "\"seq\":\"%d\","
           "\"type\":\"%s-%x\","
           "\"uptime\":%lu}}",
           payload,
           seq_nr_value,
           type_id,
           linkaddr_node_addr.u8[7],
           clock_seconds());
  state = STATE_PUBLISHING;

  /* Wake up the state machine thread */
  process_poll(mqtt_fsm_process);
}


/*--------------------------EVENT HANDLER------------------------------------*/
/**
 * This function handles the arrival of MQTT messages from the MQTT driver.
 */
static void
mqtt_event_handler(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  case MQTT_EVENT_CONNECTED:
  {
    DBG("APP - Application has a MQTT connection\n");
    state = STATE_CONNECTED;
    break;
  }

  case MQTT_EVENT_DISCONNECTED:
  {
    DBG("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));
    state = STATE_DISCONNECTED;
    break;
  }

  case MQTT_EVENT_PUBLISH:
  {
    DBG("APP - Application received a msg from a subscription\n");
    rcv_buffer = data;
    (*pub_handler)(rcv_buffer->topic, strlen(rcv_buffer->topic), rcv_buffer->payload_chunk,
                rcv_buffer->payload_length);
    break;
  }

  case MQTT_EVENT_SUBACK:
  {
    if(state == STATE_CONNECTED)
    {
      DBG("APP - Application is subscribed to topic successfully\n");
      state = STATE_IDLE;
    }
    break;
  }

  case MQTT_EVENT_UNSUBACK:
  {
    DBG("APP - Application is unsubscribed to topic successfully\n");
    break;
  }

  case MQTT_EVENT_PUBACK:
  {
    DBG("APP - Publishing complete.\n");
    if(state == STATE_PUBLISHING)
    {
      state = STATE_IDLE;
    }
    break;
  }

  default:
    DBG("APP - Application got a unhandled MQTT event: %i\n", event);
    break;
  }

  /* Wake up the state machine thread */
  process_poll(mqtt_fsm_process);
}


/*-------------------------STATE MACHINE-------------------------------------*/
/**
 * This function executes the action corresponding to the current state.
 */
void mqtt_fsm_run(process_event_t ev, process_data_t data)
{
  /* If the mote is in IDLE state, execute the handler. */
  if(state==STATE_IDLE)
  {
    (*idle_state_handler)(ev, data);
  }

  /* In other states, react only to fsm timer and polling */
  else if((ev == PROCESS_EVENT_TIMER && data == &fsm_periodic_timer) ||
          ev == PROCESS_EVENT_POLL)
  {
    switch(state) {
    /**
     * INIT state: initialize the mqtt driver and set last will. This state
     * performs a spontaneous transition to the CONNECTING state when it's finished.
     */
    case STATE_INIT:
    {
      printf("APP [%s] - STATE INIT\n", type_id);
      mqtt_status_t status = mqtt_register(&conn, mqtt_fsm_process, client_id,
                                          mqtt_event_handler, MAX_TCP_SEGMENT_SIZE);

      if(status == MQTT_STATUS_OK)
      {
        //mqtt_set_last_will()...
        connect_attempt = 0;
        state = STATE_DISCONNECTED;
      }
      else
      {
        state = STATE_ERROR;
        return;
      }

    }
    /* Continue (i.e. spontaneous transition to next state). */

    /**
     * DISCONNECTED state: try to connect to the broker. The mote
     * can exit this state either by receiving a MQTT_EVENT_CONNECTED, handled
     * in the mqtt_event_handler(), or because of an error.
     */
    case STATE_DISCONNECTED:
    {
      printf("APP [%s] - STATE DISCONNECTED, Connect attempt %u\n",
        type_id, connect_attempt);

      /* Calculate backoff */
      clock_time_t interval = connect_attempt < 3 ?
                  RECONNECT_INTERVAL << connect_attempt :RECONNECT_INTERVAL << 3;

      mqtt_status_t status = mqtt_connect(&conn, broker_ip, broker_port,
                     PUB_INTERVAL * 3);

      if(status != MQTT_STATUS_OK)
      {
        state = STATE_ERROR;
      }

      /* Reschedule */
      etimer_set(&fsm_periodic_timer, interval);
      connect_attempt++;
      break;
    }

    /**
     * CONNECTED state: try to subscribe to the sub_topic. The subscription
     * needs to be confirmed by the broker. The mote
     * can exit this state either by receiving a MQTT_EVENT_SUBACK, handled
     * in the mqtt_event_handler(), or because of an error.
     */
    case STATE_CONNECTED:
    {
      printf("APP [%s] - STATE CONNECTED\n", type_id);
      connect_attempt = 1;

      mqtt_status_t status = mqtt_subscribe(&conn, NULL, sub_topic,
                                              MQTT_QOS_LEVEL_1);

      DBG("APP - Subscribing!\n");
      if(status != MQTT_STATUS_OK)
      {
        state = STATE_ERROR;
      }

      /* Reschedule */
      etimer_set(&fsm_periodic_timer, SUBSCRIBE_INTERVAL);
      break;
    }

    /**
     * PUBLISHING state: try to publish a message. If the queue is full, schedule
     * a retry. The state can be exited when receiving a PUBACK message.
     */
    case STATE_PUBLISHING:
    {
      DBG("APP [%s] - STATE Publishing\n", type_id);

      if(mqtt_ready(&conn) && conn.out_buffer_sent)
      {
        mqtt_publish(&conn, NULL, pub_topic, (uint8_t *)send_buffer,
                 strlen(send_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
        state = STATE_IDLE;

        DBG("APP - Publish!\n");
      }
      else
      {
        DBG("Publishing error (MQTT state=%d, q=%u)\n", conn.state,
            conn.out_queue_full);
      }

      /*  Reschedule */
      etimer_set(&fsm_periodic_timer, PUB_INTERVAL);
      break;
    }

    /**
     * ERROR state: light up a led.
     */
    case STATE_ERROR:
    {
      leds_on(LEDS_RED);
      break;
    }
    }
  }
}