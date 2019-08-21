/**
 * Smart citofono for IoT project.
 */
#include "contiki-conf.h"
#include "mqtt.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"

#define STATE_INIT            0
#define STATE_CONNECTING      1
#define STATE_CONNECTED       2
#define STATE_IDLE            3
#define STATE_PUBLISHING      4
#define STATE_DISCONNECTED 0xFE
#define STATE_ERROR        0xFF

#define MAX_TCP_SEGMENT_SIZE    32

#define RETRY_FOREVER              0xFF
#define RECONNECT_INTERVAL         (CLOCK_SECOND * 2)
#define RECONNECT_ATTEMPTS         RETRY_FOREVER
#define NET_CONNECT_PERIODIC        (CLOCK_SECOND >> 2)
#define STATE_MACHINE_PERIODIC     (CLOCK_SECOND >> 1)

/*-----------------------CONFIGURATION---------------------------------------*/
/* Specific implementation may vary between different motes. */
#include "mote-impl.h"

/*----------------------GLOBAL VARIABLES---------------------------------------*/
static const clock_time_t pub_interval =  (30 * CLOCK_SECOND);
static const unsigned int keep_alive_timer =   60;
static const int rt_ping_interval = (CLOCK_SECOND * 30);

static struct etimer fsm_periodic_timer;
static uint16_t seq_nr_value = 0;
static struct etimer echo_request_timer;

static struct mqtt_connection conn;
static uint8_t state;
static struct mqtt_message *msg_ptr = 0;
static uint8_t connect_attempt;

/*-----------------------PROCESS INIT----------------------------------------*/
PROCESS(mqtt_fsm_process, "MQTT Smart Citofono");
PROCESS_NAME(mqtt_fsm_process);
AUTOSTART_PROCESSES(&mqtt_fsm_process);


/*--------------------------EVENT HANDLER------------------------------------*/
/**
 * This function handles the arrival of MQTT messages from the MQTT driver
 *
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
    process_poll(&mqtt_fsm_process);
    break;
  }

  case MQTT_EVENT_PUBLISH:
  {
    msg_ptr = data;

    /* Implement first_flag in publish message? */
    if(msg_ptr->first_chunk) {
      msg_ptr->first_chunk = 0;
      DBG("APP - Application received a publish on topic '%s'. Payload "
          "size is %i bytes. Content:\n\n",
          msg_ptr->topic, msg_ptr->payload_length);
    }

    pub_handler(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    break;
  }

  case MQTT_EVENT_SUBACK:
  {
    DBG("APP - Application is subscribed to topic successfully\n");
    if(state == STATE_CONNECTED)
    {
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
}


/*-------------------------STATE MACHINE-------------------------------------*/
/**
 * This function executes the action corresponding to the current state.
 */
static void state_machine(void)
{
  switch(state) {

  /**
   * INIT state: initialize the mqtt driver and set last will. This state
   * performs a spontaneous transition to the CONNECTING state when it's finished.
   */
  case STATE_INIT:
  {
    printf("STATE INIT\n");
    mqtt_status_t status = mqtt_register(&conn, &mqtt_fsm_process, client_id,
                                        mqtt_event_handler, MAX_TCP_SEGMENT_SIZE);

    if(status == MQTT_STATUS_OK)
    {
       mqtt_set_username_password(&conn, "use-token-auth", auth_token);
      //mqtt_set_last_will()...
      connect_attempt = 1;
      state = STATE_CONNECTING;
    }
    else
    {
      state = STATE_ERROR;
      return;
    }

  }
  /* Continue (i.e. spontaneous transition to next state). */

  /**
   * CONNECTING state: try to connect to the broker. The mote
   * can exit this state either by receiving a MQTT_EVENT_CONNECTED, handled
   * in the mqtt_event_handler(), or because of a driver error.
   */
  case STATE_CONNECTING:
  {
    printf("STATE REGISTERED. Connect attempt %u\n", connect_attempt);

    mqtt_status_t status = mqtt_connect(&conn, broker_ip, broker_port,
                   pub_interval * 3);

    if(status != MQTT_STATUS_OK)
    {
      state = STATE_ERROR;
    }

    etimer_set(&fsm_periodic_timer, NET_CONNECT_PERIODIC);
    return;
    break;
  }

  /**
   * CONNECTED state: try to subscribe to the sub_topic. The subscription
   * needs to be confirmed by the broker. The mote
   * can exit this state either by receiving a MQTT_EVENT_SUBACK, handled
   * in the mqtt_event_handler(), or because of a driver error.
   */
  case STATE_CONNECTED:
  {
    printf("STATE CONNECTED\n");
    connect_attempt = 1;

    mqtt_status_t status = mqtt_subscribe(&conn, NULL, sub_topic,
                                            MQTT_QOS_LEVEL_0);

    DBG("APP - Subscribing!\n");
    if(status != MQTT_STATUS_OK)
    {
      state = STATE_ERROR;
    }
    return;
    break;
  }

  /**
   * IDLE state: the mote is correctly connected with the broker and ready
   * to execute its nominal behavour.
   */
  case STATE_IDLE:
  {
    return;
    break;
  }

  /**
   * PUBLISHING state: try to publish a message. If the queue is full, schedule
   * a retry.
   */
  case STATE_PUBLISHING:
  {
    printf("STATE Publishing\n");

    if(mqtt_ready(&conn) && conn.out_buffer_sent)
    {
      //leds_on(STATUS_LED);
      //ctimer_set(&ct, PUBLISH_LED_ON_DURATION, publish_led_off, NULL);
      publish();

      etimer_set(&fsm_periodic_timer, pub_interval);

      DBG("Publishing\n");
      /* Return here so we don't end up rescheduling the timer */
      return;
    }
    else
    {
      DBG("Publishing error (MQTT state=%d, q=%u)\n", conn.state,
          conn.out_queue_full);
    }
    break;
  }

  /**
   * DISCONNECTED state: the mote has received a DISCONNECTED event. Retry to
   * connect.
   */
  case STATE_DISCONNECTED:
  {
    printf("STATE DISCONNECTED\n");

    if(connect_attempt < RECONNECT_ATTEMPTS ||
       RECONNECT_ATTEMPTS == RETRY_FOREVER) {
      /* Disconnect and backoff */
      clock_time_t interval;
      mqtt_disconnect(&conn);
      connect_attempt++;

      interval = connect_attempt < 3 ? RECONNECT_INTERVAL << connect_attempt :
        RECONNECT_INTERVAL << 3;

      DBG("Disconnected. Attempt %u in %lu ticks\n", connect_attempt, interval);

      etimer_set(&fsm_periodic_timer, interval);

      state = STATE_CONNECTING;
      return;
    } else {
      /* Max reconnect attempts reached. Enter error state */
      state = STATE_ERROR;
      DBG("Aborting connection after %u attempts\n", connect_attempt - 1);
    }
    break;
  }

  /**
   * ERROR state: light up a led.
   */
  case STATE_ERROR:
  {
    // TODO led blink
    break;
  }
  }

  /* If we didn't return so far, reschedule state machine */
  etimer_set(&fsm_periodic_timer, STATE_MACHINE_PERIODIC);
}


/*-------------------------MAIN PROCESS--------------------------------------*/
PROCESS_THREAD(mqtt_fsm_process, ev, data)
{
  PROCESS_BEGIN();
  printf("Smart citofono started\n");

  /* Init fsm  */
  seq_nr_value = 0;
  state = STATE_INIT;
  etimer_set(&fsm_periodic_timer, 0); // schedule first state machine event

  /* Start timers */
  // etimer_set(&echo_request_timer, conf.def_rt_ping_interval);

  /* Main loop */
  while(1) {

    PROCESS_YIELD();

    /* If it's an internal event, advance the mqtt state machine */
    if((ev == PROCESS_EVENT_TIMER && data == &fsm_periodic_timer) ||
       ev == PROCESS_EVENT_POLL)
    {
      state_machine();
    }

    /* If the state is IDLE, execute the mote-specific IDLE handler. Each
     * mote type has a specific state handler (e.g. interphone and remote
     * controller).
     */
    if(state == STATE_IDLE)
    {
      enum next_action_t next = idle_state_handler(ev, data);

      if(next == PUBLISH)
      {
        state = STATE_PUBLISHING;
      }
      else if(next == ERROR)
      {
        state = STATE_ERROR;
      }
    }


    /* handle the echo timer */
    if(ev == PROCESS_EVENT_TIMER && data == &echo_request_timer) {
      //ping_parent();
      //etimer_set(&echo_request_timer, conf.def_rt_ping_interval);
    }
  }

  PROCESS_END();
}