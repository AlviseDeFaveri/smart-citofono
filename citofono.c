
/*---------------------------------------------------------------------------*/
/**
 * Smart citofono for IoT project.
 *
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "mqtt.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
// #include "dev/cc2538-sensors.h"

#include <string.h>


/*---------------------------------------------------------------------------*/
/*
 * A timeout used when waiting for something to happen (e.g. to connect or to
 * disconnect)
 */
#define STATE_MACHINE_PERIODIC     (CLOCK_SECOND >> 1)
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS during various states */
/* When connecting to broker */
#define CONNECTING_LED_DURATION    (CLOCK_SECOND >> 2)

/* Each time we try to publish */
#define PUBLISH_LED_ON_DURATION    (CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
/* Connections and reconnections */
#define RETRY_FOREVER              0xFF
#define RECONNECT_INTERVAL         (CLOCK_SECOND * 2)

/*
 * Number of times to try reconnecting to the broker.
 * Can be a limited number (e.g. 3, 10 etc) or can be set to RETRY_FOREVER
 */
#define RECONNECT_ATTEMPTS         RETRY_FOREVER
#define CONNECTION_STABLE_TIME     (CLOCK_SECOND * 5)
static struct timer connection_life;
static uint8_t connect_attempt;
/*---------------------------------------------------------------------------*/
/* Various states */

#define STATE_INIT            0
#define STATE_REGISTERED      1
#define STATE_CONNECTING      2
#define STATE_CONNECTED       3
#define STATE_PUBLISHING      4
#define STATE_DISCONNECTED    5
#define STATE_NEWCONFIG       6
#define STATE_CONFIG_ERROR 0xFE
#define STATE_ERROR        0xFF
/*---------------------------------------------------------------------------*/
#define CONFIG_ORG_ID_LEN        32
#define CONFIG_TYPE_ID_LEN       32
#define CONFIG_AUTH_TOKEN_LEN    32
#define CONFIG_EVENT_TYPE_ID_LEN 32
#define CONFIG_CMD_TYPE_LEN       8
#define CONFIG_IP_ADDR_STR_LEN   64
/*---------------------------------------------------------------------------*/
#define RSSI_MEASURE_INTERVAL_MAX 86400 /* secs: 1 day */
#define RSSI_MEASURE_INTERVAL_MIN     5 /* secs */
#define PUBLISH_INTERVAL_MAX      86400 /* secs: 1 day */
#define PUBLISH_INTERVAL_MIN          5 /* secs */
/*---------------------------------------------------------------------------*/
/* A timeout used when waiting to connect to a network */
#define NET_CONNECT_PERIODIC        (CLOCK_SECOND >> 2)
#define NO_NET_LED_DURATION         (NET_CONNECT_PERIODIC >> 1)
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* Take a sensor reading on button press */
#define PUBLISH_TRIGGER &button_sensor

/* Payload length of ICMPv6 echo requests used to measure RSSI with def rt */
#define ECHO_REQ_PAYLOAD_LEN   20
/*---------------------------------------------------------------------------*/

/**
 * \brief Data structure declaration for the MQTT client configuration
 */
typedef struct mqtt_client_config {
  char org_id[CONFIG_ORG_ID_LEN];
  char type_id[CONFIG_TYPE_ID_LEN];
  char auth_token[CONFIG_AUTH_TOKEN_LEN];
  char event_type_id[CONFIG_EVENT_TYPE_ID_LEN];
  char broker_ip[CONFIG_IP_ADDR_STR_LEN];
  char cmd_type[CONFIG_CMD_TYPE_LEN];
  clock_time_t pub_interval;
  int def_rt_ping_interval;
  uint16_t broker_port;
} mqtt_client_config_t;
/*---------------------------------------------------------------------------*/
/* Maximum TCP segment size for outgoing segments of our socket */
#define MAX_TCP_SEGMENT_SIZE    32
/*---------------------------------------------------------------------------*/
#define STATUS_LED LEDS_GREEN
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
#define QUICKSTART "quickstart"
/*---------------------------------------------------------------------------*/
static struct mqtt_message *msg_ptr = 0;
static struct etimer fsm_periodic_timer;
static struct ctimer ct;
static char *buf_ptr;
static uint16_t seq_nr_value = 0;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
static struct etimer echo_request_timer;
static int def_rt_rssi = 0;
/*---------------------------------------------------------------------------*/
static mqtt_client_config_t conf;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
int
ipaddr_sprintf(char *buf, uint8_t buf_len, const uip_ipaddr_t *addr)
{
  uint16_t a;
  uint8_t len = 0;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        len += snprintf(&buf[len], buf_len - len, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        len += snprintf(&buf[len], buf_len - len, ":");
      }
      len += snprintf(&buf[len], buf_len - len, "%x", a);
    }
  }

  return len;
}
/*---------------------------------------------------------------------------*/
static void
publish_led_off(void *d)
{
  leds_off(STATUS_LED);
}


/*---------------------------------------------------------------------------*/
static int
construct_client_id(void)
{
  int len = snprintf(client_id, BUFFER_SIZE, "d:%s:%s:%02x%02x%02x%02x%02x%02x",
                     conf.org_id, conf.type_id,
                     linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
                     linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
                     linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Client ID: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }

  return 1;
}

/*---------------------------------------------------------------------------*/
static void
pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
  DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,
      chunk_len);

  /* If we don't like the length, ignore */
  if(topic_len != 23 || chunk_len != 1) {
    printf("Incorrect topic or chunk len. Ignored\n");
    return;
  }

  // TODO change
  if(strncmp(&topic[10], "leds", 4) == 0) {
    if(chunk[0] == '1') {
      leds_on(LEDS_RED);
    } else if(chunk[0] == '0') {
      leds_off(LEDS_RED);
    }
    return;
  }
}

/*---------------------------------------------------------------------------*/
static void
publish(void)
{
  /* Publish MQTT topic in IBM quickstart format */
  int len;
  int remaining = APP_BUFFER_SIZE;

  seq_nr_value++;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
                 "{"
                 "\"d\":{"
                 "\"myName\":\"MQTTMote\","
                 "\"Seq #\":%d,"
                 "\"Uptime (sec)\":%lu",
                 seq_nr_value, clock_seconds());

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;

  /* Put our Default route's string representation in a buffer */
  char def_rt_str[64];
  memset(def_rt_str, 0, sizeof(def_rt_str));
  ipaddr_sprintf(def_rt_str, sizeof(def_rt_str), uip_ds6_defrt_choose());

  len = snprintf(buf_ptr, remaining, ",\"Def Route\":\"%s\",\"RSSI (dBm)\":%d",
                 def_rt_str, def_rt_rssi);

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }
  remaining -= len;
  buf_ptr += len;

  /*len = snprintf(buf_ptr, remaining, ",\"On-Chip Temp (mC)\":%d",
                 cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED));*/

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }
  remaining -= len;
  buf_ptr += len;

  /*len = snprintf(buf_ptr, remaining, ",\"VDD3 (mV)\":%d",
                 vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED));*/

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }
  remaining -= len;
  buf_ptr += len;

  len = snprintf(buf_ptr, remaining, "}}");

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  mqtt_publish(&conn, NULL, pub_topic, (uint8_t *)app_buffer,
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);

  DBG("APP - Publish!\n");
}














/*-----------------------CONFIGURATION---------------------------------------*/
/* Default configuration values */
static const char* DEFAULT_TYPE_ID =     "cc2538";
static const char* auth_token =  "Z18CAN";

static const char *broker_ip = "fd00::1";
static const uint16_t broker_port = 1883;
static const char* org_id =  "smart-citofono";
static const char* client_id = "d:quickstart:EUI64";
static const char* pub_topic = "iot-2/evt/status/fmt/json";
static const char* sub_topic = "iot-2/cmd/+/fmt/json";

static const clock_time_t pub_interval =  (30 * CLOCK_SECOND);
static const unsigned int keep_alive_timer =   60;
static const int rt_ping_interval = (CLOCK_SECOND * 30);

#define APP_BUFFER_SIZE 512
static char app_buffer[APP_BUFFER_SIZE];

/* Global variables (file-private) */
static struct mqtt_connection conn;
static uint8_t state;

/*---------------------------------------------------------------------------*/
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
    mqtt_status_t status = mqtt_register(&conn, &mqtt_process, client_id, mqtt_event,
                  MAX_TCP_SEGMENT_SIZE);

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
   * SUBSCRIBED state: the mote is correctly connected with the broker and ready
   * to execute its nominal behavour.
   */
  case STATE_SUBSCRIBED:
  state = STATE_IDLE;
  /* Continue */
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
   * PUBLISHED state: all good, return back to idle.
   */
  case STATE_PUBLISHED:
  {
    printf("Message published.\n");
    state = SUBSCRIBED;
    break;
  }

  /**
   * DISCONNECTED state: the mote has received a DISCONNECTED event. Retry to
   * connect.
   */
  case STATE_DISCONNECTED:
    printf("STATE DISCONNECTED\n");
    DBG("Disconnected\n");
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

  /* If we didn't return so far, reschedule state machine */
  etimer_set(&fsm_periodic_timer, STATE_MACHINE_PERIODIC);
}

/*---------------------------------------------------------------------------*/
static void
mqtt_event_handler(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  case MQTT_EVENT_CONNECTED: {
    DBG("APP - Application has a MQTT connection\n");
    timer_set(&connection_life, CONNECTION_STABLE_TIME);
    state = STATE_CONNECTED;
    break;
  }
  case MQTT_EVENT_DISCONNECTED: {
    DBG("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));

    state = STATE_DISCONNECTED;
    process_poll(&mqtt_process);
    break;
  }
  case MQTT_EVENT_PUBLISH: {
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
  case MQTT_EVENT_SUBACK: {
    DBG("APP - Application is subscribed to topic successfully\n");
    if(state == STATE_CONNECTED)
    {
      state = STATE_SUBSCRIBED;
    }
    break;
  }
  case MQTT_EVENT_UNSUBACK: {
    DBG("APP - Application is unsubscribed to topic successfully\n");
    break;
  }
  case MQTT_EVENT_PUBACK: {
    DBG("APP - Publishing complete.\n");
    if(state == PUBLISHING)
    {
      state = PUBLISHED;
    }
    break;
  }
  default:
    DBG("APP - Application got a unhandled MQTT event: %i\n", event);
    break;
  }
}


/*-----------------------PROCESS INIT----------------------------------------*/ù
PROCESS(mqtt_process, "MQTT Smart Citofono");
PROCESS_NAME(mqtt_process);
AUTOSTART_PROCESSES(&mqtt_process);


/*-------------------------MAIN PROCESS--------------------------------------*/
PROCESS_THREAD(mqtt_process, ev, data)
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

    /* If it's an internal event, process the state machine */
    if((ev == PROCESS_EVENT_TIMER && data == &fsm_periodic_timer) ||
       ev == PROCESS_EVENT_POLL)
    {

      if((ev == PROCESS_EVENT_TIMER && data == &fsm_periodic_timer))
          printf("Process event = timer, Publish periodic timer\n");
      if(ev == PROCESS_EVENT_POLL)
          printf("Process event = poll\n");

      state_machine();
    }

    /* If the state is IDLE, execute the idle function */
    if(state = STATE_IDLE)
    {
      idle_state_handler(mqtt_process, ev, data);
    }


    /* handle the echo timer */
    if(ev == PROCESS_EVENT_TIMER && data == &echo_request_timer) {
      //ping_parent();
      etimer_set(&echo_request_timer, conf.def_rt_ping_interval);
    }
  }

  PROCESS_END();
}