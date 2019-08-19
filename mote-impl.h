#pragma once

#include "contiki-conf.h"
#include "sys/etimer.h"

/**
 * Mote-specific configuration parameters. Must be specified
 * for each mote.
 */
extern const char* DEFAULT_TYPE_ID;
extern const char* auth_token;

extern const char *broker_ip;
extern const uint16_t broker_port;
extern const char* org_id;
extern const char* client_id;
extern const char* pub_topic;
extern const char* sub_topic;

enum next_action_t
{
    NO_ACTION,
    PUBLISH,
    ERROR
};

/* Functions used in the MQTT state machine: override for every mote impl.*/
void publish();
void pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
                    uint16_t chunk_len);
enum next_action_t idle_state_handler(process_event_t ev, process_data_t data);