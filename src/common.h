/**
 * Smart citofono for IoT project.
 * Author: Alvise de'Faveri Tron
 *
 * This file contains the interface to the mqtt state machine, which handles
 * the registration, connection and disconnection phases of the MQTT communication.
 * It is used by both the mote and the interphone node.
 */
#pragma once

#include "contiki-conf.h"
#include "mqtt.h"

#define STATE_INIT            0
#define STATE_DISCONNECTED    1
#define STATE_CONNECTED       2
#define STATE_IDLE            3
#define STATE_PUBLISHING      4
#define STATE_ERROR        0xFF

// Where to search for the payload inside a published message
#define STATE_OFFSET_IN_MSG  12
#define SEQ_OFFSET_IN_MSG    21

typedef void (*IdleHandler)(process_event_t ev, process_data_t data);
typedef void (*PubHandler) (const char *topic, uint16_t topic_len,
                            const uint8_t *chunk, uint16_t chunk_len);


/*-------------------------INIT FUNC----------------------------------------*/
/**
 * Initialization of the mqtt state-machine.
 */
void mqtt_fsm_init( char* broker_ip,
                    uint16_t broker_port,
                    char* type_id,
                    char* pub_topic,
                    char* sub_topic,
                    struct process* p,
                    IdleHandler idle_h,
                    PubHandler  pub_h );


/*-----------------------PUBLISH FUNC----------------------------------------*/
/**
 * Set the message to publish.
 */
void mqtt_fsm_publish(char payload);


/*-------------------------STATE MACHINE-------------------------------------*/
/**
 * This function executes the action corresponding to the current state.
 */
void mqtt_fsm_run(process_event_t ev, process_data_t data);