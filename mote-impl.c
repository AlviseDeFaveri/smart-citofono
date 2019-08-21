#include "sys/etimer.h"

static struct etimer publish_timer;
static uint8_t init = 0:

/**
 * What to do when publishing.
 */
void publish()
{
    printf("Publishing\n");
}

/**
 * What to do when a message is received.
 */
void pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
                    uint16_t chunk_len)
{
    printf("Received message!\n");
}

/**
 * What to do in idle state.
 */
enum next_action_t idle_state_handler(process_event_t ev, process_data_t data)
{
    if(init == 0)
    {
        init = 1;
        etimer_set(&publish_timer, 0);
    }

    if(ev == PROCESS_EVENT_TIMER && data == &publish_timer)
    {
        etimer_set(&publish_timer, 1000);
        return PUBLISH;
    }
    else
    {
        return NO_ACTION;
    }
}