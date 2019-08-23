/**
 * Smart citofono for IoT project.
 * Author: Alvise de'Faveri Tron
 *
 * This file contains the protocol used in the published message, in particular
 * the meaning of the values inside the "co" fields.
 */
#pragma once

// code sent by interphone
#define CITOFONO_IDLE          'i'
#define CITOFONO_RINGING       'r'
#define CITOFONO_OPENING       'o'
#define CITOFONO_OPENED        'y'
#define CITOFONO_NOT_OPENED    'n'

// code sent by mote
#define MOTE_PING      'p'
#define MOTE_OPEN      'o'