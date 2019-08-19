CONTIKI_PROJECT = common
all: common

CONTIKI_WITH_IPV6 = 1
APPS += mqtt

PROJECT_SOURCEFILES= mote-impl.c

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
