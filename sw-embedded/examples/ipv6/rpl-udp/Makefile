all: udp-client udp-server
CONTIKI=../../..
# Add stm32nucleo platform folder
TARGETDIRS += ../../platform/stm32nucleo-dw1000
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+= -DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif

ifdef SERVER_REPLY
CFLAGS+=-DSERVER_REPLY=$(SERVER_REPLY)
endif
ifdef PERIOD
CFLAGS+=-DPERIOD=$(PERIOD)
endif

ifeq ($(MAKE_WITH_NON_STORING),1)
CFLAGS += -DWITH_NON_STORING=1
endif

CONTIKI_WITH_RIME = 0
CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include

include $(CONTIKI)/NodeSerialNumbers.include

upA: udp-server.bin
	st-flash --serial $(NODEA) write udp-server.bin 0x08000000

upB: udp-client.bin
	st-flash --serial $(NODEB) write udp-client.bin 0x08000000

upC: udp-client.bin
	st-flash --serial $(NODEC) write udp-client.bin 0x08000000

upD: udp-client.bin
	st-flash --serial $(NODED) write udp-client.bin 0x08000000

up2: udp-client.bin
	st-flash --serial $(NODE2) write udp-client.bin 0x08000000

up3: udp-client.bin
	st-flash --serial $(NODE3) write udp-client.bin 0x08000000

up5: udp-client.bin
	st-flash --serial $(NODE5) write udp-client.bin 0x08000000

up6: udp-client.bin
	st-flash --serial $(NODE6) write udp-client.bin 0x08000000

up8: udp-client.bin
	st-flash --serial $(NODE8) write udp-client.bin 0x08000000

up9: udp-client.bin
	st-flash --serial $(NODE9) write udp-client.bin 0x08000000

up10: udp-client.bin
	st-flash --serial $(NODE10) write udp-client.bin 0x08000000

up11: udp-client.bin
	st-flash --serial $(NODE11) write udp-client.bin 0x08000000

up12: udp-client.bin
	st-flash --serial $(NODE12) write udp-client.bin 0x08000000

upGroup: upA upClients

upClients: up8 up11 up12 upD
