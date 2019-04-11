all: rpl-node
CONTIKI=../../..
# Add stm32nucleo platform folder
TARGETDIRS += ../../platform/stm32nucleo-dw1000
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

CONTIKI_WITH_RIME = 0
CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include

include $(CONTIKI)/NodeSerialNumbers.include

upA: rpl-node.bin
	st-flash --serial $(NODEA) write rpl-node.bin 0x08000000

upB: rpl-node.bin
	st-flash --serial $(NODEB) write rpl-node.bin 0x08000000

upC: rpl-node.bin
	st-flash --serial $(NODEC) write rpl-node.bin 0x08000000

upD: rpl-node.bin
	st-flash --serial $(NODED) write rpl-node.bin 0x08000000

upE: rpl-node.bin
	st-flash --serial $(NODEE) write rpl-node.bin 0x08000000

upS: rpl-node.bin
	st-flash --serial $(NODES) write rpl-node.bin 0x08000000

up2: rpl-node.bin
	st-flash --serial $(NODE2) write rpl-node.bin 0x08000000

up3: rpl-node.bin
	st-flash --serial $(NODE3) write rpl-node.bin 0x08000000

up5: rpl-node.bin
	st-flash --serial $(NODE5) write rpl-node.bin 0x08000000

up6: rpl-node.bin
	st-flash --serial $(NODE6) write rpl-node.bin 0x08000000

up8: rpl-node.bin
	st-flash --serial $(NODE8) write rpl-node.bin 0x08000000

up9: rpl-node.bin
	st-flash --serial $(NODE9) write rpl-node.bin 0x08000000

up10: rpl-node.bin
	st-flash --serial $(NODE10) write rpl-node.bin 0x08000000

up11: rpl-node.bin
	st-flash --serial $(NODE11) write rpl-node.bin 0x08000000

up12: rpl-node.bin
	st-flash --serial $(NODE12) write rpl-node.bin 0x08000000

upDWMF: rpl-node.bin
	st-flash --serial $(NODE_DWM_F) write rpl-node.bin 0x08000000

upDWMM: rpl-node.bin
	st-flash --serial $(NODE_DWM_M) write rpl-node.bin 0x08000000

upDWMN: rpl-node.bin
	st-flash --serial $(NODE_DWM_N) write rpl-node.bin 0x08000000

#upClients: up11 upD up8 up5 upE
upClients: upD
upAll: upA upClients
