all: example_wireless_communication_unicast_RX example_wireless_communication_unicast_TX

# Add stm32nucleo platform folder
TARGETDIRS += ../../platform/stm32nucleo-dw1000

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

CONTIKI_WITH_RIME = 0
CONTIKI_WITH_IPV6 = 1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include


NODEA=303637344646353435313532373137
NODEB=303637304646343835373534373237
NODEC=303636394646353434383534363737
NODED=303636414646353434383534363737
NODE2=303636414646333333303336343334
NODE3=303636454646333333303336343334
NODE5=303636434646333333303336343334
NODE6=303636384646353434383534363737
NODE8=303636464646353435313532373137

upTx: example_wireless_communication_unicast_TX.bin
	st-flash --serial $(NODE2) write example_wireless_communication_unicast_TX.bin 0x08000000

upRx: example_wireless_communication_unicast_RX.bin
	st-flash --serial $(NODE5) write example_wireless_communication_unicast_RX.bin 0x08000000

upBoth: upTx upRx
