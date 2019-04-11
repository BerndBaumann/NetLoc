CONTIKI_PROJECT = collect-test
all: $(CONTIKI_PROJECT)

CFLAGS += -u _printf_float

# Add stm32nucleo platform folder
TARGETDIRS += ../../platform/stm32nucleo-dw1000
DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

# No network stack for the moment
CONTIKI_WITH_IPV4 = 0
CONTIKI_WITH_IPV6 = 1
CONTIKI_WITH_RIME = 1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

upA: $(CONTIKI_PROJECT).bin
	st-flash --serial 303637344646353435313532373137 write $(CONTIKI_PROJECT).bin 0x08000000

upB: $(CONTIKI_PROJECT).bin
	st-flash --serial 303637304646343835373534373237 write $(CONTIKI_PROJECT).bin 0x08000000
