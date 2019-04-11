# NetLoc
NetLoc: A UWB-based Localization System for the IoT

# Bernd Baumann's Master's Thesis


#######################################################################
## HARDWARE

### Altium 17 Project Files

- extension_shield.PrjPcb (Main Project File)
- PCB2.PcbDoc (PCB Design)
- DW1000.SchDoc (DW1000 Schematic)
- Pinheader.SchDoc (Pinheader, LEDs, Jumper Schematic)

#######################################################################
## Embedded Software (Contiki)
Toolchain: 
arm-none-eabi-gcc 8.3

### sw-embedded/examples/rng_nucleo_calib)


### sw-embedded/examples/ranging


### sw-embedded/examples/ipv6/rpl-udp


### sw-embedded/examples/ranging-nucleo


### sw-embedded/examples/range-collect


### sw-embedded/examples/ipv6/rpl-uwb


#######################################################################
## PC Software

requirements:
- Python 3.7.3
- matplotlib 3.0.3
- scipy 1.2.1
- numpy 1.16.2

### sw-pc/testdata/labtest.log


### sw-pc/why_so_serial.py


### sw-pc/draw_graph.py


### sw-pc/cdf_analysis.py


