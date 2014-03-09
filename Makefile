#MCU_TARGET     = at90s2313
#MCU_TARGET     = at90s2333
#MCU_TARGET     = at90s4414
#MCU_TARGET     = at90s4433
#MCU_TARGET     = at90s4434
#MCU_TARGET     = at90s8515
#MCU_TARGET     = at90s8535
#MCU_TARGET     = atmega128
#MCU_TARGET     = atmega1280
#MCU_TARGET     = atmega1281
#MCU_TARGET     = atmega1284p
#MCU_TARGET     = atmega16
#MCU_TARGET     = atmega163
#MCU_TARGET     = atmega164p
#MCU_TARGET     = atmega165
#MCU_TARGET     = atmega165p
#MCU_TARGET     = atmega168
#MCU_TARGET 	= atmega328p
#MCU_TARGET     = atmega169
#MCU_TARGET     = atmega169p
MCU_TARGET     = atmega2560
#MCU_TARGET     = atmega2561
#MCU_TARGET     = atmega32
#MCU_TARGET     = atmega324p
#MCU_TARGET     = atmega325
#MCU_TARGET     = atmega3250
#MCU_TARGET     = atmega329
#MCU_TARGET     = atmega3290
#MCU_TARGET     = atmega48
#MCU_TARGET     = atmega64
#MCU_TARGET     = atmega640
#MCU_TARGET     = atmega644
#MCU_TARGET     = atmega644p
#MCU_TARGET     = atmega645
#MCU_TARGET     = atmega6450
#MCU_TARGET     = atmega649
#MCU_TARGET     = atmega6490
#MCU_TARGET     = atmega8
#MCU_TARGET     = atmega8515
#MCU_TARGET     = atmega8535
#MCU_TARGET     = atmega88
#MCU_TARGET     = attiny2313
#MCU_TARGET     = attiny24
#MCU_TARGET     = attiny25
#MCU_TARGET     = attiny26
#MCU_TARGET     = attiny261
#MCU_TARGET     = attiny44
#MCU_TARGET     = attiny45
#MCU_TARGET     = attiny461
#MCU_TARGET     = attiny84
#MCU_TARGET     = attiny85
#MCU_TARGET     = attiny861

LIBNAME		= librtos.a
OPTIMIZE	= -O2
DEFS		=  F_CPU=16000000 __AVR_ATmega2560__ GCC_MEGA_AVR MCU_TARGET=__$(MCU_TARGET)__
INCLUDE		=  include portable

CC			= avr-gcc
AR			= avr-ar

# Sources located at the top of the source tree.
# Other sources are included by .mk files

sources := croutine.c event_groups.c list.c queue.c tasks.c timers.c

# Make a list of object files
cfiles = $(filter %.c,$(sources))
sfiles = $(filter %.S,$(sources))

cobjs = $(subst .c,.o,$(cfiles))
sobjs = $(subst .S,.o,$(sfiles))
 
objs = $(cobjs)
objs += $(sobjs)

# override %.S rule to use our CC and CFLAGS

%.o:	%.S	
	$(CC) $(CFLAGS) -c $< -o $@ 


# Compiler flags
override CFLAGS        = --std=gnu99 -ffunction-sections -fdata-sections -mcall-prologues -mrelax -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(addprefix -D,$(DEFS)) $(addprefix -I,$(INCLUDE))


# .mk includes in subdirectories
include portable/portable.mk
include MemMang/memmang.mk
include lib_io/lib_io.mk
include lib_hd44780/lib_hd44780.mk
include lib_time/lib_time.mk
include lib-uIP/lib-uIP.mk
include lib_iinchip/lib_iinchip.mk
include lib_rtc/lib_rtc.mk
include lib_ft800/lib_ft800.mk

.PHONY:	all clean
	
all: $(objs)
	$(AR) rcs $(LIBNAME) $(objs)
	

clean:
	rm -f $(objs) $(LIBNAME)



