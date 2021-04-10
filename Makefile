# ========================================================================================
# Makefile
# ========================================================================================

# ========================================================================================
# Compile flags

#  Rpi1 x-compile uses https://github.com/raspberrypi/tools

CC = gcc -march=native
XRPICC = ~/Tools/rpi/arm-bcm2708/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc \
			-marm -march=armv6zk -mcpu=arm1176jzf-s -mfpu=vfp
COPT = -O2
CFLAGS = -Wall -Wextra -Wpedantic -Werror -std=gnu11 -D_GNU_SOURCE
CFLAGS += -D BUILD_VERSION="\"$(shell git describe --dirty --always)\""	\
		-D BUILD_DATE="\"$(shell date '+%Y-%m-%d_%H:%M:%S')\""

BIN = jammon

# ========================================================================================
# Source files

SRCDIR = .

SRC = $(SRCDIR)/main.c \
		$(SRCDIR)/telemetry.c \
		$(SRCDIR)/cmp.c

# ========================================================================================
# External Libraries

LIBSDIR = 
LIBS = 

# ========================================================================================
# Makerules

all:
	$(CC) $(COPT) $(CFLAGS) $(SRC) -o $(BIN) $(LIBSDIR) $(LIBS)
cross2rpi:
	$(XRPICC) $(COPT) $(CFLAGS) $(SRC) -o $(BIN) $(LIBSDIR) $(LIBS)

debug: COPT = -Og -gdwarf -fno-omit-frame-pointer -D__DEBUG
debug: all

clean:
	rm -fv *.o $(BIN)
