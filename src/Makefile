CC          := gcc
INCLUDE_DIRS:= -I.                       # current directory for headers
CDEFS       :=
CFLAGS      := -march=armv8-a+simd -O3 -g -std=c11 -Wall -Werror $(INCLUDE_DIRS) $(CDEFS)

LIBS        := -pthread -lrt             # rt for clock_gettime

# --------------------------------------------------------------------
#  Source / object lists
# --------------------------------------------------------------------
HFILES      :=
CFILES      := capture.c
OBJS        := $(CFILES:.c=.o)

# --------------------------------------------------------------------
#  Build rules
# --------------------------------------------------------------------
all: capture

capture: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LIBS)

%.o: %.c $(HFILES)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) -f *.o *.d capture *.ppm *.pgm
	$(RM) -rf /dev/mqueue/cap_to_det_mq
	$(RM) -rf /dev/mqueue/det_to_save_mq

distclean: clean
