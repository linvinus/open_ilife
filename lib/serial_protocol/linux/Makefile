CC=gcc
CFLAGS =-Wall -Wextra  -O0 -g -I . -I .. -I ../.. -I../../COBS

SRCS = $(wildcard *.c) ../serial_protocol.c ../../COBS/cobs.c
OBJS = $(SRCS:%.c=%.o)
EXE = main

all: $(EXE)

$(EXE): $(OBJS)

-include $(subst .c,.d,$(SRCS))

%.d: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -MM -MF $@ -MP -MT $(subst .c,.o,$<) $<

clean:
	rm -f $(EXE)
	rm -f $(OBJS)
	rm -f $(subst .c,.d,$(SRCS))
	rm -f *~

.PHONY: clean all
