TARGETS = multi-lookup

CROSS_TOOL = 
CC_CPP = $(CROSS_TOOL)g++
CC_C = $(CROSS_TOOL)gcc

CFLAGS = -Wall -Wextra -pthread

all: clean $(TARGETS)

$(TARGETS):
	$(CC_C) $(CFLAGS) $@.c -o $@

clean:
	rm -f $(TARGETS)