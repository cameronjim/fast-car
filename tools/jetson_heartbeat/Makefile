# tools/jetson_heartbeat -- see README.md for what this is and how to install it.
#
# Two independent build products:
#   racer-heartbeat        the actual GPIO-toggling binary. Needs libgpiod-dev
#                           (`sudo apt-get install libgpiod-dev`); built and run ON THE
#                           JETSON, not in this repo's dev containers.
#   jetson_heartbeat_tests the host-runnable unit tests (src/config.c + tests/), no
#                           libgpiod dependency, buildable anywhere with plain gcc -- this
#                           is what .github/scripts/jetson_heartbeat_host_tests.sh runs.
CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Wpedantic -O2
INCLUDES = -Iinclude

.PHONY: all test clean

all: racer-heartbeat

racer-heartbeat: src/main.c src/config.c include/jetson_heartbeat/config.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ src/main.c src/config.c $$(pkg-config --cflags --libs libgpiod)

jetson_heartbeat_tests: src/config.c tests/main.c tests/test_heartbeat_config.c tests/framework.h
	$(CC) $(CFLAGS) $(INCLUDES) -Itests -o $@ src/config.c tests/main.c tests/test_heartbeat_config.c

test: jetson_heartbeat_tests
	./jetson_heartbeat_tests

clean:
	rm -f racer-heartbeat jetson_heartbeat_tests
