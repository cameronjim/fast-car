# tools/jetson_heartbeat -- see README.md for what this is and how to install it.
#
# Two independent build products:
#   racer-heartbeat        the actual GPIO-toggling binary. Needs libgpiod-dev
#                           (`sudo apt-get install libgpiod-dev`); built and run ON THE
#                           JETSON, not in this repo's dev containers.
#   jetson_heartbeat_tests the host-runnable unit tests (src/config.c + tests/), no
#                           libgpiod dependency, buildable anywhere with plain gcc -- this
#                           is what .github/scripts/jetson_heartbeat_host_tests.sh runs.
#
# `make install` (run ON THE JETSON, as root) lays down the binary, the systemd unit, and the
# /etc/default environment file the unit reads its GPIO chip/line/rate from -- see
# racer-heartbeat.default. It never overwrites an existing environment file: a fresh install
# gets the documented defaults, but a re-install after the pin has been chosen and the file
# hand-edited must not silently revert that choice back to the pin-undecided defaults.
CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -Wpedantic -O2
INCLUDES = -Iinclude

PREFIX ?= /opt/racer/jetson_heartbeat
SYSTEMD_UNIT_DIR ?= /etc/systemd/system
ENV_FILE_DIR ?= /etc/default

.PHONY: all test clean install

all: racer-heartbeat

racer-heartbeat: src/main.c src/config.c include/jetson_heartbeat/config.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ src/main.c src/config.c $$(pkg-config --cflags --libs libgpiod)

jetson_heartbeat_tests: src/config.c tests/main.c tests/test_heartbeat_config.c tests/framework.h
	$(CC) $(CFLAGS) $(INCLUDES) -Itests -o $@ src/config.c tests/main.c tests/test_heartbeat_config.c

test: jetson_heartbeat_tests
	./jetson_heartbeat_tests

# Installs the binary and the systemd unit unconditionally (they are not operator-editable
# state), but only SEEDS the environment file if one is not already there -- `cp -n` is a
# no-op, not an error, when the destination exists, so re-running `make install` after the
# pin has been chosen and the file edited leaves that edit alone.
install: racer-heartbeat
	install -d $(PREFIX)
	install -m 0755 racer-heartbeat $(PREFIX)/racer-heartbeat
	install -d $(SYSTEMD_UNIT_DIR)
	install -m 0644 systemd/racer-heartbeat.service $(SYSTEMD_UNIT_DIR)/racer-heartbeat.service
	install -d $(ENV_FILE_DIR)
	cp -n racer-heartbeat.default $(ENV_FILE_DIR)/racer-heartbeat
	systemctl daemon-reload
	@echo "installed. Review $(ENV_FILE_DIR)/racer-heartbeat, then:"
	@echo "  sudo systemctl enable --now racer-heartbeat.service"

clean:
	rm -f racer-heartbeat jetson_heartbeat_tests
