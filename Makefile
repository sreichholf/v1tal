# V1TAL — build the on-robot ARM binaries.
#
# The cross toolchain is NOT vendored. Put an extracted Bootlin
# armv7-eabihf--glibc--stable-2018.02-2 tree under artifacts/toolchain/,
# or point TOOLCHAIN at it. See artifacts/README.md and BUILD.md.

SHELL := /bin/bash
.DEFAULT_GOAL := all
.SUFFIXES:

TOOLCHAIN ?= artifacts/toolchain/armv7-eabihf--glibc--stable-2018.02-2
CROSS     ?= $(TOOLCHAIN)/bin/arm-linux-
SYSROOT   ?= $(TOOLCHAIN)/arm-buildroot-linux-gnueabihf/sysroot
CC_ARM    := $(CROSS)gcc
CXX_ARM   := $(CROSS)g++

S5_ROOTFS ?= artifacts/s5/rootfs
BUILD     ?= build
DIST      ?= dist
ROBOT     ?=
REBOOT    ?=

CFLAGS_C    := -O2 -Wall -Wextra -std=gnu11
CFLAGS_SHIM := -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Isrc

SHIM_SRCS := src/addon.c src/gates.S src/persist.c src/segedit.c
SHIM_HDRS := $(wildcard src/*.h)

S5_CLEANER := $(S5_ROOTFS)/opt/rockrobo/cleaner
S5_LIB     := $(S5_CLEANER)/lib
S5_USR_LIB := $(S5_ROOTFS)/usr/lib/arm-linux-gnueabihf
S5_CONF    := $(S5_CLEANER)/conf
KIT        := $(DIST)/kit
KIT_LIB    := $(KIT)/lib

.PHONY: all host kit clean help check-toolchain install uninstall status

all: $(BUILD)/libv1seg_addon.so $(BUILD)/v1seg $(BUILD)/v1appd $(BUILD)/v1guard $(BUILD)/reanchor

$(BUILD):
	mkdir -p $@

check-toolchain:
	@test -x "$(CC_ARM)" || { \
		echo "error: ARM C compiler not found: $(CC_ARM)"; \
		echo "       Put the Bootlin toolchain under artifacts/toolchain/ or set TOOLCHAIN=..."; \
		echo "       See artifacts/README.md."; \
		exit 1; }
	@test -x "$(CXX_ARM)" || { \
		echo "error: ARM C++ compiler not found: $(CXX_ARM)"; \
		echo "       Put the Bootlin toolchain under artifacts/toolchain/ or set TOOLCHAIN=..."; \
		echo "       See artifacts/README.md."; \
		exit 1; }

$(BUILD)/libv1seg_addon.so: $(SHIM_SRCS) $(SHIM_HDRS) | check-toolchain $(BUILD)
	$(CC_ARM) --sysroot="$(SYSROOT)" -shared -fPIC -fvisibility=hidden $(CFLAGS_SHIM) \
		-o $@ $(SHIM_SRCS) -lpthread

$(BUILD)/v1seg: src/v1seg.cpp src/crc32.h | check-toolchain $(BUILD)
	$(CXX_ARM) -O2 -Wall -std=gnu++11 -o $@ $< -ldl -lm

$(BUILD)/v1appd: src/v1appd.c | check-toolchain $(BUILD)
	$(CC_ARM) $(CFLAGS_C) -o $@ $<

$(BUILD)/v1guard: src/guard.c src/persist.c $(SHIM_HDRS) | check-toolchain $(BUILD)
	$(CC_ARM) $(CFLAGS_C) -Isrc -o $@ src/guard.c src/persist.c

$(BUILD)/reanchor: src/reanchor.c src/reanchor.h src/crc32.h | check-toolchain $(BUILD)
	$(CC_ARM) $(CFLAGS_C) -Isrc -o $@ src/reanchor.c -lm

# Host builds are compile-only sanity checks. The binaries keep their on-robot
# defaults (/mnt/data/rockrobo, /opt/rockrobo); do not run them as root.
host: $(BUILD)/host/v1appd $(BUILD)/host/v1guard $(BUILD)/host/reanchor

$(BUILD)/host:
	mkdir -p $@

$(BUILD)/host/v1appd: src/v1appd.c | $(BUILD)/host
	gcc $(CFLAGS_C) -o $@ $<

$(BUILD)/host/v1guard: src/guard.c src/persist.c $(SHIM_HDRS) | $(BUILD)/host
	gcc $(CFLAGS_C) -Isrc -o $@ src/guard.c src/persist.c

$(BUILD)/host/reanchor: src/reanchor.c src/reanchor.h src/crc32.h | $(BUILD)/host
	gcc $(CFLAGS_C) -Isrc -o $@ src/reanchor.c -lm

# Assemble the exact runtime kit the upstart jobs expect. The S5 payload is
# needed only for this target (roomseg + OpenCV/Boost + the DNN model).
kit: all
	@test -d "$(S5_ROOTFS)" || { \
		echo "error: S5 payload not found: $(S5_ROOTFS)"; \
		echo "       Run tools/unpack_s5.sh (see artifacts/README.md)."; \
		exit 1; }
	@for f in \
		"$(S5_LIB)/libroomseg.so.0.8.60" \
		"$(S5_LIB)/libopencv_core.so.3.4.2" \
		"$(S5_LIB)/libopencv_imgproc.so.3.4.2" \
		"$(S5_LIB)/libopencv_dnn.so.3.4.2" \
		"$(S5_USR_LIB)/libboost_system.so.1.55.0" \
		"$(S5_USR_LIB)/libboost_filesystem.so.1.55.0" \
		"$(S5_CONF)/roomseg.mdl.4"; do \
		test -f "$$f" || { echo "error: missing S5 payload: $$f"; exit 1; }; \
	done
	mkdir -p "$(KIT_LIB)" "$(DIST)/init" "$(DIST)/valetudo"
	cp $(BUILD)/v1appd $(BUILD)/v1seg $(BUILD)/v1guard $(BUILD)/reanchor \
		$(BUILD)/libv1seg_addon.so "$(KIT)/"
	cp "$(S5_CONF)/roomseg.mdl.4" "$(KIT)/roomseg.mdl"
	cp "$(S5_LIB)/libroomseg.so.0.8.60" "$(KIT_LIB)/"
	cp "$(S5_LIB)/libopencv_core.so.3.4.2" "$(KIT_LIB)/"
	cp "$(S5_LIB)/libopencv_imgproc.so.3.4.2" "$(KIT_LIB)/"
	cp "$(S5_LIB)/libopencv_dnn.so.3.4.2" "$(KIT_LIB)/"
	cp "$(S5_USR_LIB)/libboost_system.so.1.55.0" "$(KIT_LIB)/"
	cp "$(S5_USR_LIB)/libboost_filesystem.so.1.55.0" "$(KIT_LIB)/"
	ln -sfn libroomseg.so.0.8.60              "$(KIT_LIB)/libroomseg.so.0"
	ln -sfn libroomseg.so.0                   "$(KIT_LIB)/libroomseg.so"
	ln -sfn libopencv_core.so.3.4.2           "$(KIT_LIB)/libopencv_core.so.3.4"
	ln -sfn libopencv_core.so.3.4             "$(KIT_LIB)/libopencv_core.so"
	ln -sfn libopencv_imgproc.so.3.4.2        "$(KIT_LIB)/libopencv_imgproc.so.3.4"
	ln -sfn libopencv_imgproc.so.3.4          "$(KIT_LIB)/libopencv_imgproc.so"
	ln -sfn libopencv_dnn.so.3.4.2            "$(KIT_LIB)/libopencv_dnn.so.3.4"
	ln -sfn libopencv_dnn.so.3.4              "$(KIT_LIB)/libopencv_dnn.so"
	ln -sfn libboost_system.so.1.55.0         "$(KIT_LIB)/libboost_system.so"
	ln -sfn libboost_filesystem.so.1.55.0     "$(KIT_LIB)/libboost_filesystem.so"
	cp init/*.conf "$(DIST)/init/"
	cp valetudo/RoborockV1SegmentedValetudoRobot.js valetudo/UPSTREAM_PATCH.diff "$(DIST)/valetudo/"
	cd "$(KIT)" && { \
		sha256sum v1appd v1seg v1guard reanchor libv1seg_addon.so roomseg.mdl; \
		find lib -maxdepth 1 -type f -print0 | sort -z | xargs -0 sha256sum; \
	} > SHA256SUMS

install: $(if $(ROBOT),kit,)
	@[ -n "$(ROBOT)" ] || { echo "usage: make install ROBOT=root@host"; exit 2; }
	tools/install.sh $(if $(filter 1,$(REBOOT)),--reboot,) "$(ROBOT)"

uninstall:
	@[ -n "$(ROBOT)" ] || { echo "usage: make uninstall ROBOT=root@host"; exit 2; }
	tools/install.sh --uninstall $(if $(filter 1,$(REBOOT)),--reboot,) "$(ROBOT)"

status:
	@[ -n "$(ROBOT)" ] || { echo "usage: make status ROBOT=root@host"; exit 2; }
	tools/install.sh --status "$(ROBOT)"

clean:
	rm -rf "$(BUILD)" "$(DIST)"

help:
	@echo "V1TAL build targets:"
	@echo "  make            build the ARM deliverables (default)"
	@echo "  make host       build host-side compile checks (gcc)"
	@echo "  make kit        stage the runtime kit + init jobs + Valetudo inputs"
	@echo "  make install    install/update on ROBOT=root@host"
	@echo "  make status     health check on ROBOT=root@host"
	@echo "  make uninstall  remove V1TAL from ROBOT=root@host"
	@echo "  make clean      remove build/ and dist/"
	@echo ""
	@echo "Variables:"
	@echo "  TOOLCHAIN=...   Bootlin toolchain dir (default: artifacts/toolchain/...)"
	@echo "  S5_ROOTFS=...   extracted S5 payload (default: artifacts/s5/rootfs)"
	@echo "  ROBOT=...       ssh target for install/status/uninstall"
	@echo "  REBOOT=1        skip the reboot prompt"
