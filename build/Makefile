PROJECT = sl28cpld
TOP_LEVEL_ENTITY = sl28_top
SEED ?= 0

FAMILY = MAX V
PART = 5M570ZM100I5
BOARDFILE = pins

GIT_COMMIT = $(shell echo $$(git describe --always --abbrev=7 --dirty=d)0 | cut -b1-8)

SRCS  = ../src/top.v ../src/i2c_slave.v ../src/pwm.v ../src/gpio.v
SRCS += ../src/gpi.v ../src/gpo.v ../src/tacho.v ../src/usbfixer.v
SRCS += ../src/clockgen.v ../src/watchdog.v ../src/cfg_ctrl_altera_ufm.v
SRCS += ../src/i2c_bus_reset.v ../src/power_fsm.v
SRCS += ../src/intc.v ../src/sync_edge.v ../src/reset_req.v
SRCS += ../src/async_negedge_latch_sync_out.v ../src/config_trits.v

all: $(PROJECT).jbc $(PROJECT).jam summary

map: $(PROJECT).map.rpt
fit: $(PROJECT).fit.rpt
asm: $(PROJECT).asm.rpt
sta: $(PROJECT).sta.rpt

.PHONY: summary
summary: $(PROJECT).fit.summary
	@echo "*** SUMMARY ***"
	@cat $(PROJECT).fit.summary

.PHONY: smart.log
smart.log: $(PROJECT).qpf $(PROJECT).qsf
	@quartus_sh --determine_smart_action $(PROJECT) > $@

$(PROJECT).qpf $(PROJECT).qsf: $(BOARDFILE)
	@quartus_sh --prepare -f "$(FAMILY)" -t "$(TOP_LEVEL_ENTITY)" $(PROJECT)
	@quartus_sh --set STRATIX_JTAG_USER_CODE=$(GIT_COMMIT) $(PROJECT)
	@quartus_sh --set USE_CHECKSUM_AS_USERCODE=OFF $(PROJECT)
	@echo >> $(PROJECT).qsf
	@cat $(BOARDFILE) >> $(PROJECT).qsf

$(PROJECT).map.rpt: $(PROJECT).qsf map.chg $(SOURCE_FILES)
	@quartus_map --read_settings_files=on $(addprefix --source=,$(SRCS)) $(PROJECT)
	@touch fit.chg

$(PROJECT).fit.rpt: $(PROJECT).qsf fit.chg $(PROJECT).map.rpt
	@quartus_fit --read_settings_files=on --part=$(PART) --seed=$(SEED) $(PROJECT)
	@touch asm.chg
	@touch sta.chg

$(PROJECT).asm.rpt: $(PROJECT).qsf asm.chg $(PROJECT).fit.rpt
	@quartus_asm $(PROJECT)

$(PROJECT).sta.rpt: $(PROJECT).qsf sta.chg $(PROJECT).fit.rpt
	@quartus_sta $(PROJECT)

$(PROJECT).jam $(PROJECT).jbc: $(PROJECT).asm.rpt
	@quartus_cpf -c $(PROJECT).pof $@

map.chg fit.chg sta.chg asm.chg:
	@touch $@

clean:
	rm -rf db/ incremental_db/ *.rpt *.chg *.eqn *.sof *.pof *.summary *.qpf *.smsg *.jdi *.jbc *.jam *.pin *.qsf *.sld smart.log
