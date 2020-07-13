PROJECT = sl28cpld
TOP_LEVEL_ENTITY = sl28_top

FAMILY = MAX V
PART = 5M570ZM100I5
BOARDFILE = pins

SRCS = ../src/top.v ../src/i2c_slave.v ../src/pwm.v ../src/gpio.v

all: $(PROJECT).jbc

map: $(PROJECT).map.rpt
fit: $(PROJECT).fit.rpt
asm: $(PROJECT).asm.rpt
sta: $(PROJECT).sta.rpt

.PHONY: smart.log
smart.log: $(PROJECT).qpf $(PROJECT).qsf
	quartus_sh --determine_smart_action $(PROJECT) > $@

$(PROJECT).qpf $(PROJECT).qsf: $(BOARDFILE)
	quartus_sh --prepare -f "$(FAMILY)" -t "$(TOP_LEVEL_ENTITY)" $(PROJECT)
	-echo >> $(PROJECT).qsf
	-cat $(BOARDFILE) >> $(PROJECT).qsf

$(PROJECT).map.rpt: smart.log $(PROJECT).qsf map.chg $(SOURCE_FILES)
	quartus_map --read_settings_files=on $(addprefix --source=,$(SRCS)) $(PROJECT)
	touch fit.chg

$(PROJECT).fit.rpt: smart.log $(PROJECT).qsf fit.chg $(PROJECT).map.rpt
	quartus_fit --read_settings_files=on --part=$(PART) $(PROJECT)
	touch asm.chg
	touch sta.chg

$(PROJECT).asm.rpt: smart.log $(PROJECT).qsf asm.chg $(PROJECT).fit.rpt
	quartus_asm $(PROJECT)

$(PROJECT).sta.rpt: smart.log $(PROJECT).qsf sta.chg $(PROJECT).fit.rpt
	quartus_sta $(PROJECT)

$(PROJECT).jam $(PROJECT).jbc: $(PROJECT).asm.rpt
	quartus_cpf -c $(PROJECT).pof $@

map.chg fit.chg sta.chg asm.chg:
	touch $@

clean:
	rm -rf db/ incremental_db/ *.rpt *.chg *.eqn *.sof *.pof *.summary *.qpf *.smsg *.jdi *.jbc *.jam *.pin *.qsf *.sld smart.log
