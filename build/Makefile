PROJECT = sl28cpld
TOP_LEVEL_ENTITY = sl28_top

FAMILY = MAX V
PART = 5M570ZM100I5
BOARDFILE = pins

SRCS = top.v

$(PROJECT).qpf $(PROJECT).qsf:
	quartus_sh --prepare -f "$(FAMILY)" -t "$(TOP_LEVEL_ENTITY)" $(PROJECT)
	-cat $(BOARDFILE) >> $(PROJECT).qsf

$(PROJECT).map.rpt: map.chg $(SOURCE_FILES)
	quartus_map --read_settings_files=on $(addprefix --source=,$(SRCS)) $(PROJECT)
	touch fit.chg

$(PROJECT).fit.rpt: fit.chg $(PROJECT).map.rpt
	quartus_fit --read_settings_files=on --part=$(PART)
	touch asm.chg
	touch sta.chg

$(PROJECT).asm.rpt: asm.chg $(PROJECT).fit.rpt
	quartus_asm $(PROJECT)

$(PROJECT).sta.rpt: sta.chg $(PROJECT).fit.rpt
	quartus_sta $(PROJECT)

map.chg fit.chg sta.chg asm.chg:
	touch $@
