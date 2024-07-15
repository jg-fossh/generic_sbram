PROJ = generic_sbram
#PIN_DEF = up5k
#DEVICE = up5k
#PACKAGE = sg48
PIN_DEF = hx8k
DEVICE = hx8k
PACKAGE = ct256
PLACE_AND_ROUTE = nextpnr-ice40
SEED_0 = 0

all: clean build assertions

assertions: 
	egrep -w 'Error-Type|Error-Msg' $(PROJ)_$(DEVICE)_syn.log

syn:
	yosys -ql $(PROJ)_$(DEVICE)_syn.log syn_ice40.ys $^

map:
	yosys -ql $(PROJ)_$(DEVICE)_syn.log map_ice40.ys $^

pnr: map 
	$(PLACE_AND_ROUTE) --$(DEVICE) --package $(PACKAGE) --json $(PROJ)_syn.json --pcf $(PIN_DEF).pcf --pcf-allow-unconstrained --timing-allow-fail --ignore-loops --pre-pack pre_pack.py --opt-timing --seed $(SEED_0) --asc $(PROJ)_$(SEED_0).asc --report critical_path_rpt$(SEED_0).json

rpt: pnr
	icetime -d $(DEVICE) -p $(PIN_DEF).pcf -P $(PACKAGE) -c 12 -mtr Timming_Report_$(SEED_0).rpt $(PROJ)_$(SEED_0).asc

build: rpt
	icepack $(PROJ)_$(SEED_0).asc $(PROJ).bin

prog: $(PROJ).bin
	iceprog $<

sudo-prog: $(PROJ).bin
	@echo 'Executing prog as root!!!'
	sudo iceprog $<

clean:
	rm -f $(PROJ)_syn.blif $(PROJ)_syn.edif $(PROJ).asc $(PROJ).rpt $(PROJ).bin $(PROJ)_syn.json $(PROJ)_$(DEVICE)_syn.log $(PROJ)_*.asc Timming_Report_* critical_path_rpt*.json

.SECONDARY:
.PHONY: all syn map pnr rpt build prog sudo-prog assertions clean
