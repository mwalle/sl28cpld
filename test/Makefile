all: sim

tb_clockgen: tb_clockgen.v ../src/clockgen.v
tb_pwm: tb_pwm.v sys.v csr.v ../src/pwm.v
tb_i2c_slave: tb_i2c_slave.v sys.v ../src/sync_edge.v ../src/i2c_slave.v
tb_gpio: tb_gpio.v sys.v csr.v ../src/sync_edge.v ../src/gpio.v
tb_watchdog: tb_watchdog.v sys.v csr.v slowclock.v ../src/watchdog.v
tb_i2c_bus_reset: tb_i2c_bus_reset.v sys.v slowclock.v ../src/i2c_bus_reset.v
tb_power_fsm: tb_power_fsm.v sys.v slowclock.v ../src/power_fsm.v
tb_reset_req: tb_reset_req.v sys.v slowclock.v ../src/sync_edge.v ../src/async_negedge_latch_sync_out.v ../src/reset_req.v
tb_cfg_ctrl_altera_ufm: tb_cfg_ctrl_altera_ufm.v sys.v csr.v altufm_none.v ../src/cfg_ctrl_altera_ufm.v
tb_top: tb_top.v altufm_none.v $(wildcard ../src/*.v)

tb_%:
	iverilog -o $@ $(filter-out sys.v csr.v slowclock.v,$^)

%.lx2: tb_%
	vvp $^ -lxt2

sim: clockgen.lx2 pwm.lx2 i2c_slave.lx2 gpio.lx2 watchdog.lx2 \
	 i2c_bus_reset.lx2 power_fsm.lx2 reset_req.lx2 cfg_ctrl_altera_ufm.lx2 \
	 top.lx2

clean:
	rm -f tb_clockgen tb_pwm tb_i2c_slave tb_gpio tb_watchdog \
	tb_i2c_bus_reset tb_power_fsm tb_reset_req tb_cfg_ctrl_altera_ufm *lx2
