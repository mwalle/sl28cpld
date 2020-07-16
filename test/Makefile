all: sim

tb_clockgen: tb_clockgen.v ../src/clockgen.v
tb_pwm: tb_pwm.v sys.v csr.v ../src/pwm.v
tb_i2c_slave: tb_i2c_slave.v sys.v ../src/i2c_slave.v
tb_gpio: tb_gpio.v sys.v csr.v ../src/gpio.v
tb_watchdog: tb_watchdog.v sys.v csr.v slowclock.v ../src/watchdog.v
tb_i2c_bus_reset: tb_i2c_bus_reset.v sys.v slowclock.v ../src/i2c_bus_reset.v
tb_power_fsm: tb_power_fsm.v sys.v slowclock.v ../src/power_fsm.v
tb_cfg_ctrl_altera_ufm: tb_cfg_ctrl_altera_ufm.v sys.v csr.v ../src/cfg_ctrl_altera_ufm.v

tb_clockgen tb_pwm tb_i2c_slave tb_gpio tb_watchdog tb_i2c_bus_reset:
tb_power_fsm:
	iverilog -o $@ $(filter-out sys.v csr.v slowclock.v,$^)

sim: sim_clockgen sim_pwm sim_i2c_slave sim_gpio sim_watchdog \
	 sim_i2c_bus_reset sim_power_fsm

sim_%: tb_%
	vvp $^ -lxt2

clean:
	rm -f tb_clockgen tb_pwm tb_i2c_slave tb_gpio tb_watchdog \
	tb_i2c_bus_reset tb_power_fsm *lx2
