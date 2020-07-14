all: sim

tb_clockgen: tb_clockgen.v ../src/clockgen.v
tb_pwm: tb_pwm.v sys.v csr.v ../src/pwm.v
tb_i2c_slave: tb_i2c_slave.v sys.v ../src/i2c_slave.v
tb_gpio: tb_gpio.v sys.v csr.v ../src/gpio.v
tb_watchdog: tb_watchdog.v sys.v csr.v ../src/watchdog.v
tb_cfg_ctrl_altera_ufm: tb_cfg_ctrl_altera_ufm.v sys.v csr.v ../src/cfg_ctrl_altera_ufm.v

tb_clockgen tb_pwm tb_i2c_slave tb_gpio tb_watchdog:
	iverilog -o $@ $(filter-out sys.v csr.v,$^)

sim: tb_clockgen tb_pwm tb_i2c_slave tb_gpio tb_watchdog tb_cfg_ctrl_altera_ufm
	vvp tb_clockgen -lxt2
	vvp tb_pwm -lxt2
	vvp tb_i2c_slave -lxt2
	vvp tb_gpio -lxt2
	vvp tb_watchdog -lxt2
	vvp tb_cfg_ctrl_altera_ufm -lxt2

clean:
	rm -f tb_clockgen tb_pwm tb_i2c_slave tb_gpio tb_watchdog tb_cfg_ctrl_altera_ufm *lx2
