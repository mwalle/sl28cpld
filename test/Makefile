all: sim

tb_clockgen: tb_clockgen.v ../src/clockgen.v
tb_pwm: tb_pwm.v sys.v csr.v ../src/pwm.v
tb_i2c_slave: tb_i2c_slave.v sys.v ../src/i2c_slave.v
tb_gpio: tb_gpio.v sys.v csr.v ../src/gpio.v

tb_clockgen tb_pwm tb_i2c_slave tb_gpio:
	iverilog -o $@ $(filter-out sys.v csr.v,$^)

sim: tb_clockgen tb_pwm tb_i2c_slave tb_gpio
	vvp tb_clockgen -lxt2
	vvp tb_pwm -lxt2
	vvp tb_i2c_slave -lxt2
	vvp tb_gpio -lxt2

clean:
	rm -f tb_clockgen tb_pwm tb_i2c_slave tb_gpio *lx2
