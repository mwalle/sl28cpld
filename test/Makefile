all: sim

tb_pwm: tb_pwm.v ../src/pwm.v
	iverilog -o $@ $^

tb_i2c_slave: tb_i2c_slave.v ../src/i2c_slave.v
	iverilog -o $@ $^

sim: tb_pwm tb_i2c_slave
	vvp tb_pwm -lxt2
	vvp tb_i2c_slave -lxt2

clean:
	rm -f tb_pwm tb_i2c_slave *lx2
