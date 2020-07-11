all: sim

tb_pwm: tb_pwm.v ../src/pwm.v tb_pwm.v
	iverilog -o $@ $^

sim: tb_pwm
	vvp tb_pwm -lxt2

clean:
	rm -f tb_pwm *lx2
