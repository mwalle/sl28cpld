// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * USB fixer module.
 *
 * The SMARC standard combines both the enable and the fault signal into
 * one signal. But virtually all SoCs and USB controllers need them as
 * different signals.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module usbfixer (
	input enable,
	inout usb_en_oc_n,
	input usb_drvvbus,
	output usb_pwrfault
);

reg pwrfault;
always @(negedge usb_en_oc_n, negedge usb_drvvbus)
	if (!usb_drvvbus)
		pwrfault <= 1'b0;
	else
		pwrfault <= 1'b1;

assign usb_pwrfault = enable ? pwrfault : 1'b0;
assign usb_en_oc_n = !enable | (usb_drvvbus & !usb_pwrfault) ? 1'bz : 1'b0;

endmodule
