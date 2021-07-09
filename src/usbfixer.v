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
