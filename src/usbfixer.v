module usbfixer (
	inout usb_en_oc_n,
	input usb_drvvbus,
	output reg usb_pwrfault
);

always @(negedge usb_en_oc_n, negedge usb_drvvbus)
	if (!usb_drvvbus)
		usb_pwrfault <= 1'b0;
	else
		usb_pwrfault <= 1'b1;
assign usb_en_oc_n = usb_drvvbus ? 1'bz : 1'b0;

endmodule
