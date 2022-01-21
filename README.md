# CPLD implementation for the sl28 board

An open source CPLD implementation for the Kontron SMARC-sAL28 board.

It features an I²C controller which supports multi byte reads and writes, a
glitch-free PWM, a blinking LED with two states (indicating failsafe mode),
GPIO controllers, interrupt controllers, a whopping 16 bits non-volatile
memory, a watchdog with automatic failsafe boot, power control and various
other glue logic.

All modules use the same clock domain which is clocked by the internal
oscillator. The frequency of this oscillator vary very much with the
temperature. Thus all timing dependent components like the watchdog might
be inaccurate.

## Building

Make sure quartus is in your PATH. Then build it using the following
command:

    make -C build clean all

## Simulation

You can simulate each module by using iverilog and gtkwave.

## Example fitter report

    +---------------------------------------------------+-------------+
    ; Compilation Hierarchy Node                        ; Logic Cells ;
    +---------------------------------------------------+-------------+
    ; |sl28_top                                         ; 565 (81)    ;
    ;    |async_negedge_latch_sync_out:power_off_latch| ; 3 (1)       ;
    ;       |sync_edge:sync_edge_latch|                 ; 2 (2)       ;
    ;    |cfg_ctrl_altera_ufm:cfg_ctrl|                 ; 48 (48)     ;
    ;       |altufm_none:ufm|                           ; 0 (0)       ;
    ;          |altufm_none_pmd:auto_generated|         ; 0 (0)       ;
    ;    |clockgen:clockgen|                            ; 35 (35)     ;
    ;    |config_trits:config_trits|                    ; 4 (4)       ;
    ;    |gpi:board_control|                            ; 1 (1)       ;
    ;    |gpi:gpi|                                      ; 2 (2)       ;
    ;    |gpio:gpio0|                                   ; 61 (45)     ;
    ;       |sync_edge:sync_edge_gpio|                  ; 16 (16)     ;
    ;    |gpio:gpio1|                                   ; 42 (33)     ;
    ;       |sync_edge:sync_edge_gpio|                  ; 9 (9)       ;
    ;    |gpo:gpo|                                      ; 7 (7)       ;
    ;    |gpo:misc_ctrl|                                ; 7 (7)       ;
    ;    |i2c_bus_reset:i2c_bus_gp_reset|               ; 5 (5)       ;
    ;    |i2c_bus_reset:i2c_bus_pm_reset|               ; 5 (5)       ;
    ;    |i2c_bus_reset:i2c_local_bus_reset|            ; 6 (6)       ;
    ;    |i2c_slave:i2c_slave|                          ; 57 (53)     ;
    ;       |sync_edge:sync_edge_scl|                   ; 2 (2)       ;
    ;       |sync_edge:sync_edge_sda|                   ; 2 (2)       ;
    ;    |intc:intc|                                    ; 23 (23)     ;
    ;    |power_fsm:power_fsm|                          ; 18 (18)     ;
    ;    |pwm:pwm0|                                     ; 33 (33)     ;
    ;    |pwm:pwm1|                                     ; 29 (29)     ;
    ;    |reset_req:reset_req|                          ; 8 (6)       ;
    ;       |async_negedge_latch_sync_out:latch|        ; 2 (1)       ;
    ;          |sync_edge:sync_edge_latch|              ; 1 (1)       ;
    ;    |sync_edge:sync_edge_batlow|                   ; 1 (1)       ;
    ;    |sync_edge:sync_edge_cfg_read_done|            ; 2 (2)       ;
    ;    |sync_edge:sync_edge_charger_prsnt|            ; 1 (1)       ;
    ;    |sync_edge:sync_edge_charging|                 ; 1 (1)       ;
    ;    |sync_edge:sync_edge_force_recov|              ; 1 (1)       ;
    ;    |sync_edge:sync_edge_lid|                      ; 1 (1)       ;
    ;    |sync_edge:sync_edge_pcie_wake|                ; 2 (2)       ;
    ;    |sync_edge:sync_edge_poreset|                  ; 2 (2)       ;
    ;    |sync_edge:sync_edge_power_btn|                ; 2 (2)       ;
    ;    |sync_edge:sync_edge_rtc_int|                  ; 2 (2)       ;
    ;    |sync_edge:sync_edge_sleep|                    ; 2 (2)       ;
    ;    |sync_edge:sync_edge_smb_alert|                ; 2 (2)       ;
    ;    |sync_edge:sync_edge_wol_int|                  ; 2 (2)       ;
    ;    |tacho:tacho|                                  ; 24 (24)     ;
    ;    |usbfixer:usbfixer|                            ; 2 (2)       ;
    ;    |watchdog:watchdog|                            ; 43 (43)     ;
    +---------------------------------------------------+-------------+

    +---------------------------------------------------------------------+
    ; Fitter Summary                                                      ;
    +-----------------------+---------------------------------------------+
    ; Fitter Status         ; Successful - Fri Jan 21 20:28:52 2022       ;
    ; Quartus Prime Version ; 20.1.0 Build 711 06/05/2020 SJ Lite Edition ;
    ; Revision Name         ; sl28cpld                                    ;
    ; Top-level Entity Name ; sl28_top                                    ;
    ; Family                ; MAX V                                       ;
    ; Device                ; 5M570ZM100I5                                ;
    ; Timing Models         ; Final                                       ;
    ; Total logic elements  ; 565 / 570 ( 99 % )                          ;
    ; Total pins            ; 69 / 74 ( 93 % )                            ;
    ; Total virtual pins    ; 0                                           ;
    ; UFM blocks            ; 1 / 1 ( 100 % )                             ;
    +-----------------------+---------------------------------------------+
