
"""
Created on Tue 18 Jun 2024

@author: ASA2, GDR
"""

from RDK4Driver import RDK4Functions

rdk4 = RDK4Functions()
rdk4.connect()
rdk4.set_uart_mode()
rdk4.osp_reset()
rdk4.osp_init_bidir(1)
rdk4.osp_clearerror(0)
rdk4.osp_go_active(0)
rdk4.osp_set_current_channel(3, 0, sync= 0, hyb = 0, dith= 0, rcurr="3mA", gcurr ="3mA", bcurr ="3mA")
rdk4.osp_set_current_channel(3, 1, sync= 0, hyb = 0, dith= 0, rcurr="3mA", gcurr ="3mA", bcurr ="3mA")
rdk4.osp_setpwm(3, 0, 0x3FFF, 0x0000, 0x0000)
rdk4.osp_setpwm(3, 0, 0x0000, 0x3FFF, 0x0000)
rdk4.osp_setpwm(3, 0, 0x0000, 0x0000, 0x3FFF)
rdk4.osp_setpwm_os(1,0x0000,0x0000,2048,0000)
rdk4.osp_setpwm_os(2,2048,0x0000,0x0000,0000)
rdk4.osp_setpwm_os(4,0x0000,2048,0x0000,0000)
rdk4.osp_setpwm_os(5,0x0000,0x0000,2048,0000)
rdk4.osp_setpwm(3, 1, 0x3FFF, 0x0000, 0x0000)
rdk4.osp_setpwm(3, 1, 0x0000, 0x3FFF, 0x0000)
rdk4.osp_setpwm(3, 1, 0x0000, 0x0000, 0x3FFF)
rdk4.close()

# rdk4.osp_setpwm(3, 2, 0, 0, 16383)
# hex(rdk4.osp_identify(3))
# rdk4.osp_setpwm_os(8,2048,2048,2048,0000)
# rdk4.osp_readtemp(4)
# rdk4.osp_readtempstat(2)
# rdk4.osp_read_current_channel(3,0)
# rdk4.osp_set_current_channel(3,0, sync= 0, hyb = 0, dith= 0, rcurr= "24mA", gcurr = "24mA", bcurr = "24mA")
# rdk4.osp_set_current_channel(3,1, sync= 0, hyb = 0, dith= 0, rcurr= "24mA", gcurr = "24mA", bcurr = "24mA")
# rdk4.close()


