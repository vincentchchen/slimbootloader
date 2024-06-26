/** @file
  ACPI RTD3 SSDT table

  Copyright (c) 2024, Intel Corporation. All rights reserved.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

External(\_SB.OSCO)
External(\_SB.PC00,DeviceObj)
External(\_SB.PC00.SAT0, DeviceObj)
External(\_SB.PC00.SAT0.PRT0, DeviceObj)
External(\_SB.PC00.SAT0.PRT1, DeviceObj)
External(\_SB.PC00.SAT0.PRT2, DeviceObj)
External(\_SB.PC00.SAT0.PRT3, DeviceObj)
External(\_SB.PC00.SAT0.PRT4, DeviceObj)
External(\_SB.PC00.SAT0.PRT5, DeviceObj)
External(\_SB.PC00.SAT0.PRT6, DeviceObj)
External(\_SB.PC00.SAT0.PRT7, DeviceObj)

Include ("PcieRpObjCommonExternalInclude.asl")

External (\_SB.PC00.RP01.VDID)
External (\_SB.PC00.RP02.VDID)
External (\_SB.PC00.RP03.VDID)
External (\_SB.PC00.RP04.VDID)
External (\_SB.PC00.RP05.VDID)
External (\_SB.PC00.RP06.VDID)
External (\_SB.PC00.RP07.VDID)
External (\_SB.PC00.RP08.VDID)
External (\_SB.PC00.RP09.VDID)
External (\_SB.PC00.RP10.VDID)
External (\_SB.PC00.RP11.VDID)
External (\_SB.PC00.RP12.VDID)
External (\_SB.PC00.RP13.VDID)
External (\_SB.PC00.RP17.VDID)
External (\_SB.PC00.RP21.VDID)

External (\_SB.PC00.RP01.L23D, MethodObj)
External (\_SB.PC00.RP02.L23D, MethodObj)
External (\_SB.PC00.RP03.L23D, MethodObj)
External (\_SB.PC00.RP04.L23D, MethodObj)
External (\_SB.PC00.RP05.L23D, MethodObj)
External (\_SB.PC00.RP06.L23D, MethodObj)
External (\_SB.PC00.RP07.L23D, MethodObj)
External (\_SB.PC00.RP08.L23D, MethodObj)
External (\_SB.PC00.RP09.L23D, MethodObj)
External (\_SB.PC00.RP10.L23D, MethodObj)
External (\_SB.PC00.RP11.L23D, MethodObj)
External (\_SB.PC00.RP12.L23D, MethodObj)
External (\_SB.PC00.RP13.L23D, MethodObj)
External (\_SB.PC00.RP17.L23D, MethodObj)
External (\_SB.PC00.RP21.L23D, MethodObj)

External (\_SB.PC00.RP01.DL23, MethodObj)
External (\_SB.PC00.RP02.DL23, MethodObj)
External (\_SB.PC00.RP03.DL23, MethodObj)
External (\_SB.PC00.RP04.DL23, MethodObj)
External (\_SB.PC00.RP05.DL23, MethodObj)
External (\_SB.PC00.RP06.DL23, MethodObj)
External (\_SB.PC00.RP07.DL23, MethodObj)
External (\_SB.PC00.RP08.DL23, MethodObj)
External (\_SB.PC00.RP09.DL23, MethodObj)
External (\_SB.PC00.RP10.DL23, MethodObj)
External (\_SB.PC00.RP11.DL23, MethodObj)
External (\_SB.PC00.RP12.DL23, MethodObj)
External (\_SB.PC00.RP13.DL23, MethodObj)
External (\_SB.PC00.RP17.DL23, MethodObj)
External (\_SB.PC00.RP21.DL23, MethodObj)

External (\_SB.PC02,DeviceObj)
External (\_SB.PC02.SAT0, DeviceObj)
External (\_SB.PC02.SAT0.PRT0, DeviceObj)
External (\_SB.PC02.SAT0.PRT1, DeviceObj)
External (\_SB.PC02.SAT0.PRT2, DeviceObj)
External (\_SB.PC02.SAT0.PRT3, DeviceObj)
External (\_SB.PC02.SAT0.PRT4, DeviceObj)
External (\_SB.PC02.SAT0.PRT5, DeviceObj)
External (\_SB.PC02.SAT0.PRT6, DeviceObj)
External (\_SB.PC02.SAT0.PRT7, DeviceObj)

External (\_SB.PC02.RP01.VDID)
External (\_SB.PC02.RP04.VDID)
External (\_SB.PC02.RP05.VDID)
External (\_SB.PC02.RP07.VDID)
External (\_SB.PC02.RP09.VDID)
External (\_SB.PC02.RP13.VDID)
External (\_SB.PC02.RP17.VDID)
External (\_SB.PC02.RP21.VDID)

External (\_SB.PC02.RP01.L23D, MethodObj)
External (\_SB.PC02.RP04.L23D, MethodObj)
External (\_SB.PC02.RP05.L23D, MethodObj)
External (\_SB.PC02.RP07.L23D, MethodObj)
External (\_SB.PC02.RP09.L23D, MethodObj)
External (\_SB.PC02.RP13.L23D, MethodObj)
External (\_SB.PC02.RP17.L23D, MethodObj)
External (\_SB.PC02.RP21.L23D, MethodObj)

External (\_SB.PC02.RP01.DL23, MethodObj)
External (\_SB.PC02.RP04.DL23, MethodObj)
External (\_SB.PC02.RP05.DL23, MethodObj)
External (\_SB.PC02.RP07.DL23, MethodObj)
External (\_SB.PC02.RP09.DL23, MethodObj)
External (\_SB.PC02.RP13.DL23, MethodObj)
External (\_SB.PC02.RP17.DL23, MethodObj)
External (\_SB.PC02.RP21.DL23, MethodObj)

External (\_SB.PC00.XHCI, DeviceObj)
External (\_SB.PC00.XDCI, DeviceObj)
External (\_SB.PC02.XDCI, DeviceObj)

External (\_SB.PC00.XHCI.MEMB)
External (\_SB.PC00.XDCI.D0I3)
External (\_SB.PC00.XDCI.XDCB)
External (\_SB.PC02.XDCI.D0I3)
External (\_SB.PC02.XDCI.XDCB)
External (\GBEP)
External (\_SB.PC00.XHCI.RHUB, DeviceObj)
External (\_SB.PC00.XHCI.RHUB.SS01, DeviceObj)
External (\_SB.PC00.XHCI.RHUB.SS02, DeviceObj)
External (\_SB.PC00.XHCI.RHUB.HS01, DeviceObj) //xHCI HSP port 1
External (\_SB.PC00.XHCI.RHUB.HS02, DeviceObj) //xHCI HSP port 2

External (GBES)
External (\_SB.PC00.GLAN, DeviceObj)
External (\_SB.PC02.GBES)
External (\_SB.PC02.GLAN, DeviceObj)

External (PEP0)
External (XHPR)
External (RCG0, IntObj)     // RTD3 Config Setting0(BIT0:Reserved, BIT1:Reserved, Bit3: BT, BIT6:Card Reader, BIT7:WWAN, BIT8:WSB SIP FAB1 Card reader)
External (RCG1, IntObj)     // RTD3 Config Setting1(BIT0:Sata Port0, BIT1:Sata Port1, BIT2:Sata Port2, BIT3:Sata Port3)
External (\GPRW, MethodObj)
External (P8XH, MethodObj)
External (XDST, IntObj)

//
// Externs common to ULT0RTD3.asl and FFRDRTD3.asl and exclude for BRRTD3.asl
//

// GPIO methods
External (\_SB.SPMV, MethodObj)
External (\_SB.SGRA, MethodObj)
External (\_SB.SGWP, MethodObj)
External (\_SB.GGIV, MethodObj)
External (\_SB.GGOV, MethodObj)
External (\_SB.SGOV, MethodObj)
External (\_SB.SGII, MethodObj)
External (\_SB.CAGS, MethodObj)
External (\_SB.ISME, MethodObj)
External (\_SB.PGPI.IGPI, MethodObj)
External (\_SB.PGPI.CGPI, MethodObj)
External (\_SB.PGPI.GGOV, MethodObj)
External (\_SB.PGPI.SGOV, MethodObj)
External (\_SB.PGPI.SGRA, MethodObj)

External (\UAMS)
External (SDS0,FieldUnitObj)
External (SDS1,FieldUnitObj)
External (IC1D,FieldUnitObj)
External (IC1S,FieldUnitObj)
External (IC0D,FieldUnitObj)
External (AUDD,FieldUnitObj)
External (SHSB,FieldUnitObj)
External (VRRD,FieldUnitObj)

//
// DG variables
//
External (DGBA, FieldUnitObj)
External (DGOP, FieldUnitObj)

External (SPCO,MethodObj)

External (\_SB.PCLK.SPCO, MethodObj)

External (\_SB.CSD0, MethodObj)
External (\_SB.CSD3, MethodObj)
External (DVID)

External (\DTFS, IntObj) // TBT support enable/disable
External (\RPS0, IntObj)
External (\RPT0, IntObj)
External (\DTRO, IntObj)
External (\DTOE, IntObj)
External (\DTRC, IntObj)
External (\DTRD, IntObj)

Include ("HsioDefine.asl")

#define RCG1_RTD3_PRT0_ENABLED 0x01
#define RCG1_RTD3_PRT1_ENABLED 0x02
#define RCG1_RTD3_PRT2_ENABLED 0x04
#define RCG1_RTD3_PRT3_ENABLED 0x08
#define RCG1_RTD3_PRT4_ENABLED 0x10
#define RCG1_RTD3_PRT5_ENABLED 0x20
#define RCG1_RTD3_PRT6_ENABLED 0x40
#define RCG1_RTD3_PRT7_ENABLED 0x80
#define RCG1_RTD3_NVM1_ENABLED 0x100
#define RCG1_RTD3_NVM2_ENABLED 0x200
#define RCG1_RTD3_NVM3_ENABLED 0x400

//
// GPIO states mapping
//
#define PIN_GPIO_ACTIVE_LOW 0
#define PIN_GPIO_ACTIVE_HIGH 1
