/** @file
  The platform hook library.

  Copyright (c) 2020, Intel Corporation. All rights reserved.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Library/BaseLib.h>
#include <Library/IoLib.h>
#include <Library/PciLib.h>
#include <Library/PchInfoLib.h>
#include <Library/PchPcrLib.h>
#include <Library/BootloaderCommonLib.h>
#include <Library/BootloaderCoreLib.h>
#include <PlatformBase.h>
#include <PchReservedResources.h>
#include <PchAccess.h>
#include <IndustryStandard/Pci.h>
#include <Register/SerialIoUartRegs.h>

#define MM_PCI_OFFSET(Bus, Device, Function) \
    ( (UINTN)(Bus << 20) +    \
      (UINTN)(Device << 15) + \
      (UINTN)(Function << 12) )

CONST UINT32
mUartMmPciOffset[] = {
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART0, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART0),
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART1, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART1),
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART2, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART2),
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART3, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART3),
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART4, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART4),
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART5, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART5),
  MM_PCI_OFFSET (0, PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART6, PCI_FUNCTION_NUMBER_PCH_SERIAL_IO_UART6),
};


typedef struct {
  UINT8 Register;
  UINT8 Value;
} EFI_SIO_TABLE;

EFI_SIO_TABLE mSioF81216TableSerialPort[] = {

  // Start
  {0x025, 0x00}, // F81216E_CLOCK = 24Mhz
  {0x028, 0x0F}, // F81216E_SERIAL_PORT1_PRESENT

  // Program GPIO.
  {0x007, 0x09}, // SIO GPIO

  {0x0C0, 0x00}, // Enabled GPIO30~37 as Input function.
  {0x0C1, 0xFF}, //
  {0x0C3, 0x0C}, //

  {0x0D0, 0x00}, // Enabled GPIO20~27 as Input function.
  {0x0D1, 0xFF}, //
  {0x0D3, 0x2C}, //

  {0x0E0, 0xFF}, // Enabled GPIO10~17 as output function.
  //=========================================
  // GPIO1 Output Data Register, Index E1h
  //   BIT0 = GPIO10 : COM2_EN
  //   BIT1 = GPIO11 : COM2_GPIO0
  //   BIT2 = GPIO12 : COM2_GPIO1
  //   BIT3 = GPIO13 : COM2_TERM
  //   BIT4 = GPIO14 : COM1_EN
  //   BIT5 = GPIO15 : COM1_GPIO0
  //   BIT6 = GPIO16 : COM1_GPIO1
  //   BIT7 = GPIO17 : COM1_TERM
  //=========================================
  // SP339 COM Port Mode
  //          Mode1   Mode0
  //   RS232    0       1
  //   RS485    1       0
  //   RS422    1       1
  //=========================================
  {0x0E1, 0x33}, // RS232.
  {0x0E3, 0xFF}, // Set GPIO10~17 to push pull.

  // Program COM1.
  {0x007, 0x00}, // Com1 Logical Device Number select
  {0x061, 0xF8}, // Serial Port 1 Base Address LSB Register
  {0x060, 0x03}, // Serial Port 1 Base Address MSB Register
  {0x070, 0x04}, // F81216E_IRQ
  {0x0F0, 0x00}, // RS232 mode
  {0x0F6, 0x03}, // Program 128 Byte FIFO Control Registers
  {0x030, 0x01}, // Serial Port 1 activate

/*
  // Program COM2.
  {0x007, 0x01}, // Com2 Logical Device Number select
  {0x061, 0xF8}, // Serial Port 2 Base Address LSB Register
  {0x060, 0x02}, // Serial Port 2 Base Address MSB Register
  {0x070, 0x03}, // F81216E_IRQ
  {0x0F0, 0x00}, // RS232 mode
  {0x0F6, 0x03}, // Program 128 Byte FIFO Control Registers
  {0x030, 0x01}, // Serial Port 2 activate

  // Program COM3.
  {0x007, 0x02}, // COM3 RS232
  {0x060, 0x00}, // F81216E_BASE1_HI_REGISTER
  {0x061, 0x00}, // F81216E_BASE1_LO_REGISTER
  {0x030, 0x00}, // F81216E_ACTIVE

  // Program COM4.
  {0x007, 0x03}, // COM4 RS232
  {0x060, 0x00}, // F81216E_BASE1_HI_REGISTER
  {0x061, 0x00}, // F81216E_BASE1_LO_REGISTER
  {0x030, 0x00}, // F81216E_ACTIVE
*/
};


UINT8
EFIAPI
GetSerialPortStrideSize (
  VOID
  )
{
  if (GetDebugPort () >= PCH_MAX_SERIALIO_UART_CONTROLLERS) {
    // External UART, assume 0x3F8 I/O port
    return 1;
  } else {
    // SOC UART, MMIO only
    return 4;
  }
}


UINT64
EFIAPI
GetSerialPortBase (
  VOID
  )
{
  UINT16  Cmd16;
  UINTN   PciAddress;
  UINT8   DebugPort;
  UINT64  MmioBase;

  DebugPort = GetDebugPort ();
  if (DebugPort >=  PCH_MAX_SERIALIO_UART_CONTROLLERS) {
    if (DebugPort == 0xFE) {
      return 0x2F8;
    } else {
      return 0x3F8;
    }
  }
  Cmd16 = 0;
  PciAddress = mUartMmPciOffset[DebugPort] + (UINTN)PcdGet64(PcdPciExpressBaseAddress);

  Cmd16 = MmioRead16 (PciAddress + PCI_VENDOR_ID_OFFSET);
  if (Cmd16 == 0xFFFF) {
    return LPSS_UART_TEMP_BASE_ADDRESS(DebugPort);
  } else {
    if (MmioRead32 (PciAddress + PCI_COMMAND_OFFSET) & EFI_PCI_COMMAND_MEMORY_SPACE) {
      MmioBase  = LShiftU64 (MmioRead32 (PciAddress + PCI_BASE_ADDRESSREG_OFFSET + 4), 32);
      MmioBase += (MmioRead32 (PciAddress + PCI_BASE_ADDRESSREG_OFFSET) & 0xFFFFFFF0);
      return MmioBase;
    } else {
      return 0;
    }
  }
}

/**
  Performs platform specific initialization required for the CPU to access
  the hardware associated with a SerialPortLib instance.  This function does
  not initialize the serial port hardware itself.  Instead, it initializes
  hardware devices that are required for the CPU to access the serial port
  hardware.  This function may be called more than once.

  @retval RETURN_SUCCESS       The platform specific initialization succeeded.
  @retval RETURN_DEVICE_ERROR  The platform specific initialization could not be completed.

**/
RETURN_STATUS
EFIAPI
LegacySerialPortInitialize (
  VOID
  )
{
  UINTN   eSPIBaseAddr;
  UINT16  Data16;
  UINT32  Data32;
  UINT16  Index;

  eSPIBaseAddr = PCI_LIB_ADDRESS (
    DEFAULT_PCI_BUS_NUMBER_PCH,
    PCI_DEVICE_NUMBER_PCH_ESPI,
    PCI_FUNCTION_NUMBER_PCH_ESPI,
    0);

  Data16 = PciRead16 (eSPIBaseAddr + R_LPC_CFG_IOD);
  Data16 |= (V_LPC_CFG_IOD_COMB_2F8 << N_LPC_CFG_IOD_COMB);
  Data16 |= (V_LPC_CFG_IOD_COMA_3F8 << N_LPC_CFG_IOD_COMA);
  MmioWrite16 (PCH_PCR_ADDRESS (PID_DMI, R_PCH_DMI_PCR_LPCIOD), Data16);
  PciWrite16 (eSPIBaseAddr + R_LPC_CFG_IOD, Data16);

  Data16 = PciRead16 (eSPIBaseAddr + R_LPC_CFG_IOE);
  Data16 |= B_LPC_CFG_IOE_ME2; // enable decoding of I/O locations 4Eh and 4Fh
  Data16 |= B_LPC_CFG_IOE_CBE;
  Data16 |= B_LPC_CFG_IOE_CAE;
  MmioWrite16 (PCH_PCR_ADDRESS (PID_DMI, R_PCH_DMI_PCR_LPCIOE), Data16);
  PciWrite16 (eSPIBaseAddr + R_LPC_CFG_IOE, Data16);

  // Enable CS1# IO Routing
  Data32 = PciRead32 (eSPIBaseAddr + R_ESPI_CFG_CS1IORE);
  Data32 |= B_ESPI_CFG_CS1IORE_DPRE; // for 80h
  Data32 |= B_ESPI_CFG_CS1IORE_MRE2; // for 4Eh, 4Fh
  Data32 |= B_ESPI_CFG_CS1IORE_CARE; // for Com Port A
  PciWrite32(eSPIBaseAddr + R_ESPI_CFG_CS1IORE, Data32);

  // Specify 0x80 in DWord-aligned address format for decoding to CS1#
  Data32 = PciRead32 (eSPIBaseAddr + R_ESPI_CFG_CS1GIR1);
  Data32 &= ~0x00FFFFFF;
  Data32 |= (0x80 >> 2) << N_ESPI_CFG_CS1GIR1_ADDR_MASK;
  Data32 |= (0x80 >> 2) << N_ESPI_CFG_CS1GIR1_ADDR;
  Data32 |= B_ESPI_CFG_CS1GIR1_LDE;
  PciWrite32(eSPIBaseAddr + R_ESPI_CFG_CS1GIR1, Data32);

  // Output 0x1 to 80h (7-segment LED)
  IoWrite8(0x80, 0x1);

  // Init SIO F81216E
  // Enter Config Mode
  IoWrite8(0x4E, 0x77);
  IoWrite8(0x4E, 0x77);

  for (Index = 0; Index < sizeof(mSioF81216TableSerialPort) / sizeof(EFI_SIO_TABLE); Index++) {
    IoWrite8(0x4E, mSioF81216TableSerialPort[Index].Register);
    IoWrite8(0x4F, mSioF81216TableSerialPort[Index].Value);
  }

  //Exit Config Mode
  IoWrite8(0x4E, 0xAA);

  return RETURN_SUCCESS;
}

RETURN_STATUS
EFIAPI
PlatformHookSerialPortInitialize (
  VOID
  )
{
  UINTN   PciAddress;
  UINT32  BarAddress;
  UINT8   DebugPort;

  DebugPort = GetDebugPort ();
  if (DebugPort >= PCH_MAX_SERIALIO_UART_CONTROLLERS) {
    LegacySerialPortInitialize ();
  } else {
    BarAddress = LPSS_UART_TEMP_BASE_ADDRESS(DebugPort);
    PciAddress = mUartMmPciOffset[DebugPort] + (UINTN)PcdGet64(PcdPciExpressBaseAddress);
    MmioWrite32 (PciAddress + R_SERIAL_IO_CFG_BAR0_LOW,  BarAddress);
    MmioWrite32 (PciAddress + R_SERIAL_IO_CFG_BAR0_HIGH, 0x0);
    MmioWrite32 (PciAddress + R_SERIAL_IO_CFG_BAR1_LOW,  BarAddress + 0x1000);
    MmioWrite32 (PciAddress + R_SERIAL_IO_CFG_BAR1_HIGH, 0x0);
    MmioWrite32 (PciAddress + PCI_COMMAND_OFFSET, EFI_PCI_COMMAND_BUS_MASTER | EFI_PCI_COMMAND_MEMORY_SPACE);
    MmioOr32    (PciAddress + R_SERIAL_IO_CFG_D0I3MAXDEVPG, BIT18 | BIT17 | BIT16);

    // get controller out of reset
    MmioOr32 (BarAddress + R_SERIAL_IO_MEM_PPR_RESETS,
      B_SERIAL_IO_MEM_PPR_RESETS_FUNC | B_SERIAL_IO_MEM_PPR_RESETS_APB | B_SERIAL_IO_MEM_PPR_RESETS_IDMA);

    // Program clock dividers for UARTs
    MmioWrite32 (BarAddress + R_SERIAL_IO_MEM_PPR_CLK,
        (B_SERIAL_IO_MEM_PPR_CLK_UPDATE | (V_SERIAL_IO_MEM_PPR_CLK_N_DIV << 16) |
         (V_SERIAL_IO_MEM_PPR_CLK_M_DIV << 1) | B_SERIAL_IO_MEM_PPR_CLK_EN )
        );
  }
  return RETURN_SUCCESS;
}
