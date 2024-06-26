/** @file

  Copyright (c) 2024, Intel Corporation. All rights reserved.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#define I2C0_MODE         IM00
#define I2C1_MODE         IM01
#define I2C2_MODE         IM02
#define I2C3_MODE         IM03
#define I2C4_MODE         IM04
#define I2C5_MODE         IM05

#define I2C0_PCIE_BASE    IC00
#define I2C1_PCIE_BASE    IC01
#define I2C2_PCIE_BASE    IC02
#define I2C3_PCIE_BASE    IC03
#define I2C4_PCIE_BASE    IC04
#define I2C5_PCIE_BASE    IC05

#define I2C_DEVICE_ADR       SERIAL_IO_I2C0_ADR
#define I2C_DEVICE_MODE      I2C0_MODE
#define I2C_DEVICE_PCI_BASE  I2C0_PCIE_BASE
Device (I2C0) {
Include ("SerialIoI2cController.asl")
}
#undef I2C_DEVICE_ADR
#undef I2C_DEVICE_MODE
#undef I2C_DEVICE_PCI_BASE

#define I2C_DEVICE_ADR       SERIAL_IO_I2C1_ADR
#define I2C_DEVICE_MODE      I2C1_MODE
#define I2C_DEVICE_PCI_BASE  I2C1_PCIE_BASE
Device (I2C1) {
Include ("SerialIoI2cController.asl")
}
#undef I2C_DEVICE_ADR
#undef I2C_DEVICE_MODE
#undef I2C_DEVICE_PCI_BASE

#define I2C_DEVICE_ADR       SERIAL_IO_I2C2_ADR
#define I2C_DEVICE_MODE      I2C2_MODE
#define I2C_DEVICE_PCI_BASE  I2C2_PCIE_BASE
Device (I2C2) {
Include ("SerialIoI2cController.asl")
}
#undef I2C_DEVICE_ADR
#undef I2C_DEVICE_MODE
#undef I2C_DEVICE_PCI_BASE

#define I2C_DEVICE_ADR       SERIAL_IO_I2C3_ADR
#define I2C_DEVICE_MODE      I2C3_MODE
#define I2C_DEVICE_PCI_BASE  I2C3_PCIE_BASE
Device (I2C3) {
#if (FixedPcdGet8(PcdMtlSSupport) == 1)
Include ("SerialIoI2cController.asl")
#else
Include ("SerialIoI2cControllerEmbedded.asl")
#endif
}
#undef I2C_DEVICE_ADR
#undef I2C_DEVICE_MODE
#undef I2C_DEVICE_PCI_BASE

#define I2C_DEVICE_ADR       SERIAL_IO_I2C4_ADR
#define I2C_DEVICE_MODE      I2C4_MODE
#define I2C_DEVICE_PCI_BASE  I2C4_PCIE_BASE
Device (I2C4) {
Include ("SerialIoI2cController.asl")
}
#undef I2C_DEVICE_ADR
#undef I2C_DEVICE_MODE
#undef I2C_DEVICE_PCI_BASE

#define I2C_DEVICE_ADR       SERIAL_IO_I2C5_ADR
#define I2C_DEVICE_MODE      I2C5_MODE
#define I2C_DEVICE_PCI_BASE  I2C5_PCIE_BASE
Device (I2C5) {
Include ("SerialIoI2cController.asl")
}
#undef I2C_DEVICE_ADR
#undef I2C_DEVICE_MODE
#undef I2C_DEVICE_PCI_BASE

