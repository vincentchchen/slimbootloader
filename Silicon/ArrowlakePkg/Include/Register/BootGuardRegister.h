/** @file

  Copyright (c) 2024, Intel Corporation. All rights reserved.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/
#ifndef _BOOT_GUARD_REGISTER_H
#define _BOOT_GUARD_REGISTER_H


#define MSR_BOOT_GUARD_SACM_INFO                                      0x13A
#define V_TPM_PRESENT_MASK                                            0x06
#define B_BOOT_GUARD_SACM_INFO_NEM_ENABLED                            BIT0
#define B_BOOT_GUARD_SACM_INFO_TPM_SUCCESS                            BIT3
#define B_BOOT_GUARD_SACM_INFO_MEASURED_BOOT                          BIT5
#define B_BOOT_GUARD_SACM_INFO_VERIFIED_BOOT                          BIT6
#define B_BOOT_GUARD_SACM_INFO_CAPABILITY                             BIT32
#define B_BOOT_GUARD_ACM_ERRORCODE_MASK                               0x00007FF0
#define TXT_PUBLIC_BASE                                               0xFED30000
#define R_CPU_BOOT_GUARD_BOOTSTATUS                                   0xA0
#define B_CPU_BOOT_GUARD_BOOTSTATUS_S3_TPM_STARTUP_FAILED             BIT46

#define R_CPU_BOOT_GUARD_ACM_STATUS                                   0x328
#define R_CPU_ACM_POLICY_STATUS                                       0x378
#define MMIO_ACM_POLICY_STATUS                                        (TXT_PUBLIC_BASE + R_CPU_ACM_POLICY_STATUS)

#define R_ME_HFS                                                      0x40
#define R_ME_HFS_2                                                    0x48
#define R_ME_HFS_3                                                    0x60
#define R_ME_HFS_4                                                    0x64
#define R_ME_HFS_5                                                    0x68
#define R_ME_HFS_6                                                    0x6C
#define B_BOOT_GUARD_ENF_MASK                                         0x0200
#define B_TPM_DISCONNECT                                              0x1000


#endif
