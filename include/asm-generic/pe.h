/*
 *  Portable Executable binary format structures
 *
 *  Copyright (c) 2016 Alexander Graf
 *
 *  Based on wine code
 *
 *  SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef _ASM_PE_H
#define _ASM_PE_H

/* PE32+ Subsystem type for EFI images */
#define IMAGE_SUBSYSTEM_EFI_APPLICATION         10
#define IMAGE_SUBSYSTEM_EFI_BOOT_SERVICE_DRIVER 11
#define IMAGE_SUBSYSTEM_EFI_RUNTIME_DRIVER      12
#define IMAGE_SUBSYSTEM_SAL_RUNTIME_DRIVER      13

#endif
