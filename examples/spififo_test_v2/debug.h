/*
 * Copyright (C) 2024 Hery Dang (henrydang@mijoconnected.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

void cdc_acm_init(void);
void cdc_acm_printf(const char *format, ...);

#define usb_printf cdc_acm_printf

#endif /* _DEBUG_H_ */
