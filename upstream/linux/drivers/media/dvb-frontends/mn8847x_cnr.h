/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef MN8847X_CNR_H
#define MN8847X_CNR_H

#include <linux/math64.h>
#include <linux/types.h>

/* Input is a signed log10 ratio, including its offset, in Q24 format. */
static inline int mn8847x_cnr_millidb(s64 logratio)
{
	/* Match the vendor's zero floor before scaling or narrowing the result. */
	if (logratio <= 0)
		return 0;

	return div_s64(logratio * 10000, 1 << 24);
}

#endif
