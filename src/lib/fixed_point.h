/*

Read Route Record

Copyright (C) 2019 Atle Solbakken atle@goliathdns.no

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef RRR_FIXED_POINT_H
#define RRR_FIXED_POINT_H

#define RRR_FIXED_POINT_BASE2_EXPONENT 24
#define RRR_FIXED_POINT_NUMBER_MAX 0x7FFFFFFFFF

#include <inttypes.h>

typedef int64_t rrr_fixp;

int rrr_ldouble_to_fixp (rrr_fixp *target, long double source);
int rrr_fixp_to_ldouble (long double *target, rrr_fixp source);
int rrr_fixp_to_str (char *target, ssize_t target_size, rrr_fixp source);
int rrr_str_to_fixp (rrr_fixp *target, const char *str);

#endif /* RRR_FIXED_POINT_H */
