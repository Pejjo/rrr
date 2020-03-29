/*

Read Route Record

Copyright (C) 2020 Atle Solbakken atle@goliathdns.no

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

// Override possibly dangerous library functions

#ifndef RRR_INTERCEPT_H
#define RRR_INTERCEPT_H

#ifndef RRR_INTERCEPT_ALLOW_READDIR
	// Not guaranteed thread-safety in current POSIX specification, rrr wrapper with
	// locking must be used
#	define readdir(x) RRR_INTERCEPT_H_UNSAFE_LIBARY_FUNCTION_READDIR
#endif

#ifndef RRR_INTERCEPT_ALLOW_STRERROR
#	define strerror(x) RRR_INTERCEPT_H_UNSAFE_LIBARY_FUNCTION_STRERROR
#endif

#endif /* RRR_INTERCEPT_H */