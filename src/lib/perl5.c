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

#include <pthread.h>
#include <stddef.h>

#include <EXTERN.h>
#include <perl.h>

#include "../../build_directory.h"
#include "perl5.h"
#include "messages.h"
#include "rrr_socket_msg.h"

#define RRR_PERL5_BUILD_LIB_PATH_1 \
	RRR_BUILD_DIR "/debian/rrr/usr/lib/x86_64-linux-gnu/perl5/5.26/"

#define RRR_PERL5_BUILD_LIB_PATH_2 \
	RRR_BUILD_DIR "/"

static pthread_mutex_t main_python_lock = PTHREAD_MUTEX_INITIALIZER;
static int perl5_users = 0;

static void __rrr_perl5_global_lock(void) {
	pthread_mutex_lock(&main_python_lock);
}

static void __rrr_perl5_global_unlock(void) {
	pthread_mutex_unlock(&main_python_lock);
}

int rrr_perl5_init3(int argc, char **argv, char **env) {
	__rrr_perl5_global_lock();
	if (++perl5_users == 1) {
		PERL_SYS_INIT3(&argc, &argv, &env);
	}
	__rrr_perl5_global_unlock();
	return 0;
}

int rrr_perl5_sys_term(void) {
	__rrr_perl5_global_lock();
	if (--perl5_users == 0) {
		PERL_SYS_TERM();
	}
	__rrr_perl5_global_unlock();
	return 0;
}

static PerlInterpreter *__rrr_perl5_construct(void) {
	PerlInterpreter *ret = NULL;

	__rrr_perl5_global_lock();

	ret = perl_alloc();
	if (ret == NULL) {
		VL_MSG_ERR("Could not allocate perl5 interpreter in rrr_perl5_construct\n");
		goto out_unlock;
	}

	perl_construct(ret);
//	PL_exit_flags |= PERL_EXIT_DESTRUCT_END;

	out_unlock:
	__rrr_perl5_global_unlock();

	out:
	return ret;
}

static void __rrr_perl5_destruct (PerlInterpreter *interpreter) {
	if (interpreter == NULL) {
		return;
	}

	__rrr_perl5_global_lock();
	perl_destruct(interpreter);
	perl_free(interpreter);
	__rrr_perl5_global_unlock();
}

void rrr_perl5_destroy_ctx (struct rrr_perl5_ctx *ctx) {
	if (ctx == NULL) {
		return;
	}
	__rrr_perl5_destruct(ctx->interpreter);
	free(ctx);
}

int rrr_perl5_new_ctx (struct rrr_perl5_ctx **target) {
	int ret = 0;
	struct rrr_perl5_ctx *ctx = NULL;

	ctx = malloc(sizeof(*ctx));
	if (ctx == NULL) {
		VL_MSG_ERR("Could not allocate memory in rrr_perl5_new_ctx\n");
		ret = 1;
		goto out;
	}
	memset (ctx, '\0', sizeof(*ctx));

	ctx->interpreter = __rrr_perl5_construct();
	if (ctx->interpreter == NULL) {
		VL_MSG_ERR("Could not create perl5 interpreter in rrr_perl5_new_ctx\n");
		ret = 1;
		goto out;
	}

	*target = ctx;

	out:
	if (ret != 0 && ctx != NULL) {
		rrr_perl5_destroy_ctx (ctx);
	}
	return ret;
}

/* From perl5_xsi.c generated by configure */
EXTERN_C void xs_init (pTHX);

static void __rrr_perl5_xs_init(pTHX) {
	xs_init(my_perl);
}

int rrr_perl5_ctx_parse (struct rrr_perl5_ctx *ctx, char *filename) {
	int ret = 0;

	PERL_SET_CONTEXT(ctx->interpreter);

	// Test-open file
	int fd = open(filename, O_RDONLY);
	if (fd < 1) {
		VL_MSG_ERR("Could not open perl5 file %s: %s\n",
				filename, strerror(errno));
		ret = 1;
		goto out;
	}
	close(fd);

	char *args[] = {
			"",
			"-I" RRR_PERL5_BUILD_LIB_PATH_1,
			"-I" RRR_PERL5_BUILD_LIB_PATH_2,
			filename,
			NULL
	};

	if (perl_parse(ctx->interpreter, __rrr_perl5_xs_init, 4, args, (char**) NULL) != 0) {
		VL_MSG_ERR("Could not parse perl5 file %s\n", filename);
		ret = 1;
		goto out;
	}

	out:
	return ret;
}


int rrr_perl5_ctx_run (struct rrr_perl5_ctx *ctx) {
	PERL_SET_CONTEXT(ctx->interpreter);
	return perl_run(ctx->interpreter);
}

int rrr_perl5_call_blessed_hvref (struct rrr_perl5_ctx *ctx, const char *sub, const char *class, HV *hv) {
	int ret = 0;

	PerlInterpreter *my_perl = ctx->interpreter;
    PERL_SET_CONTEXT(my_perl);

    HV *stash = gv_stashpv(class, GV_ADD);
    if (stash == NULL) {
    	VL_BUG("No stash HV returned in rrr_perl5_call_blessed_hvref\n");
    }

    SV *ref = newRV_inc((SV*) hv);
    if (ref == NULL) {
    	VL_BUG("No ref SV returned in rrr_perl5_call_blessed_hvref\n");
    }

    SV *blessed_ref = sv_bless(ref, stash);
    if (blessed_ref == NULL) {
    	VL_BUG("No blessed ref SV returned in rrr_perl5_call_blessed_hvref\n");
    }

//    printf ("A: Blessed a reference, package is %s\n", HvNAME(stash));
//    printf ("B: Blessed a reference, package is %s\n", HvNAME(SvSTASH(SvRV(blessed_ref))));

	dSP;
	ENTER;
	SAVETMPS;
	PUSHMARK(SP);
	EXTEND(SP, 3);
	PUSHs(sv_2mortal(blessed_ref));
	PUTBACK;
	call_pv(sub, G_DISCARD);
	FREETMPS;
	LEAVE;

	return ret;
}
/*
#define QUOTE(str) \
        "\"" #str "\""

#define PASTE(a,b) \
        a ## b
*/
#define SV_DEC_UNLESS_NULL(sv) \
	do {if (sv != NULL) { SvREFCNT_dec(sv); }} while (0)


struct rrr_perl5_message_hv *rrr_perl5_allocate_message_hv (struct rrr_perl5_ctx *ctx) {
	PerlInterpreter *my_perl = ctx->interpreter;
    PERL_SET_CONTEXT(my_perl);

    struct rrr_perl5_message_hv *message_hv = malloc(sizeof(*message_hv));
    if (message_hv == NULL) {
    	VL_MSG_ERR("Could not allocate memory in rrr_perl5_message_allocate_hv\n");
    	goto out;
    }

    message_hv->hv = newHV();

    SV **tmp;

    tmp = hv_fetch(message_hv->hv, "type", strlen("type"), 1);
    message_hv->type = *tmp;

    tmp = hv_fetch(message_hv->hv, "class", strlen("class"), 1);
    message_hv->class = *tmp;

    tmp = hv_fetch(message_hv->hv, "timestamp_from", strlen("timestamp_from"), 1);
    message_hv->timestamp_from = *tmp;

    tmp = hv_fetch(message_hv->hv, "timestamp_to", strlen("timestamp_to"), 1);
    message_hv->timestamp_to = *tmp;

    tmp = hv_fetch(message_hv->hv, "data_numeric", strlen("data_numeric"), 1);
    message_hv->data_numeric = *tmp;

    tmp = hv_fetch(message_hv->hv, "length", strlen("length"), 1);
    message_hv->length = *tmp;

    message_hv->data = newSV(18); // Conservative size
    SvUTF8_off(message_hv->data);
    sv_setpvn(message_hv->data, "0", 1);
    tmp = hv_store(message_hv->hv, "data", strlen("data"), message_hv->data, 0);
    message_hv->data = *tmp;

    out:
    return message_hv;
}

void rrr_perl5_destruct_message_hv (
		struct rrr_perl5_ctx *ctx,
		struct rrr_perl5_message_hv *source
) {
	if (source == NULL) {
		return;
	}

	PerlInterpreter *my_perl = ctx->interpreter;
    PERL_SET_CONTEXT(my_perl);

	SV_DEC_UNLESS_NULL(source->hv);

	free(source);
}

int rrr_perl5_hv_to_message (
		struct vl_message *target,
		struct rrr_perl5_ctx *ctx,
		struct rrr_perl5_message_hv *source
) {
	int ret = 0;

	PerlInterpreter *my_perl = ctx->interpreter;
    PERL_SET_CONTEXT(my_perl);

	target->type = SvUV(source->type);
	target->class = SvUV(source->class);
	target->timestamp_from = SvUV(source->timestamp_from);
	target->timestamp_to = SvUV(source->timestamp_to);
	target->data_numeric = SvUV(source->data_numeric);
	target->length = SvUV(source->length);

	if (target->length > MSG_DATA_MAX_LENGTH) {
		VL_MSG_ERR("Data length returned from perl5 function was too long (%" PRIu32 " > %i)\n",
				target->length, MSG_DATA_MAX_LENGTH);
		ret = 1;
		goto out;
	}

	if (SvLEN(source->data) < target->length) {
		VL_MSG_ERR("Data length returned from perl5 function was shorter than given length in length field\n");
		ret = 1;
		goto out;
	}

	STRLEN len = target->length;
	char *data_str = SvPV(source->data, len);
	strncpy(target->data, data_str, target->length);

	out:
	if (ret != 0) {

	}


	return ret;
}

int rrr_perl5_message_to_hv (
		struct rrr_perl5_message_hv *message_hv,
		struct rrr_perl5_ctx *ctx,
		struct vl_message *message
) {
	int ret = 0;

	if (!RRR_SOCKET_MSG_IS_VL_MESSAGE(message)) {
		VL_BUG("Message to rrr_perl5_message_to_hv was not a VL message\n");
	}

	if (message->length > MSG_DATA_MAX_LENGTH) {
		VL_BUG("Message length was too long in rrr_perl5_message_to_hv\n");
	}

	PerlInterpreter *my_perl = ctx->interpreter;
    PERL_SET_CONTEXT(my_perl);

    sv_setuv(message_hv->type, message->type);
    sv_setuv(message_hv->class, message->class);
    sv_setuv(message_hv->timestamp_from, message->timestamp_from);
    sv_setuv(message_hv->timestamp_to, message->timestamp_to);
    sv_setuv(message_hv->data_numeric, message->data_numeric);
    sv_setuv(message_hv->length, message->length);
    sv_setpvn(message_hv->data, message->data, message->length);

    out:
	return ret;
}

int rrr_perl5_message_to_new_hv (
		struct rrr_perl5_message_hv **target,
		struct rrr_perl5_ctx *ctx,
		struct vl_message *message
) {
    int ret = 0;

    struct rrr_perl5_message_hv *message_hv = rrr_perl5_allocate_message_hv(ctx);
    if (message_hv == NULL) {
    	ret = 1;
    	goto out;
    }

    if ((ret = rrr_perl5_message_to_hv(message_hv, ctx, message)) != 0) {
    	VL_MSG_ERR("Error in rrr_perl5_message_to_new_hv\n");
    	goto out;
    }

    *target = message_hv;

    out:
	if (ret != 0) {
		rrr_perl5_destruct_message_hv(ctx, message_hv);
	}
    return ret;
}
