/*
 * Licensed under GPL version 3.
 *
 * Copyright 2020 Per Johansson <nisse@hoj.nu>
 *
 */
#include <string.h>
#include <stdlib.h>

#include "log.h"
#include "cmodule.h"
#include "util/posix.h"
#include "util/rrr_time.h"

#include <nodave.h>
#include "s7plc/setport.h"

// Globals
daveInterface * di;
daveConnection * dc;
_daveOSserialType fds;

PDU wr_p;
daveResultSet wr_rs;

PDU rd_p;
daveResultSet rd_rs;

uint8_t  process_command;
uint16_t process_value;


#define MAX_RETRY_CNT	10

#define CMD_NONE	0
#define CMD_SETTIME	1
#define CMD_TRIGGER	2
#define CLR_TRIGGER     3

struct dummy_data {
        char *device_setting;
	char *speed_setting;
};

static struct dummy_data dummy_data = {0};

int config(RRR_CONFIG_ARGS) {

        int localPPI=0;
        int plcPPI=2;


	struct dummy_data *data = &dummy_data;

	int ret = 0;

	ctx->application_ptr = data;

	RRR_MSG_1("---> S7 in config()\n");

	RRR_INSTANCE_CONFIG_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("s7plc_device", device_setting);
	RRR_INSTANCE_CONFIG_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("s7plc_baud", speed_setting);

	if (data->device_setting == NULL || *(data->device_setting) == '\0') {
		RRR_MSG_0("Could not find setting 's7plc_device' in configuration\n");
		ret = 1;
		goto out;
	}

        if (data->speed_setting == NULL || *(data->speed_setting) == '\0') {
                RRR_MSG_0("Could not find setting 's7plc_baud' in configuration\n");
                ret = 1;
                goto out;
        }


	RRR_MSG_1("S7 Custom settings: %s %s\n", data->device_setting, data->speed_setting);

        fds.rfd=setPort(data->device_setting,data->speed_setting,'E');
        fds.wfd=fds.rfd;
        if (fds.rfd>0) {
                di =daveNewInterface(fds, "IF1", localPPI, daveProtoPPI, daveSpeed187k);
                dc =daveNewConnection(di, plcPPI, 0, 0);
                daveConnectPLC(dc);
                davePrepareReadRequest(dc, &rd_p);
                daveAddVarToReadRequest(&rd_p,daveDB,1,100,10);
                daveAddVarToReadRequest(&rd_p,daveFlags,0,19,4);
                daveAddVarToReadRequest(&rd_p,daveTimer200,0,37,4);
		RRR_MSG_0("Communication open\n");
        }
        else {
		RRR_MSG_0("Could not open communication port %s\n", data->device_setting);
                ret = 1;
        }



	out:
        RRR_FREE_IF_NOT_NULL(data->device_setting);
        RRR_FREE_IF_NOT_NULL(data->speed_setting);
	return ret;
}

int swapbytes(int b) {
    union {
        short a;
        unsigned char b[2];
    } u;
    u.b[1]=b&0xff;
    b++;
    u.b[0]=b>>8;
    return u.a;
}


int source(RRR_SOURCE_ARGS) {
#define BUFSZ	32
	(void)(ctx);
	(void)(message_addr);
	int res, cnvres;
  	float plcTemp, plcHumid;
 	int plcVatid;
 	float plcVatmr;
//        rrr_fixp fixTemp, fixHumid;
	unsigned char  plcLarm,  plcVaStrb;
	struct rrr_array arr_response = {0};
	struct rrr_msg_msg *sendmsg;
	daveResultSet rs;
   	char topicbuf[BUFSZ];
	char databuf[BUFSZ];
	PDU p;
	static int write_retry=0;
//	RRR_MSG_1("S7 Source entry %" PRIu64 " \n", rrr_time_get_64());
	rrr_free(message); // release allocated message
	rrr_array_clear(&arr_response);
	
	//daveAddVarToReadRequest(&p,daveDB,1,100,11);
	//    res=daveWriteBytes(dc, AREA, area_Number, start_address, length, buffer);	
	// 	printf("Trying to set single bit E0.5\n");

	if (process_command==CMD_SETTIME)
	{
		int wrVal;
		wrVal=swapbytes(process_value);
		res=daveWriteBytes(dc, daveDB, 1, 108, 2, &wrVal);
		if (res==0)
		{
			process_command=CMD_NONE;
		}
                else
                {
                        write_retry++;
                }

		RRR_MSG_2("S7 Write timer %d. Result: %x \n", process_value, res);
	}
	else if (process_command==CMD_TRIGGER)
        {
		int a=1;
		res=daveWriteBytes(dc, daveDB, 1, 110, 1, &a);

                if (res==0)
                {
                        process_command=CLR_TRIGGER;
                }
                else
                {
                        write_retry++;
                }

                RRR_MSG_2("S7 Trigger set. Result: %x \n", process_value, res);
        }
        else if (process_command==CLR_TRIGGER)
        {
                int a=0;
                res=daveWriteBytes(dc, daveDB, 1, 110, 1, &a);
                if (res==0)
                {
                        process_command=CMD_NONE;
                }
		else
		{
			write_retry++;
		}
                RRR_MSG_2("S7 Trigger clear. Result: %x \n", process_value, res);
        }

	if (write_retry>MAX_RETRY_CNT)
	{
		RRR_MSG_2("Max number of retries. Discarding command");
		process_command=CMD_NONE;
		write_retry=0;
	}



	davePrepareReadRequest(dc, &p);
	daveAddVarToReadRequest(&p,daveDB,1,100,11);
	daveAddVarToReadRequest(&p,daveFlags,0,19,4);
	daveAddVarToReadRequest(&p,daveTimer200,0,37,4);

	res=daveExecReadRequest(dc, &p, &rs);
	snprintf(topicbuf, BUFSZ, "s7plc/comstat");
	snprintf(databuf, BUFSZ, "%d", res);
//	RRR_MSG_1("S7 Poll %" PRIu64 " \n", rrr_time_get_64());
//	_daveDump("Data",dc->resultPointer,dc->AnswLen);
	rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
	rrr_send_and_free(ctx, sendmsg, message_addr);

//	snprintf(topicbuf, BUFSZ, "s7plc/comstatint");
//	rrr_msg_msg_new_empty (&sendmsg, RRR_TYPE_H, MSG_CLASS_DATA, rrr_time_get_64(), strlen(topicbuf), 1);
//        memcpy (MSG_TOPIC_PTR(sendmsg), topicbuf, strlen(topicbuf));

//	*MSG_DATA_PTR(sendmsg)=res;
//	rrr_send_and_free(ctx, sendmsg, message_addr);

	snprintf(topicbuf, BUFSZ, "s7plc/commsg");
	snprintf(databuf, BUFSZ, "%s", daveStrerror(res));
        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
        rrr_send_and_free(ctx, sendmsg, message_addr);

//	rrr_array_push_value_i64_with_tag(&arr_response,"com status",res);
//	rrr_array_push_value_str_with_tag(&arr_response,"com msg",daveStrerror(res));

//	daveAddVarToReadRequest(&p,daveDB,1,100,11);


	if (res==0) {
//		RRR_MSG_1("S7 Parse 1 %" PRIu64 " \n", rrr_time_get_64());
		res=daveUseResult(dc, &rs, 0); // first result
		if (res==0) {
//                	a=daveGetU16(dc);
			plcTemp=daveGetFloat(dc); // VD100, temperature
			plcHumid=daveGetFloat(dc)*10/3; // VD104, hudidity. Scale error correction
			plcVatid=daveGetU16(dc);
			plcVaStrb=daveGetU8(dc);
//			RRR_MSG_1("Siemens S7 got temp: %03.2f, hum: %02.1f, VaTid: %d, VaStrb:%x\n", plcTemp, plcHumid, plcVatid, plcVaStrb);
//			cnvres=rrr_fixp_ldouble_to_fixp(&fixTemp, plcTemp);
//			RRR_MSG_1("Fixpcnf: %d", cnvres);
// 			cnvres=rrr_fixp_ldouble_to_fixp(&fixHumid, plcHumid);
		        
			snprintf(topicbuf, BUFSZ, "s7plc/air_temp");
        		snprintf(databuf, BUFSZ, "%.3f", plcTemp);
        		rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
        		rrr_send_and_free(ctx, sendmsg, message_addr);

                        snprintf(topicbuf, BUFSZ, "s7plc/soil_humid");
                        snprintf(databuf, BUFSZ, "%0.2f", plcHumid);
                        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        rrr_send_and_free(ctx, sendmsg, message_addr);

                        snprintf(topicbuf, BUFSZ, "s7plc/water_preset");
                        snprintf(databuf, BUFSZ, "%d", plcVatid);
                        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        rrr_send_and_free(ctx, sendmsg, message_addr);

                        snprintf(topicbuf, BUFSZ, "s7plc/water_strobe");
                        snprintf(databuf, BUFSZ, "%d", plcVaStrb);
                        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        rrr_send_and_free(ctx, sendmsg, message_addr);


//			rrr_array_push_value_fixp_with_tag(&arr_response,"temp",fixTemp);
//			rrr_array_push_value_fixp_with_tag(&arr_response,"humid",fixHumid);
//			rrr_array_push_value_u64_with_tag(&arr_response,"wpreset",plcVatid);
//			rrr_array_push_value_u64_with_tag(&arr_response,"wstrb",plcVaStrb);
  		}
   		else
   		{
  			RRR_MSG_1("DaveSet1: %d\n",res);
  			RRR_MSG_1("%s\n",daveStrerror(res));
                }
		res=daveUseResult(dc, &rs, 1); // 2nd result
//      	_daveDump("Data",dc->resultPointer,dc->AnswLen);
//		RRR_MSG_1("S7 Parse 2 %" PRIu64 " \n", rrr_time_get_64());
  		if (res==0) {
  			plcLarm=daveGetU8(dc);
//  			RRR_MSG_1("Siemens S7 got Larm: %x\n", plcLarm);
//			rrr_array_push_value_u64_with_tag(&arr_response,"alarm",plcLarm);
                        snprintf(topicbuf, BUFSZ, "s7plc/alarm");
                        snprintf(databuf, BUFSZ, "%d", plcLarm);
                        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        rrr_send_and_free(ctx, sendmsg, message_addr);

 		}
 		else
		{
			RRR_MSG_1("DaveSet2: %d\n",res);
 			RRR_MSG_1("%s\n",daveStrerror(res));
   		}
          	res=daveUseResult(dc, &rs, 2); //3rd result
//		_daveDump("Data",dc->resultPointer,dc->AnswLen);

    		if (res==0) {
//			RRR_MSG_1("S7 Parse 3 %" PRIu64 " \n", rrr_time_get_64());
      			plcVatmr=daveGetIECSeconds(dc);
//     			RRR_MSG_1("Siemens S7 got VaTid: %0.3f\n", plcVatmr);
//			rrr_array_push_value_u64_with_tag(&arr_response,"wtime",plcVatmr);
                        snprintf(topicbuf, BUFSZ, "s7plc/water_time");
                        snprintf(databuf, BUFSZ, "%0.3f", plcVatmr);
                        rrr_msg_msg_new_with_data (&sendmsg, RRR_TYPE_STR, MSG_CLASS_DATA, rrr_time_get_64(), topicbuf, strlen(topicbuf), databuf, strlen(databuf));
                        rrr_send_and_free(ctx, sendmsg, message_addr);

   		}
    		else
      		{
     			RRR_MSG_1("DaveSet3: %d\n",res);
   			RRR_MSG_1("%s\n",daveStrerror(res));
              	}
   		daveFreeResults(&rs);
       	}
	else
 	{		
        	RRR_MSG_1("DaveExec: %d\n",res);
       		RRR_MSG_1("%s\n",daveStrerror(res));
   	}
//	RRR_MSG_1("S7 Exit %" PRIu64 " \n", rrr_time_get_64());
//	rrr_array_dump(&arr_response);
//	rrr_array_clear(&arr_response);

//	res=rrr_array_new_message_from_collection(&message, &arr_response, rrr_time_get_64(), topic, strlen(topic));
//	RRR_MSG_1("arr2msg: %d\n",res);
//	rrr_send_and_free(ctx, message, message_addr);
//	rrr_array_clear(&arr_response);
	return 0;
}

int process(RRR_PROCESS_ARGS) {
	char *result;
	char *out;
	char *tokpos;
	char *strval=0;
	char *strtag=0;
	struct rrr_array array_tmp = {0};
	int retval=0;
	int intval;
        struct rrr_msg_msg *sendmsg;
        char topicbuf[BUFSZ]="return\0";
        char databuf[BUFSZ];

        if ((MSG_TYPE(message)==MSG_TYPE_MSG)&&(MSG_CLASS(message)==MSG_CLASS_ARRAY)) 
	{
                if (rrr_array_message_append_to_collection(&array_tmp, message) != 0) {
                        RRR_MSG_0("Could not create temporary array collection in s7plc process\n");
                        retval = 1;
                        goto out_free;
                }

		rrr_array_dump(&array_tmp);

	        struct rrr_type_value *value = NULL;

	        if ((value = rrr_array_value_get_by_tag(&array_tmp, "msg")) == NULL) {
	                RRR_MSG_0("Could not find value in array\n");
	                retval = 1;
	                goto out_free;
        	}

	        if (!RRR_TYPE_IS_NSEP(value->definition->type)) { // type seem to be always 0 
        	        RRR_MSG_2("Value in array was not str as expected %d\n", value->definition->type);
                	retval = 2;
                	goto out_free;
        	}

                tokpos=memchr(value->data, '=', value->total_stored_length); // Try to find the tag / value separator
                if (tokpos != NULL) // We have a separator
                {
			int vallen;
                        *tokpos=0; // Set NULL @ separator position
                        tokpos++;  // Incement pointer to first value char
			strtag=value->data; // Just take the pointer. Terminator is already set where separator was.

			vallen=value->total_stored_length - (strlen(strtag) + 1); // Length of value part.
			
                        if (strncmp("SetTime",strtag, value->total_stored_length) == 0) // Did we get expected tag?
                        {
                                RRR_DBG_2("Got Key %s \n", strtag);
				strval=strndup(tokpos, vallen);
				intval=atoi(strval);
                                RRR_DBG_2("Value is %s (0x%x)\n ", strval,intval);
				process_command=CMD_SETTIME;
				process_value=intval;
                        }
                        else if (strncmp("WTrig",strtag, value->total_stored_length) == 0) // Did we get expected tag?
                        {
                                RRR_DBG_2("Got Key %s \n", strtag);
                                strval=strndup(tokpos, vallen);
                                intval=atoi(strval);
                                RRR_DBG_2("Value is %s (0x%x)\n ", strval,intval);
				process_command=CMD_TRIGGER;
				process_value=0;
                        }
			else
			{
				retval=0x80;
			}
			RRR_FREE_IF_NOT_NULL(strval);
                }

        }

out_free:
	
        rrr_free(message); // release allocated message
	rrr_array_clear(&array_tmp);
	if (retval)
	{
		RRR_MSG_1("\nS7 process parse error %d\n", retval);
		return 1;
	}

        return retval;
}

int cleanup(RRR_CLEANUP_ARGS) {
	struct dummy_data *data = ctx->application_ptr;

	RRR_MSG_1("\nS7 cmodule cleanup\n");

	ctx->application_ptr = NULL;

	return 0;
}
