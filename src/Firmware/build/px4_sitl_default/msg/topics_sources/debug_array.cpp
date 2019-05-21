/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file debug_array.msg */


#include <inttypes.h>
#include <px4_log.h>
#include <px4_defines.h>
#include <uORB/topics/debug_array.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>

constexpr char __orb_debug_array_fields[] = "uint64_t timestamp;float[58] data;uint16_t id;char[10] name;uint8_t[4] _padding0;";

ORB_DEFINE(debug_array, struct debug_array_s, 252, __orb_debug_array_fields);


void print_message(const debug_array_s& message)
{
	PX4_INFO_RAW(" debug_array_s\n");
	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, hrt_elapsed_time(&message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\tdata: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", (double)message.data[0], (double)message.data[1], (double)message.data[2], (double)message.data[3], (double)message.data[4], (double)message.data[5], (double)message.data[6], (double)message.data[7], (double)message.data[8], (double)message.data[9], (double)message.data[10], (double)message.data[11], (double)message.data[12], (double)message.data[13], (double)message.data[14], (double)message.data[15], (double)message.data[16], (double)message.data[17], (double)message.data[18], (double)message.data[19], (double)message.data[20], (double)message.data[21], (double)message.data[22], (double)message.data[23], (double)message.data[24], (double)message.data[25], (double)message.data[26], (double)message.data[27], (double)message.data[28], (double)message.data[29], (double)message.data[30], (double)message.data[31], (double)message.data[32], (double)message.data[33], (double)message.data[34], (double)message.data[35], (double)message.data[36], (double)message.data[37], (double)message.data[38], (double)message.data[39], (double)message.data[40], (double)message.data[41], (double)message.data[42], (double)message.data[43], (double)message.data[44], (double)message.data[45], (double)message.data[46], (double)message.data[47], (double)message.data[48], (double)message.data[49], (double)message.data[50], (double)message.data[51], (double)message.data[52], (double)message.data[53], (double)message.data[54], (double)message.data[55], (double)message.data[56], (double)message.data[57]);
	PX4_INFO_RAW("\tid: %u\n", message.id);
	PX4_INFO_RAW("\tname: [%c, %c, %c, %c, %c, %c, %c, %c, %c, %c]\n", message.name[0], message.name[1], message.name[2], message.name[3], message.name[4], message.name[5], message.name[6], message.name[7], message.name[8], message.name[9]);
	
}
