/*
 * Copyright © 2016-2017 The TokTok team.
 * Copyright © 2013-2015 Tox project.
 *
 * This file is part of Tox, the free peer to peer instant messenger.
 *
 * Tox is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Tox is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Tox.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef VIDEO_H
#define VIDEO_H

#include "toxav.h"

#include "../toxcore/logger.h"
#include "../toxcore/util.h"

#include <vpx/vpx_decoder.h>
#include <vpx/vpx_encoder.h>
#include <vpx/vpx_image.h>

#include <vpx/vp8cx.h>
#include <vpx/vp8dx.h>


// Zoff --
// -- VP8 codec ----------------
#define VIDEO_CODEC_DECODER_INTERFACE_VP8 (vpx_codec_vp8_dx())
#define VIDEO_CODEC_ENCODER_INTERFACE_VP8 (vpx_codec_vp8_cx())
// -- VP9 codec ----------------
#define VIDEO_CODEC_DECODER_INTERFACE_VP9 (vpx_codec_vp9_dx())
#define VIDEO_CODEC_ENCODER_INTERFACE_VP9 (vpx_codec_vp9_cx())
// Zoff --

#define VIDEO_CODEC_DECODER_MAX_WIDTH  (800) // (16384)
#define VIDEO_CODEC_DECODER_MAX_HEIGHT (600) // (16384)


#define VIDEO_SEND_X_KEYFRAMES_FIRST (4) // force the first 8 frames to be keyframes!
#define VPX_MAX_DIST_NORMAL (48)
#define VPX_MAX_DIST_START (48)

#define VPX_MAX_ENCODER_THREADS (4)
#define VPX_MAX_DECODER_THREADS (4)

#define VPX_VP8_CODEC (0)
#define VPX_VP9_CODEC (1)

#define VPX_ENCODER_USED VPX_VP8_CODEC
#define VPX_DECODER_USED VPX_VP8_CODEC // this will switch automatically


#include <pthread.h>

struct RTPMessage;
struct RingBuffer;

typedef struct VCSession_s {
    /* encoding */
    vpx_codec_ctx_t encoder[1];
    uint32_t frame_counter;

    /* decoding */
    vpx_codec_ctx_t decoder[1];
    int8_t is_using_vp9;
    struct RingBuffer *vbuf_raw; /* Un-decoded data */

    uint64_t linfts; /* Last received frame time stamp */
    uint32_t lcfd; /* Last calculated frame duration for incoming video payload */

    Logger *log;
    ToxAV *av;
    uint32_t friend_number;

    PAIR(toxav_video_receive_frame_cb *, void *) vcb; /* Video frame receive callback */

    pthread_mutex_t queue_mutex[1];
} VCSession;

VCSession *vc_new(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data);
void vc_kill(VCSession *vc);
void vc_iterate(VCSession *vc);
int vc_queue_message(void *vcp, struct RTPMessage *msg);
int vc_reconfigure_encoder(VCSession *vc, uint32_t bit_rate, uint16_t width, uint16_t height, int16_t kf_max_dist);

#endif /* VIDEO_H */
