/*
 * Copyright © 2019 zoff@zoff.cc and mail@strfry.org
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

#include "video.h"


/* !!multithreaded H264 decoding adds about 80ms of delay!! (0 .. disable, 1 .. disable also?) */
#define H264_DECODER_THREADS 4
#define H264_DECODER_THREAD_FRAME_ACTIVE 1

void vc_kill_libavcodec_decode(VCSession *vc)
{
    // decoder
    if (vc->h264_decoder->extradata) {
        free(vc->h264_decoder->extradata);
        vc->h264_decoder->extradata = NULL;
    }

    avcodec_free_context(&vc->h264_decoder);
}


VCSession *vc_new_libavcodec_decoder(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data,
                       VCSession *vc)
{
    AVCodec *codec = NULL;
#ifdef RAPI_HWACCEL_DEC

    codec = avcodec_find_decoder_by_name(H264_WANT_DECODER_NAME);

    if (!codec) {
        LOGGER_WARNING(log, "codec not found HW Accel H264 on decoder, trying software decoder ...");
        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    } else {
        LOGGER_WARNING(log, "FOUND: *HW Accel* H264 on decoder");
    }

#else
    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
#endif


    if (!codec) {
        LOGGER_WARNING(log, "codec not found H264 on decoder");
    }

    vc->h264_decoder = avcodec_alloc_context3(codec);

    if (codec)
    {
        if (codec->capabilities & AV_CODEC_CAP_TRUNCATED) {
            vc->h264_decoder->flags |= AV_CODEC_FLAG_TRUNCATED; /* we do not send complete frames */
        }

        if (codec->capabilities & AV_CODEC_FLAG_LOW_DELAY) {
            vc->h264_decoder->flags |= AV_CODEC_FLAG_LOW_DELAY;
        }

        // vc->h264_decoder->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
        vc->h264_decoder->flags |= AV_CODEC_FLAG2_SHOW_ALL;
        // vc->h264_decoder->flags2 |= AV_CODEC_FLAG2_FAST;
        // vc->h264_decoder->flags |= AV_CODEC_FLAG_TRUNCATED;
        // vc->h264_decoder->flags2 |= AV_CODEC_FLAG2_CHUNKS;

        if (H264_DECODER_THREADS > 0) {
            if (codec->capabilities & AV_CODEC_CAP_SLICE_THREADS) {
                vc->h264_decoder->thread_count = H264_DECODER_THREADS;
                vc->h264_decoder->thread_type = FF_THREAD_SLICE;
                vc->h264_decoder->active_thread_type = FF_THREAD_SLICE;
            }

            if (H264_DECODER_THREAD_FRAME_ACTIVE == 1) {
                if (codec->capabilities & AV_CODEC_CAP_FRAME_THREADS) {
                    vc->h264_decoder->thread_count = H264_DECODER_THREADS;
                    vc->h264_decoder->thread_type |= FF_THREAD_FRAME;
                    vc->h264_decoder->active_thread_type |= FF_THREAD_FRAME;
                }
            }
        }

    #if (defined (HW_CODEC_CONFIG_RPI3_TBW_TV) || defined (HW_CODEC_CONFIG_RPI3_TBW_BIDI)) && defined (RAPI_HWACCEL_DEC)
        LOGGER_WARNING(log, "setting up h264_mmal decoder ...");
        av_opt_set_int(vc->h264_decoder->priv_data, "extra_buffers)", 20, AV_OPT_SEARCH_CHILDREN);
        av_opt_set_int(vc->h264_decoder->priv_data, "extra_decoder_buffers)", 20, AV_OPT_SEARCH_CHILDREN);
        LOGGER_WARNING(log, "extra_buffers, extra_decoder_buffers");
    #endif


        vc->h264_decoder->refcounted_frames = 0;
        /*   When AVCodecContext.refcounted_frames is set to 0, the returned
        *             reference belongs to the decoder and is valid only until the
        *             next call to this function or until closing or flushing the
        *             decoder. The caller may not write to it.
        */

        vc->h264_decoder->delay = 0;
        
        #if 0 // This is broken on Mint 19.04, libavcodec Version: 7:3.4.6-0ubuntu0.18.04.1
        av_opt_set_int(vc->h264_decoder->priv_data, "delay", 0, AV_OPT_SEARCH_CHILDREN);
        #endif

        vc->h264_decoder->time_base = (AVRational) {
            40, 1000
        };
        vc->h264_decoder->framerate = (AVRational) {
            1000, 40
        };

        if (avcodec_open2(vc->h264_decoder, codec, NULL) < 0) {
            LOGGER_WARNING(log, "could not open codec H264 on decoder");
        }

        vc->h264_decoder->refcounted_frames = 0;
        /*   When AVCodecContext.refcounted_frames is set to 0, the returned
        *             reference belongs to the decoder and is valid only until the
        *             next call to this function or until closing or flushing the
        *             decoder. The caller may not write to it.
        */
    }


    // DECODER -------

    return vc;
}