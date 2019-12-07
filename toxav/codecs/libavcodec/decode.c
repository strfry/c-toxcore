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


//int32_t global_first_video_frame_data = 0;
int32_t global_decoder_delay_counter = 0;

void vc_decode_frame_h264_libavcodec(VCSession *vc, Messenger *m, uint8_t skip_video_flag, uint64_t *a_r_timestamp,
                       uint64_t *a_l_timestamp,
                       uint64_t *v_r_timestamp, uint64_t *v_l_timestamp,
                       const struct RTPHeader *header_v3,
                       struct RTPMessage *p, vpx_codec_err_t rc,
                       uint32_t full_data_len,
                       uint8_t *ret_value)
{

    LOGGER_DEBUG(vc->log, "decode_frame_h264:len=%d", full_data_len);

    if (p == NULL)
    {
        LOGGER_DEBUG(vc->log, "decode_frame_h264:NO data");
        return;
    }

    if (p->data == NULL)
    {
        LOGGER_DEBUG(vc->log, "decode_frame_h264:NO data p->data");
        return;
    }

    if (full_data_len < 1)
    {
        LOGGER_DEBUG(vc->log, "decode_frame_h264:not enough data");
        free(p);
        p = NULL;
        return;
    }

    if (vc->h264_decoder == NULL)
    {
        LOGGER_DEBUG(vc->log, "vc->h264_decoder:not ready");
        free(p);
        p = NULL;
        return;
    }

#if 0

    if ((p) && (p->data)) {
        global_first_video_frame_data++;

        if (global_first_video_frame_data == 250) {
            FILE *pFile = NULL;
            pFile = fopen("h264_frame.txt", "w");

            if (pFile) {
                fwrite(p->data, 1, full_data_len, pFile);

            }

            fclose(pFile);
            global_first_video_frame_data = 1;
        }
    }

#endif

    if ((p) && (p->data)) {
        get_info_from_sps(m, vc, vc->log, p->data, full_data_len);
    }

    /*
     For decoding, call avcodec_send_packet() to give the decoder raw
          compressed data in an AVPacket.


          For decoding, call avcodec_receive_frame(). On success, it will return
          an AVFrame containing uncompressed audio or video data.


     *   Repeat this call until it returns AVERROR(EAGAIN) or an error. The
     *   AVERROR(EAGAIN) return value means that new input data is required to
     *   return new output. In this case, continue with sending input. For each
     *   input frame/packet, the codec will typically return 1 output frame/packet,
     *   but it can also be 0 or more than 1.

     */

    AVPacket *compr_data = NULL;
    compr_data = av_packet_alloc();

    if (compr_data == NULL)
    {
        LOGGER_DEBUG(vc->log, "av_packet_alloc:ERROR");
        free(p);
        p = NULL;
        return;
    }

    uint64_t h_frame_record_timestamp = header_v3->frame_record_timestamp;

#if 0
    compr_data->pts = AV_NOPTS_VALUE;
    compr_data->dts = AV_NOPTS_VALUE;

    compr_data->duration = 0;
    compr_data->post = -1;
#endif

    // uint32_t start_time_ms = current_time_monotonic(m->mono_time);
    // HINT: dirty hack to add FF_INPUT_BUFFER_PADDING_SIZE bytes!! ----------
    uint8_t *tmp_buf = calloc(1, full_data_len + AV_INPUT_BUFFER_PADDING_SIZE);
    memcpy(tmp_buf, p->data, full_data_len);
    // HINT: dirty hack to add FF_INPUT_BUFFER_PADDING_SIZE bytes!! ----------
    // uint32_t end_time_ms = current_time_monotonic(m->mono_time);
    // LOGGER_WARNING(vc->log, "decode_frame_h264:001: %d ms", (int)(end_time_ms - start_time_ms));

    compr_data->data = tmp_buf; // p->data;
    compr_data->size = (int)full_data_len; // hmm, "int" again

    if (header_v3->frame_record_timestamp > 0) {
        compr_data->dts = (int64_t)(header_v3->frame_record_timestamp) + 0;
        compr_data->pts = (int64_t)(header_v3->frame_record_timestamp) + 1;
        compr_data->duration = 0; // (int64_t)(header_v3->frame_record_timestamp) + 1; // 0;
    }

    /* ------------------------------------------------------- */
    /* ------------------------------------------------------- */
    /* HINT: this is the only part that takes all the time !!! */
    /* HINT: this is the only part that takes all the time !!! */
    /* HINT: this is the only part that takes all the time !!! */

    // uint32_t start_time_ms = current_time_monotonic(m->mono_time);

    /* ------------------------------------------------------- */
    /* ------------------------------------------------------- */
    // The input buffer, avpkt->data must be AV_INPUT_BUFFER_PADDING_SIZE larger than the actual
    // read bytes because some optimized bitstream readers read 32 or 64 bits at once and
    // could read over the end.
    /* ------------------------------------------------------- */
    /* ------------------------------------------------------- */

    int result_send_packet = avcodec_send_packet(vc->h264_decoder, compr_data);
    if (result_send_packet != 0)
    {
        LOGGER_DEBUG(vc->log, "avcodec_send_packet:ERROR=%d", result_send_packet);
        av_packet_free(&compr_data);
        free(tmp_buf);
        free(p);
        p = NULL;
        return;
    }
    // uint32_t end_time_ms = current_time_monotonic(m->mono_time);
    // if ((int)(end_time_ms - start_time_ms) > 4) {
    //    LOGGER_WARNING(vc->log, "decode_frame_h264:002: %d ms", (int)(end_time_ms - start_time_ms));
    //}

    /* HINT: this is the only part that takes all the time !!! */
    /* HINT: this is the only part that takes all the time !!! */
    /* HINT: this is the only part that takes all the time !!! */
    /* ------------------------------------------------------- */
    /* ------------------------------------------------------- */


    int ret_ = 0;

    while (ret_ >= 0) {

        // start_time_ms = current_time_monotonic(m->mono_time);
        AVFrame *frame = av_frame_alloc();
        if (frame == NULL)
        {
            // stop decoding
            break;
        }
        // end_time_ms = current_time_monotonic(m->mono_time);
        // LOGGER_WARNING(vc->log, "decode_frame_h264:003: %d ms", (int)(end_time_ms - start_time_ms));

        // start_time_ms = current_time_monotonic(m->mono_time);
        ret_ = avcodec_receive_frame(vc->h264_decoder, frame);
        // end_time_ms = current_time_monotonic(m->mono_time);
        // LOGGER_WARNING(vc->log, "decode_frame_h264:004: %d ms", (int)(end_time_ms - start_time_ms));

        // LOGGER_ERROR(vc->log, "H264:decoder:ret_=%d\n", (int)ret_);


        if (ret_ == AVERROR(EAGAIN) || ret_ == AVERROR_EOF) {
            // error
            av_frame_free(&frame);
            break;
        } else if (ret_ < 0) {
            // Error during decoding
            av_frame_free(&frame);
            break;
        } else if (ret_ == 0) {

            // LOGGER_ERROR(vc->log, "H264:decoder:fnum=%d\n", (int)vc->h264_decoder->frame_number);
            // LOGGER_ERROR(vc->log, "H264:decoder:linesize=%d\n", (int)frame->linesize[0]);
            // LOGGER_ERROR(vc->log, "H264:decoder:w=%d\n", (int)frame->width);
            // LOGGER_ERROR(vc->log, "H264:decoder:h=%d\n", (int)frame->height);


            /*
                pkt_pts
                PTS copied from the AVPacket that was decoded to produce this frame.

                pkt_dts
                DTS copied from the AVPacket that triggered returning this frame.
            */

            // calculate the real play delay (from toxcore-in to toxcore-out)
            // this seems to give incorrect values :-(
            if (header_v3->frame_record_timestamp > 0) {

                //vc->video_play_delay_real =
                //    (current_time_monotonic(m->mono_time) + vc->timestamp_difference_to_sender) -
                //    frame->pkt_pts;
                // use the calculated values instead
                //vc->video_play_delay_real = vc->video_play_delay;

                /*
                 * TODO: there is some memory issue in the log line. DO NOT ENABLE !! ---------
                                LOGGER_DEBUG(vc->log,
                                               "real play delay=%d header_v3->frame_record_timestamp=%d, mono=%d, vc->timestamp_difference_to_sender=%d frame->pkt_pts=%ld, frame->pkt_dts=%ld, frame->pts=%ld",
                                               (int)vc->video_play_delay_real,
                                               (int)header_v3->frame_record_timestamp,
                                               (int)current_time_monotonic(m->mono_time),
                                               (int)vc->timestamp_difference_to_sender,
                                               frame->pkt_pts,
                                               frame->pkt_dts,
                                               frame->pts
                                              );
                 *
                 * TODO: there is some memory issue in the log line. DO NOT ENABLE !! ---------
                 */



                /*


                MMAL H264 Decoder - problem ?
                =============================

                 fps    ms delay    ms between frames   frames cached
                ---------------------------------------------------------
                 10     450         100                 ~4.5
                 20     230         50                  ~4.6
                 25     190         40                  ~4.75

                */

                global_decoder_delay_counter++;

                if (global_decoder_delay_counter > 60) {
                    global_decoder_delay_counter = 0;
                    LOGGER_DEBUG(vc->log, "dec:delay=%ld",
                                 (long int)(h_frame_record_timestamp - frame->pkt_pts)
                                );
                }
            }

            // start_time_ms = current_time_monotonic(m->mono_time);
            if ((frame->data[0] != NULL) && (frame->data[1] != NULL) && (frame->data[2] != NULL))
            {
                vc->vcb(vc->av, vc->friend_number, frame->width, frame->height,
                        (const uint8_t *)frame->data[0],
                        (const uint8_t *)frame->data[1],
                        (const uint8_t *)frame->data[2],
                        frame->linesize[0], frame->linesize[1],
                        frame->linesize[2], vc->vcb_user_data);
            }
            // end_time_ms = current_time_monotonic(m->mono_time);
            // LOGGER_WARNING(vc->log, "decode_frame_h264:005: %d ms", (int)(end_time_ms - start_time_ms));

        } else {
            // some other error
        }

        // start_time_ms = current_time_monotonic(m->mono_time);
        av_frame_free(&frame);
        // end_time_ms = current_time_monotonic(m->mono_time);
        // LOGGER_WARNING(vc->log, "decode_frame_h264:006: %d ms", (int)(end_time_ms - start_time_ms));
    }

    // start_time_ms = current_time_monotonic(m->mono_time);
    av_packet_free(&compr_data);
    // end_time_ms = current_time_monotonic(m->mono_time);
    // LOGGER_WARNING(vc->log, "decode_frame_h264:007: %d ms", (int)(end_time_ms - start_time_ms));

    // HINT: dirty hack to add FF_INPUT_BUFFER_PADDING_SIZE bytes!! ----------
    free(tmp_buf);
    // HINT: dirty hack to add FF_INPUT_BUFFER_PADDING_SIZE bytes!! ----------

    free(p);
}