/*
 * Copyright © 2018 zoff@zoff.cc and mail@strfry.org
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

#include <stdint.h>
#include <x264.h>

#include "../../audio.h"
#include "../../video.h"
#include "../../msi.h"
#include "../../ring_buffer.h"
#include "../../rtp.h"
#include "../../tox_generic.h"
#include "../../../toxcore/mono_time.h"
#include "../toxav_codecs.h"

/* ---------------------------------------------------
 *
 * Hardware specific defines for encoders and decoder
 * use -DXXXXXX to enable at compile time, otherwise defaults will be used
 *
 * ---------------------------------------------------
 */


/* ---------------------------------------------------
 * DEFAULT
 */
#define ACTIVE_HW_CODEC_CONFIG_NAME "_DEFAULT_"
#define H264_WANT_ENCODER_NAME "not used with x264"
#define H264_WANT_DECODER_NAME "h264"
#define X264_ENCODE_USED 1
// #define RAPI_HWACCEL_ENC 1
// #define RAPI_HWACCEL_DEC 1
/* multithreaded encoding seems to add less delay (0 .. disable) */
#define X264_ENCODER_THREADS 4
#define X264_ENCODER_SLICES 4
#define H264_ENCODE_MAX_BITRATE_OVER_ALLOW 1.3
#define H264_ENCODER_STARTWITH_PROFILE_HIGH 0
/* ---------------------------------------------------
 * DEFAULT
 */

struct x264_encoder_t {
        x264_t *h264_encoder;
        x264_picture_t in_pic;
        x264_picture_t out_pic;
};


int global_h264_enc_profile_high_enabled = 0;
int global_h264_enc_profile_high_enabled_switch = 0;


Logger *global__log = NULL;

void my_log_callback(void *ptr, int level, const char *fmt, va_list vargs)
{
    LOGGER_WARNING(global__log, fmt, vargs);
}

VCSession *vc_new_x264_encoder(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data,
                       VCSession *vc)
{
    vc->x264_encoder = malloc(sizeof(struct x264_encoder_t));
    memset(vc->x264_encoder, 0, sizeof(struct x264_encoder_t));
    // ENCODER -------

    LOGGER_WARNING(log, "HW CODEC CONFIG ACTIVE: %s", ACTIVE_HW_CODEC_CONFIG_NAME);

    x264_param_t param;

    // -- set inital value for H264 encoder profile --
    global_h264_enc_profile_high_enabled = H264_ENCODER_STARTWITH_PROFILE_HIGH;
    // -- set inital value for H264 encoder profile --

    // "ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow", "slower", "veryslow", "placebo"

    if (global_h264_enc_profile_high_enabled == 1) {
        if (x264_param_default_preset(&param, "superfast", "zerolatency,fastdecode") < 0) {
            // goto fail;
        }
    } else {
        if (x264_param_default_preset(&param, "ultrafast", "zerolatency,fastdecode") < 0) {
            // goto fail;
        }
    }

    /* Configure non-default params */
    // param.i_bitdepth = 8;
    param.i_csp = X264_CSP_I420;
    param.i_width  = 1920;
    param.i_height = 1080;
    vc->h264_enc_width = param.i_width;
    vc->h264_enc_height = param.i_height;

    param.i_threads = X264_ENCODER_THREADS;
    param.b_sliced_threads = true;
    param.i_slice_count = X264_ENCODER_SLICES;

    param.b_deterministic = false;
    //#// param.i_sync_lookahead = 0;
    //#// param.i_lookahead_threads = 0;
    //**// param.b_intra_refresh = 1;
    param.i_bframe = 0;
    // param.b_open_gop = 20;
    param.i_keyint_max = VIDEO_MAX_KF_H264;
    // param.rc.i_rc_method = X264_RC_CRF; // X264_RC_ABR;
    // param.i_nal_hrd = X264_NAL_HRD_CBR;

    //#// param.i_frame_reference = 1;

    param.b_vfr_input = 1; /* VFR input.  If 1, use timebase and timestamps for ratecontrol purposes.
                            * If 0, use fps only. */
    param.i_timebase_num = 1;       // 1 ms = timebase units = (1/1000)s
    param.i_timebase_den = 1000;   // 1 ms = timebase units = (1/1000)s
    param.b_repeat_headers = 1;
    param.b_annexb = 1;

    param.rc.f_rate_tolerance = VIDEO_F_RATE_TOLERANCE_H264;
    param.rc.i_vbv_buffer_size = VIDEO_BITRATE_INITIAL_VALUE_H264 * VIDEO_BUF_FACTOR_H264;
    param.rc.i_vbv_max_bitrate = VIDEO_BITRATE_INITIAL_VALUE_H264 * 1;
    // param.rc.i_bitrate = VIDEO_BITRATE_INITIAL_VALUE_H264 * VIDEO_BITRATE_FACTOR_H264;

    param.rc.i_qp_min = 3;
    param.rc.i_qp_max = 51; // max quantizer for x264

    vc->h264_enc_bitrate = VIDEO_BITRATE_INITIAL_VALUE_H264 * 1000;

    param.rc.b_stat_read = 0;
    param.rc.b_stat_write = 0;

    //x264_param_apply_fastfirstpass(&param);

    /* Apply profile restrictions. */

    if (global_h264_enc_profile_high_enabled == 1) {
        if (x264_param_apply_profile(&param,
                                     "high") < 0) { // "baseline", "main", "high", "high10", "high422", "high444"
            // goto fail;
            LOGGER_WARNING(log, "h264: setting high encoder failed");
        } else {
            LOGGER_WARNING(log, "h264: setting high encoder OK");
        }
    } else {
        if (x264_param_apply_profile(&param,
                                     "baseline") < 0) { // "baseline", "main", "high", "high10", "high422", "high444"
            // goto fail;
            LOGGER_WARNING(log, "h264: setting BASELINE encoder failed");
        } else {
            LOGGER_WARNING(log, "h264: setting BASELINE encoder OK");
        }
    }

    if (x264_picture_alloc(&(vc->x264_encoder->in_pic), param.i_csp, param.i_width, param.i_height) < 0) {
        // goto fail;
    }

    // vc->x264_encoder->in_pic.img.plane[0] --> Y
    // vc->x264_encoder->in_pic.img.plane[1] --> U
    // vc->x264_encoder->in_pic.img.plane[2] --> V

    vc->x264_encoder->h264_encoder  = x264_encoder_open(&param);

    return vc;
}



int vc_reconfigure_x264_encoder(Logger *log, VCSession *vc, uint32_t bit_rate,
                                uint16_t width, uint16_t height,
                                int16_t kf_max_dist)
{
    if (!vc) {
        return -1;
    }

    if (global_h264_enc_profile_high_enabled_switch == 1) {
        global_h264_enc_profile_high_enabled_switch = 0;
        kf_max_dist = -2;
        LOGGER_WARNING(log, "switching H264 encoder profile ...");
    }

    if ((vc->h264_enc_width == width) &&
            (vc->h264_enc_height == height) &&
            (vc->video_rc_max_quantizer == vc->video_rc_max_quantizer_prev) &&
            (vc->video_rc_min_quantizer == vc->video_rc_min_quantizer_prev) &&
            (vc->h264_enc_bitrate != bit_rate) &&
            (kf_max_dist != -2)) {
        // only bit rate changed

        if (vc->x264_encoder) {

            x264_param_t param;
            x264_encoder_parameters(vc->x264_encoder->h264_encoder, &param);

            LOGGER_DEBUG(log, "vc_reconfigure_encoder_h264:vb=%d [bitrate only]", (int)(bit_rate / 1000));

            param.rc.f_rate_tolerance = VIDEO_F_RATE_TOLERANCE_H264;
            param.rc.i_vbv_buffer_size = (bit_rate / 1000) * VIDEO_BUF_FACTOR_H264;
            param.rc.i_vbv_max_bitrate = (bit_rate / 1000) * 1;

            // param.rc.i_bitrate = (bit_rate / 1000) * VIDEO_BITRATE_FACTOR_H264;
            vc->h264_enc_bitrate = bit_rate;

            int res = x264_encoder_reconfig(vc->x264_encoder->h264_encoder, &param);
        } else {

        }


    } else {
        if ((vc->h264_enc_width != width) ||
                (vc->h264_enc_height != height) ||
                (vc->h264_enc_bitrate != bit_rate) ||
                (vc->video_rc_max_quantizer != vc->video_rc_max_quantizer_prev) ||
                (vc->video_rc_min_quantizer != vc->video_rc_min_quantizer_prev) ||
                (kf_max_dist == -2)
           ) {
            // input image size changed

            if (vc->x264_encoder) {

                x264_param_t param;

                if (global_h264_enc_profile_high_enabled == 1) {
                    if (x264_param_default_preset(&param, "superfast", "zerolatency,fastdecode") < 0) {
                        // goto fail;
                    }
                } else {
                    if (x264_param_default_preset(&param, "ultrafast", "zerolatency,fastdecode") < 0) {
                        // goto fail;
                    }
                }

                /* Configure non-default params */
                // param.i_bitdepth = 8;
                param.i_csp = X264_CSP_I420;
                param.i_width  = width;
                param.i_height = height;
                vc->h264_enc_width = param.i_width;
                vc->h264_enc_height = param.i_height;
                param.i_threads = X264_ENCODER_THREADS;
                param.b_sliced_threads = true;
                param.i_slice_count = X264_ENCODER_SLICES;

                param.b_deterministic = false;
                //#// param.i_sync_lookahead = 0;
                //#// param.i_lookahead_threads = 0;
                //**// param.b_intra_refresh = 1;
                param.i_bframe = 0;
                // param.b_open_gop = 20;
                param.i_keyint_max = VIDEO_MAX_KF_H264;
                // param.rc.i_rc_method = X264_RC_ABR;
                //#// param.i_frame_reference = 1;

                param.b_vfr_input = 1; /* VFR input.  If 1, use timebase and timestamps for ratecontrol purposes.
                            * If 0, use fps only. */
                param.i_timebase_num = 1;       // 1 ms = timebase units = (1/1000)s
                param.i_timebase_den = 1000;   // 1 ms = timebase units = (1/1000)s
                param.b_repeat_headers = 1;
                param.b_annexb = 1;

                LOGGER_ERROR(log, "vc_reconfigure_encoder_h264:vb=%d", (int)(bit_rate / 1000));

                param.rc.f_rate_tolerance = VIDEO_F_RATE_TOLERANCE_H264;
                param.rc.i_vbv_buffer_size = (bit_rate / 1000) * VIDEO_BUF_FACTOR_H264;
                param.rc.i_vbv_max_bitrate = (bit_rate / 1000) * 1;

                if ((vc->video_rc_min_quantizer >= 0) &&
                        (vc->video_rc_min_quantizer < vc->video_rc_max_quantizer) &&
                        (vc->video_rc_min_quantizer < 51)) {
                    param.rc.i_qp_min = vc->video_rc_min_quantizer;
                }

                if ((vc->video_rc_max_quantizer >= 0) &&
                        (vc->video_rc_min_quantizer < vc->video_rc_max_quantizer) &&
                        (vc->video_rc_max_quantizer < 51)) {
                    param.rc.i_qp_max = vc->video_rc_max_quantizer;
                }

                // param.rc.i_bitrate = (bit_rate / 1000) * VIDEO_BITRATE_FACTOR_H264;
                vc->h264_enc_bitrate = bit_rate;

                param.rc.b_stat_read = 0;
                param.rc.b_stat_write = 0;
                //x264_param_apply_fastfirstpass(&param);

                /* Apply profile restrictions. */
                if (global_h264_enc_profile_high_enabled == 1) {
                    if (x264_param_apply_profile(&param,
                                                 "high") < 0) { // "baseline", "main", "high", "high10", "high422", "high444"
                        // goto fail;
                        LOGGER_WARNING(log, "h264: setting high encoder failed (2)");
                    } else {
                        LOGGER_WARNING(log, "h264: setting high encoder OK (2)");
                    }
                } else {
                    if (x264_param_apply_profile(&param,
                                                 "baseline") < 0) { // "baseline", "main", "high", "high10", "high422", "high444"
                        // goto fail;
                        LOGGER_WARNING(log, "h264: setting BASELINE encoder failed (2)");
                    } else {
                        LOGGER_WARNING(log, "h264: setting BASELINE encoder OK (2)");
                    }
                }

                LOGGER_ERROR(log, "H264: reconfigure encoder:001: w:%d h:%d w_new:%d h_new:%d BR:%d\n",
                             vc->h264_enc_width,
                             vc->h264_enc_height,
                             width,
                             height,
                             (int)bit_rate);

                // free old stuff ---------
                x264_encoder_close(vc->x264_encoder->h264_encoder);
                x264_picture_clean(&(vc->x264_encoder->in_pic));
                // free old stuff ---------

                // alloc with new values -------
                if (x264_picture_alloc(&(vc->x264_encoder->in_pic), param.i_csp, param.i_width, param.i_height) < 0) {
                    // goto fail;
                }

                vc->x264_encoder->h264_encoder = x264_encoder_open(&param);
                // alloc with new values -------

                vc->video_rc_max_quantizer_prev = vc->video_rc_max_quantizer;
                vc->video_rc_min_quantizer_prev = vc->video_rc_min_quantizer;

            }
        }
    }

    return 0;
}

//!!!
void get_info_from_sps(const Messenger *m, VCSession *vc, const Logger *log,
                       const uint8_t data[], const uint32_t data_len)
{

    if (data_len > 7) {
        LOGGER_DEBUG(log, "SPS:len=%d bytes:%d %d %d %d %d %d %d %d", data_len, data[0], data[1], data[2], data[3], data[4],
                     data[5], data[6], data[7]);

        if (
            (data[0] == 0x00)
            &&
            (data[1] == 0x00)
            &&
            (data[2] == 0x00)
            &&
            (data[3] == 0x01)
            &&
            ((data[4] & 0x1F) == 7) // only the lower 5bits of the 4th byte denote the NAL type
            // 7 --> SPS
            // 8 --> PPS
            // (data[4] == 0x67)
        ) {
            // parse only every 5 seconds
            if ((vc->last_parsed_h264_sps_ts + 5000) < current_time_monotonic(m->mono_time)) {
                vc->last_parsed_h264_sps_ts = current_time_monotonic(m->mono_time);

                // we found a NAL unit containing the SPS
                uint8_t h264_profile = data[5];
                uint8_t h264_constraint_set0_flag = ((data[6] >> 3)  & 0x01);
                uint8_t h264_constraint_set3_flag = (data[6]  & 0x01);
                uint8_t h264_level = data[7];

                if ((h264_profile == 66) && (h264_constraint_set3_flag = 0)) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "baseline", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if ((h264_profile == 66) && (h264_constraint_set3_flag = 1)) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "contrained baseline", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if ((h264_profile == 77) && (h264_constraint_set0_flag = 0)) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "main", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if ((h264_profile == 77) && (h264_constraint_set0_flag = 1)) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "extended", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if (h264_profile == 100) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "high", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if (h264_profile == 110) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "high10", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if (h264_profile == 122) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "high422", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else if (h264_profile == 244) {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "high444", h264_level);
                    vc->parsed_h264_sps_profile_i = h264_profile;
                } else {
                    LOGGER_DEBUG(log, "profile=%s level=%d", "unkwn", h264_level);
                    vc->parsed_h264_sps_profile_i = 0;
                }

                vc->parsed_h264_sps_level_i = h264_level;
            }
        }
    }
}


int32_t global_encoder_delay_counter = 0;

uint32_t encode_frame_x264(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                           const uint8_t *y,
                           const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                           uint64_t *video_frame_record_timestamp,
                           int vpx_encode_flags,
                           x264_nal_t **nal,
                           int *i_frame_size)
{

    memcpy(call->video->x264_encoder->in_pic.img.plane[0], y, width * height);
    memcpy(call->video->x264_encoder->in_pic.img.plane[1], u, (width / 2) * (height / 2));
    memcpy(call->video->x264_encoder->in_pic.img.plane[2], v, (width / 2) * (height / 2));

    int i_nal;

    call->video->x264_encoder->in_pic.i_pts = (int64_t)(*video_frame_record_timestamp);

    if ((vpx_encode_flags & VPX_EFLAG_FORCE_KF) > 0) {
        call->video->x264_encoder->in_pic.i_type = X264_TYPE_IDR; // real full i-frame
        call->video->last_sent_keyframe_ts = current_time_monotonic(av->m->mono_time);
    } else {
        call->video->x264_encoder->in_pic.i_type = X264_TYPE_AUTO;
    }

    LOGGER_DEBUG(av->m->log, "X264 IN frame type=%d", (int)call->video->x264_encoder->in_pic.i_type);

    *i_frame_size = x264_encoder_encode(call->video->x264_encoder->h264_encoder,
                                        nal,
                                        &i_nal,
                                        &(call->video->x264_encoder->in_pic),
                                        &(call->video->x264_encoder->out_pic));

    global_encoder_delay_counter++;

    if (global_encoder_delay_counter > 60) {
        global_encoder_delay_counter = 0;
        LOGGER_DEBUG(av->m->log, "enc:delay=%ld",
                     (long int)(call->video->x264_encoder->in_pic.i_pts - (int64_t)call->video->x264_encoder->out_pic.i_pts)
                    );
    }


    *video_frame_record_timestamp = (uint64_t)call->video->x264_encoder->out_pic.i_pts;




    if (IS_X264_TYPE_I(call->video->x264_encoder->out_pic.i_type)) {
        LOGGER_DEBUG(av->m->log, "X264 out frame type=%d", (int)call->video->x264_encoder->out_pic.i_type);
    }


    if (*i_frame_size < 0) {
        // some error
    } else if (*i_frame_size == 0) {
        // zero size output
    } else {
        // *nal->p_payload --> outbuf
        // *i_frame_size --> out size in bytes

        // -- WARN -- : this could crash !! ----
        // LOGGER_ERROR(av->m->log, "H264: i_frame_size=%d nal_buf=%p KF=%d\n",
        //             (int)*i_frame_size,
        //             (*nal)->p_payload,
        //             (int)call->video->x264_encoder->out_pic.b_keyframe
        //            );
        // -- WARN -- : this could crash !! ----

    }

    if (*nal == NULL) {
        //pthread_mutex_unlock(call->mutex_video);
        //goto END;
        return 1;
    }

    if ((*nal)->p_payload == NULL) {
        //pthread_mutex_unlock(call->mutex_video);
        //goto END;
        return 1;
    }

    return 0;
}

uint32_t send_frames_h264(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                          const uint8_t *y,
                          const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                          uint64_t *video_frame_record_timestamp,
                          int vpx_encode_flags,
                          x264_nal_t **nal,
                          int *i_frame_size,
                          TOXAV_ERR_SEND_FRAME *rc)
{

#ifdef X264_ENCODE_USED


    if (*i_frame_size > 0) {

        // use the record timestamp that was actually used for this frame
        *video_frame_record_timestamp = (uint64_t)call->video->x264_encoder->in_pic.i_pts; // TODO: --> is this wrong?
        const uint32_t frame_length_in_bytes = *i_frame_size;
        const int keyframe = (int)call->video->x264_encoder->out_pic.b_keyframe;

        // LOGGER_ERROR(av->m->log, "video packet record time[1]: %lu", (*video_frame_record_timestamp));

        int res = rtp_send_data
                  (
                      call->video_rtp,
                      (const uint8_t *)((*nal)->p_payload),
                      frame_length_in_bytes,
                      keyframe,
                      *video_frame_record_timestamp,
                      (int32_t)0,
                      TOXAV_ENCODER_CODEC_USED_H264,
                      call->video_bit_rate,
                      call->video->client_video_capture_delay_ms,
                      call->video->video_encoder_frame_orientation_angle,
                      av->m->log
                  );

        (*video_frame_record_timestamp)++;

        if (res < 0) {
            LOGGER_WARNING(av->m->log, "Could not send video frame: %s", strerror(errno));
            *rc = TOXAV_ERR_SEND_FRAME_RTP_FAILED;
            return 1;
        }

        return 0;
    } else {
        *rc = TOXAV_ERR_SEND_FRAME_RTP_FAILED;
        return 1;
    }

#else

    if (*i_frame_size > 0) {

        *video_frame_record_timestamp = (uint64_t)call->video->x264_encoder->out_pic2->pts;
        const uint32_t frame_length_in_bytes = *i_frame_size;
        const int keyframe = (int)1;

        // LOGGER_ERROR(av->m->log, "video packet record time[1]: %lu", (*video_frame_record_timestamp));
        *video_frame_record_timestamp = current_time_monotonic(av->m->mono_time);
        // LOGGER_ERROR(av->m->log, "video packet record time[2]: %lu", (*video_frame_record_timestamp));


        LOGGER_DEBUG(av->m->log, "call->video->video_encoder_frame_orientation_angle==%d",
                     call->video->video_encoder_frame_orientation_angle);

        int res = rtp_send_data
                  (
                      call->video_rtp,
                      (const uint8_t *)(call->video->x264_encoder->out_pic2->data),
                      frame_length_in_bytes,
                      keyframe,
                      *video_frame_record_timestamp,
                      (int32_t)0,
                      TOXAV_ENCODER_CODEC_USED_H264,
                      call->video_bit_rate,
                      call->video->client_video_capture_delay_ms,
                      call->video->video_encoder_frame_orientation_angle,
                      av->m->log
                  );

        (*video_frame_record_timestamp)++;


        av_packet_unref(call->video->x264_encoder->out_pic2);

        if (res < 0) {
            LOGGER_WARNING(av->m->log, "Could not send video frame: %s", strerror(errno));
            *rc = TOXAV_ERR_SEND_FRAME_RTP_FAILED;
            return 1;
        }

        return 0;

    } else {
        *rc = TOXAV_ERR_SEND_FRAME_RTP_FAILED;
        return 1;
    }

#endif

}

void vc_kill_x264(VCSession *vc)
{
    if (vc->x264_encoder) {
        x264_encoder_close(vc->x264_encoder->h264_encoder);
        x264_picture_clean(&(vc->x264_encoder->in_pic));
    }
    free(vc->x264_encoder);
    vc->x264_encoder = 0;

}


