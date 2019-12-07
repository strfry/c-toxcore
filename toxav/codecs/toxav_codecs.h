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


// ----------- COMMON STUFF -----------
/*
  Soft deadline the decoder should attempt to meet, in "us" (microseconds). Set to zero for unlimited.
  By convention, the value 1 is used to mean "return as fast as possible."
*/
// TODO: don't hardcode this, let the application choose it
#define WANTED_MAX_DECODER_FPS (20)
#define MAX_DECODE_TIME_US (1000000 / WANTED_MAX_DECODER_FPS) // to allow x fps
/*
VPX_DL_REALTIME       (1)
deadline parameter analogous to VPx REALTIME mode.

VPX_DL_GOOD_QUALITY   (1000000)
deadline parameter analogous to VPx GOOD QUALITY mode.

VPX_DL_BEST_QUALITY   (0)
deadline parameter analogous to VPx BEST QUALITY mode.
*/


// initialize encoder with this value. Target bandwidth to use for this stream, in kilobits per second.
#define VIDEO_BITRATE_INITIAL_VALUE 1000
#define VIDEO_BITRATE_INITIAL_VALUE_VP9 1000

struct vpx_frame_user_data {
    uint64_t record_timestamp;
};
// ----------- COMMON STUFF -----------


// TODO: Remove this dependency from generic codec API headers
#ifndef USE_X264
// (c) 2019 x264 Project
/* The data within the payload is already NAL-encapsulated; the ref_idc and type
 * are merely in the struct for easy access by the calling application.
 * All data returned in an x264_nal_t, including the data in p_payload, is no longer
 * valid after the next call to x264_encoder_encode.  Thus it must be used or copied
 * before calling x264_encoder_encode or x264_encoder_headers again. */
typedef struct
{
    int i_ref_idc;  /* nal_priority_e */
    int i_type;     /* nal_unit_type_e */
    int b_long_startcode;
    int i_first_mb; /* If this NAL is a slice, the index of the first MB in the slice. */
    int i_last_mb;  /* If this NAL is a slice, the index of the last MB in the slice. */

    /* Size of payload (including any padding) in bytes. */
    int     i_payload;
    /* If param->b_annexb is set, Annex-B bytestream with startcode.
     * Otherwise, startcode is replaced with a 4-byte size.
     * This size is the size used in mp4/similar muxing; it is equal to i_payload-4 */
    uint8_t *p_payload;

    /* Size of padding in bytes. */
    int i_padding;
} h264_nal_t;

typedef h264_nal_t x264_nal_t; // HACK

#else
#endif



// ----------- VPX  -----------
VCSession *vc_new_vpx(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data,
                      VCSession *vc);

int vc_reconfigure_encoder_vpx(Logger *log, VCSession *vc, uint32_t bit_rate,
                               uint16_t width, uint16_t height,
                               int16_t kf_max_dist);

void decode_frame_vpx(VCSession *vc, Messenger *m, uint8_t skip_video_flag, uint64_t *a_r_timestamp,
                      uint64_t *a_l_timestamp,
                      uint64_t *v_r_timestamp, uint64_t *v_l_timestamp,
                      const struct RTPHeader *header_v3,
                      struct RTPMessage *p, vpx_codec_err_t rc,
                      uint32_t full_data_len,
                      uint8_t *ret_value);

uint32_t encode_frame_vpx(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                          const uint8_t *y,
                          const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                          uint64_t *video_frame_record_timestamp,
                          int vpx_encode_flags,
                          x264_nal_t **nal,
                          int *i_frame_size);

uint32_t send_frames_vpx(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                         const uint8_t *y,
                         const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                         uint64_t *video_frame_record_timestamp,
                         int vpx_encode_flags,
                         x264_nal_t **nal,
                         int *i_frame_size,
                         TOXAV_ERR_SEND_FRAME *rc);

void vc_kill_vpx(VCSession *vc);

struct encoder_t;
struct encoder_callbacks_t {
    
};



// ----------- GENERIC  -----------
typedef VCSession *vcs_new_t(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data,
                       VCSession *vc);

typedef int vc_reconfigure_encoder_h(Logger *log, VCSession *vc, uint32_t bit_rate,
                                uint16_t width, uint16_t height,
                                int16_t kf_max_dist);


typedef int vc_reconfigure_decoder_h(Logger *log, VCSession *vc, uint32_t bit_rate,
                                uint16_t width, uint16_t height,
                                int16_t kf_max_dist, void* spspps);

typedef void decode_frame_h(VCSession *vc, Messenger *m, uint8_t skip_video_flag, uint64_t *a_r_timestamp,
                       uint64_t *a_l_timestamp,
                       uint64_t *v_r_timestamp, uint64_t *v_l_timestamp,
                       const struct RTPHeader *header_v3,
                       struct RTPMessage *p, vpx_codec_err_t rc,
                       uint32_t full_data_len,
                       uint8_t *ret_value);

// update_h
typedef uint32_t encode_frame_h(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                           const uint8_t *y,
                           const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                           uint64_t *video_frame_record_timestamp,
                           int vpx_encode_flags,
                           x264_nal_t **nal,
                           int *i_frame_size);

// rtp_send callback ?? 
uint32_t send_frames_h264(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                          const uint8_t *y,
                          const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                          uint64_t *video_frame_record_timestamp,
                          int vpx_encode_flags,
                          x264_nal_t **nal,
                          int *i_frame_size,
                          TOXAV_ERR_SEND_FRAME *rc);

// destructor
void vcs_destroy(VCSession *vc);



// ----------- H264 -----------
VCSession *vc_new_h264(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data,
                       VCSession *vc);

int vc_reconfigure_encoder_h264(Logger *log, VCSession *vc, uint32_t bit_rate,
                                uint16_t width, uint16_t height,
                                int16_t kf_max_dist);

void decode_frame_h264(VCSession *vc, Messenger *m, uint8_t skip_video_flag, uint64_t *a_r_timestamp,
                       uint64_t *a_l_timestamp,
                       uint64_t *v_r_timestamp, uint64_t *v_l_timestamp,
                       const struct RTPHeader *header_v3,
                       struct RTPMessage *p, vpx_codec_err_t rc,
                       uint32_t full_data_len,
                       uint8_t *ret_value);

uint32_t encode_frame_h264(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                           const uint8_t *y,
                           const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                           uint64_t *video_frame_record_timestamp,
                           int vpx_encode_flags,
                           x264_nal_t **nal,
                           int *i_frame_size);

uint32_t send_frames_h264(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                          const uint8_t *y,
                          const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                          uint64_t *video_frame_record_timestamp,
                          int vpx_encode_flags,
                          x264_nal_t **nal,
                          int *i_frame_size,
                          TOXAV_ERR_SEND_FRAME *rc);

void vc_kill_h264(VCSession *vc);



// ----------- H264 OMX RaspberryPi -----------
VCSession *vc_new_h264_omx_raspi(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb,
                                 void *cb_data,
                                 VCSession *vc);

int vc_reconfigure_encoder_h264_omx_raspi(Logger *log, VCSession *vc, uint32_t bit_rate,
        uint16_t width, uint16_t height,
        int16_t kf_max_dist);

void decode_frame_h264_omx_raspi(VCSession *vc, Messenger *m, uint8_t skip_video_flag, uint64_t *a_r_timestamp,
                                 uint64_t *a_l_timestamp,
                                 uint64_t *v_r_timestamp, uint64_t *v_l_timestamp,
                                 const struct RTPHeader *header_v3,
                                 struct RTPMessage *p, vpx_codec_err_t rc,
                                 uint32_t full_data_len,
                                 uint8_t *ret_value);

uint32_t encode_frame_h264_omx_raspi(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                                     const uint8_t *y,
                                     const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                                     uint64_t *video_frame_record_timestamp,
                                     int vpx_encode_flags,
                                     x264_nal_t **nal,
                                     int *i_frame_size);

uint32_t send_frames_h264_omx_raspi(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                                    const uint8_t *y,
                                    const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                                    uint64_t *video_frame_record_timestamp,
                                    int vpx_encode_flags,
                                    x264_nal_t **nal,
                                    int *i_frame_size,
                                    TOXAV_ERR_SEND_FRAME *rc);

void h264_omx_raspi_force_i_frame(Logger *log, VCSession *vc);

void vc_kill_h264_omx_raspi(VCSession *vc);








