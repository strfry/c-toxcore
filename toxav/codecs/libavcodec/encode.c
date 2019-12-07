

VCSession *vc_new_libavcodec_encoder(Logger *log, ToxAV *av, uint32_t friend_number, toxav_video_receive_frame_cb *cb, void *cb_data,
                       VCSession *vc)
{
// ------ ffmpeg encoder ------
    AVCodec *codec2 = NULL;
    vc->libavcodec_encoder = NULL;
	

    // avcodec_register_all();

    codec2 = NULL;

#ifdef RAPI_HWACCEL_ENC
    codec2 = avcodec_find_encoder_by_name(H264_WANT_ENCODER_NAME);

    if (!codec2) {
        LOGGER_WARNING(log, "codec not found HW Accel H264 on encoder, trying software decoder ...");
        codec2 = avcodec_find_encoder_by_name("libx264");
    } else {
        LOGGER_ERROR(log, "FOUND: *HW Accel* H264 encoder: %s", H264_WANT_ENCODER_NAME);
    }

#else
    codec2 = avcodec_find_encoder_by_name("libx264");
#endif

    vc->libavcodec_encoder = avcodec_alloc_context3(codec2);

    vc->x264_encoder->out_pic2 = av_packet_alloc();

    // -- set inital value for H264 encoder profile --
    global_h264_enc_profile_high_enabled = H264_ENCODER_STARTWITH_PROFILE_HIGH;
    // -- set inital value for H264 encoder profile --

    if (global_h264_enc_profile_high_enabled == 1) {
        av_opt_set(vc->libavcodec_encoder->priv_data, "profile", "high", 0);
        vc->libavcodec_encoder->profile               = FF_PROFILE_H264_HIGH; // FF_PROFILE_H264_HIGH;
        // av_opt_set(vc->libavcodec_encoder->priv_data, "level", "4.0", AV_OPT_SEARCH_CHILDREN);
        // vc->libavcodec_encoder->level = 40; // 4.0
    } else {
        av_opt_set(vc->libavcodec_encoder->priv_data, "profile", "baseline", 0);
        vc->libavcodec_encoder->profile               = FF_PROFILE_H264_BASELINE; // FF_PROFILE_H264_HIGH;
        // av_opt_set(vc->libavcodec_encoder->priv_data, "level", "4.0", AV_OPT_SEARCH_CHILDREN);
        // vc->libavcodec_encoder->level = 40; // 4.0
    }

#if 0
    //print
    char *val_str = (char *)av_malloc(20000);
    av_opt_get(vc->libavcodec_encoder->priv_data, "preset", 0, (uint8_t **)&val_str);
    LOGGER_ERROR(log, "preset val: %s", val_str);
    av_opt_get(vc->libavcodec_encoder->priv_data, "tune", 0, (uint8_t **)&val_str);
    LOGGER_ERROR(log, "tune val: %s", val_str);
    av_opt_get(vc->libavcodec_encoder->priv_data, "profile", 0, (uint8_t **)&val_str);
    LOGGER_ERROR(log, "profile val: %s", val_str);
    av_opt_get(vc->libavcodec_encoder->priv_data, "level", 0, (uint8_t **)&val_str);
    LOGGER_ERROR(log, "level val: %s", val_str);
    av_free(val_str);
#endif


    av_opt_set(vc->libavcodec_encoder->priv_data, "annex_b", "1", 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "repeat_headers", "1", 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "tune", "zerolatency", 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "b", "100000", 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "bitrate", "100000", 0);
    // av_opt_set_int(vc->libavcodec_encoder->priv_data, "minrate", 100000, 0);
    // av_opt_set_int(vc->libavcodec_encoder->priv_data, "maxrate", (int)((float)100000 * H264_ENCODE_MAX_BITRATE_OVER_ALLOW), 0);

    av_opt_set_int(vc->libavcodec_encoder->priv_data, "cbr", true, 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "rc", "cbr", 0);
    av_opt_set_int(vc->libavcodec_encoder->priv_data, "delay", 0, 0);
    // av_opt_set_int(vc->libavcodec_encoder->priv_data, "rc-lookahead", 0, 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "preset", "llhp", 0);
    av_opt_set_int(vc->libavcodec_encoder->priv_data, "bf", 0, 0);
    av_opt_set_int(vc->libavcodec_encoder->priv_data, "qmin", 3, 0);
    av_opt_set_int(vc->libavcodec_encoder->priv_data, "qmax", 51, 0);
    av_opt_set(vc->libavcodec_encoder->priv_data, "forced-idr", "true", 0);
    av_opt_set_int(vc->libavcodec_encoder->priv_data, "zerolatency", 1, AV_OPT_SEARCH_CHILDREN);
    //y// av_opt_set_int(vc->libavcodec_encoder->priv_data, "refs", 0, 0);

    av_opt_set_int(vc->libavcodec_encoder->priv_data, "threads", X264_ENCODER_THREADS, 0);

    if (X264_ENCODER_SLICES > 0) {
        av_opt_set(vc->libavcodec_encoder->priv_data, "sliced_threads", "1", 0);
    } else {
        av_opt_set(vc->libavcodec_encoder->priv_data, "sliced_threads", "0", 0);
    }

    av_opt_set_int(vc->libavcodec_encoder->priv_data, "slice_count", X264_ENCODER_SLICES, 0);

    /* put sample parameters */
    vc->libavcodec_encoder->bit_rate = 100 * 1000;
    vc->h264_enc_bitrate = 100 * 1000;

    /* resolution must be a multiple of two */
    vc->libavcodec_encoder->width = 1920;
    vc->libavcodec_encoder->height = 1080;

    vc->h264_enc_width = vc->libavcodec_encoder->width;
    vc->h264_enc_height = vc->libavcodec_encoder->height;

    vc->libavcodec_encoder->gop_size = VIDEO_MAX_KF_H264;
    // vc->libavcodec_encoder->keyint_min = VIDEO_MAX_KF_H264;
    vc->libavcodec_encoder->max_b_frames = 0;
    vc->libavcodec_encoder->pix_fmt = AV_PIX_FMT_YUV420P;

    vc->libavcodec_encoder->time_base.num = 1;
    vc->libavcodec_encoder->time_base.den = 1000;

    // without these it won't work !! ---------------------
    vc->libavcodec_encoder->time_base = (AVRational) {
        40, 1000
    };
    vc->libavcodec_encoder->framerate = (AVRational) {
        1000, 40
    };
    // without these it won't work !! ---------------------

    // vc->libavcodec_encoder->flags2 |= AV_CODEC_FLAG2_LOCAL_HEADER;
    // vc->libavcodec_encoder->flags2 |= AV_CODEC_FLAG2_FAST;

    AVDictionary *opts = NULL;

    if (avcodec_open2(vc->libavcodec_encoder, codec2, &opts) < 0) {
        LOGGER_ERROR(log, "could not open codec H264 on encoder");
    }

    av_dict_free(&opts);

    return vc;
}


int vc_reconfigure_libavcodec_encoder(Logger *log, VCSession *vc, uint32_t bit_rate,
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

		vc->libavcodec_encoder->bit_rate = bit_rate;
		vc->h264_enc_bitrate = bit_rate;

		av_opt_set_int(vc->libavcodec_encoder->priv_data, "b", bit_rate, 0);
		av_opt_set_int(vc->libavcodec_encoder->priv_data, "bitrate", bit_rate, 0);
		// av_opt_set_int(vc->libavcodec_encoder->priv_data, "minrate", bit_rate, 0);
		// av_opt_set_int(vc->libavcodec_encoder->priv_data, "maxrate", (int)((float)bit_rate * H264_ENCODE_MAX_BITRATE_OVER_ALLOW), 0);
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

            // --- ffmpeg encoder ---
            avcodec_free_context(&vc->libavcodec_encoder);


            AVCodec *codec2 = NULL;
            vc->libavcodec_encoder = NULL;

            // avcodec_register_all();

            codec2 = NULL;

#ifdef RAPI_HWACCEL_ENC
            codec2 = avcodec_find_encoder_by_name(H264_WANT_ENCODER_NAME);

            if (!codec2) {
                LOGGER_WARNING(log, "codec not found HW Accel H264 on encoder, trying software decoder ...");
                codec2 = avcodec_find_encoder_by_name("libx264");
            } else {
                LOGGER_ERROR(log, "FOUND: *HW Accel* H264 encoder: %s", H264_WANT_ENCODER_NAME);
            }

#else
            codec2 = avcodec_find_encoder_by_name("libx264");
#endif

            vc->libavcodec_encoder = avcodec_alloc_context3(codec2);

            if (global_h264_enc_profile_high_enabled == 1) {
                av_opt_set(vc->libavcodec_encoder->priv_data, "profile", "high", 0);
                vc->libavcodec_encoder->profile               = FF_PROFILE_H264_HIGH; // FF_PROFILE_H264_HIGH;
                // av_opt_set(vc->libavcodec_encoder->priv_data, "level", "4.0", AV_OPT_SEARCH_CHILDREN);
                // vc->libavcodec_encoder->level = 40; // 4.0
                LOGGER_WARNING(log, "switching H264 encoder profile ... HIGH");

            } else {
                av_opt_set(vc->libavcodec_encoder->priv_data, "profile", "baseline", 0);
                vc->libavcodec_encoder->profile               = FF_PROFILE_H264_BASELINE; // FF_PROFILE_H264_HIGH;
                // av_opt_set(vc->libavcodec_encoder->priv_data, "level", "4.0", AV_OPT_SEARCH_CHILDREN);
                // vc->libavcodec_encoder->level = 40; // 4.0
                LOGGER_WARNING(log, "switching H264 encoder profile ... baseline");
            }

            av_opt_set(vc->libavcodec_encoder->priv_data, "annex_b", "1", 0);
            av_opt_set(vc->libavcodec_encoder->priv_data, "repeat_headers", "1", 0);
            av_opt_set(vc->libavcodec_encoder->priv_data, "tune", "zerolatency", 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "b", bit_rate, 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "bitrate", bit_rate, 0);
            // av_opt_set_int(vc->libavcodec_encoder->priv_data, "minrate", bit_rate, 0);
            // av_opt_set_int(vc->libavcodec_encoder->priv_data, "maxrate", (int)((float)bit_rate * H264_ENCODE_MAX_BITRATE_OVER_ALLOW), 0);

            av_opt_set_int(vc->libavcodec_encoder->priv_data, "cbr", true, 0);
            av_opt_set(vc->libavcodec_encoder->priv_data, "rc", "cbr", 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "delay", 0, 0);
            // av_opt_set_int(vc->libavcodec_encoder->priv_data, "rc-lookahead", 0, 0);
            av_opt_set(vc->libavcodec_encoder->priv_data, "preset", "llhp", 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "bf", 0, 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "qmin", 3, 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "qmax", 51, 0);
            av_opt_set(vc->libavcodec_encoder->priv_data, "forced-idr", "true", 0);
            av_opt_set_int(vc->libavcodec_encoder->priv_data, "zerolatency", 1, AV_OPT_SEARCH_CHILDREN);
            //y// av_opt_set_int(vc->libavcodec_encoder->priv_data, "refs", 1, 0);

            av_opt_set_int(vc->libavcodec_encoder->priv_data, "threads", X264_ENCODER_THREADS, 0);

            if (X264_ENCODER_SLICES > 0) {
                av_opt_set(vc->libavcodec_encoder->priv_data, "sliced_threads", "1", 0);
            } else {
                av_opt_set(vc->libavcodec_encoder->priv_data, "sliced_threads", "0", 0);
            }

            av_opt_set_int(vc->libavcodec_encoder->priv_data, "slice_count", X264_ENCODER_SLICES, 0);

            /* put sample parameters */
            vc->libavcodec_encoder->bit_rate = bit_rate;
            vc->h264_enc_bitrate = bit_rate;

            /* resolution must be a multiple of two */
            vc->libavcodec_encoder->width = width;
            vc->libavcodec_encoder->height = height;

            vc->h264_enc_width = vc->libavcodec_encoder->width;
            vc->h264_enc_height = vc->libavcodec_encoder->height;

            vc->libavcodec_encoder->gop_size = VIDEO_MAX_KF_H264;
            // vc->libavcodec_encoder->keyint_min = VIDEO_MAX_KF_H264;
            vc->libavcodec_encoder->max_b_frames = 0;
            vc->libavcodec_encoder->pix_fmt = AV_PIX_FMT_YUV420P;

            vc->libavcodec_encoder->time_base.num = 1;
            vc->libavcodec_encoder->time_base.den = 1000;

            // without these it won't work !! ---------------------
            vc->libavcodec_encoder->time_base = (AVRational) {
                40, 1000
            };
            vc->libavcodec_encoder->framerate = (AVRational) {
                1000, 40
            };
            // without these it won't work !! ---------------------

            /*
            av_opt_get(pCodecCtx->priv_data,"preset",0,(uint8_t **)&val_str);
                    printf("preset val: %s\n",val_str);
                    av_opt_get(pCodecCtx->priv_data,"tune",0,(uint8_t **)&val_str);
                    printf("tune val: %s\n",val_str);
                    av_opt_get(pCodecCtx->priv_data,"profile",0,(uint8_t **)&val_str);
                    printf("profile val: %s\n",val_str);
            av_free(val_str);
            */

            // vc->libavcodec_encoder->flags2 |= AV_CODEC_FLAG2_LOCAL_HEADER;
            // vc->libavcodec_encoder->flags2 |= AV_CODEC_FLAG2_FAST;

            AVDictionary *opts = NULL;

            if (avcodec_open2(vc->libavcodec_encoder, codec2, NULL) < 0) {
                LOGGER_ERROR(log, "could not open codec H264 on encoder");
            }

            av_dict_free(&opts);

            // --- ffmpeg encoder ---
        }
    }

    return 0;
}


uint32_t encode_frame_libavcodec_h264(ToxAV *av, uint32_t friend_number, uint16_t width, uint16_t height,
                           const uint8_t *y,
                           const uint8_t *u, const uint8_t *v, ToxAVCall *call,
                           uint64_t *video_frame_record_timestamp,
                           int vpx_encode_flags,
                           x264_nal_t **nal,
                           int *i_frame_size)
{
    AVFrame *frame;
    int ret;
    uint32_t result = 1;

    frame = av_frame_alloc();

    frame->format = call->video->libavcodec_encoder->pix_fmt;
    frame->width  = width;
    frame->height = height;

    ret = av_frame_get_buffer(frame, 32);

    if (ret < 0) {
        LOGGER_ERROR(av->m->log, "av_frame_get_buffer:Could not allocate the video frame data");
    }

    /* make sure the frame data is writable */
    ret = av_frame_make_writable(frame);

    if (ret < 0) {
        LOGGER_ERROR(av->m->log, "av_frame_make_writable:ERROR");
    }

    frame->pts = (int64_t)(*video_frame_record_timestamp);


    // copy YUV frame data into buffers
    memcpy(frame->data[0], y, width * height);
    memcpy(frame->data[1], u, (width / 2) * (height / 2));
    memcpy(frame->data[2], v, (width / 2) * (height / 2));

    // encode the frame
    ret = avcodec_send_frame(call->video->libavcodec_encoder, frame);

    if (ret < 0) {
        LOGGER_ERROR(av->m->log, "Error sending a frame for encoding:ERROR");
    }

    ret = avcodec_receive_packet(call->video->libavcodec_encoder, call->video->x264_encoder->out_pic2);

    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        *i_frame_size = 0;
    } else if (ret < 0) {
        *i_frame_size = 0;
        // fprintf(stderr, "Error during encoding\n");
    } else {

        // printf("Write packet %3"PRId64" (size=%5d)\n", call->video->x264_encoder->out_pic2->pts, call->video->x264_encoder->out_pic2->size);
        // fwrite(call->video->x264_encoder->out_pic2->data, 1, call->video->x264_encoder->out_pic2->size, outfile);

        global_encoder_delay_counter++;

        if (global_encoder_delay_counter > 60) {
            global_encoder_delay_counter = 0;
            LOGGER_DEBUG(av->m->log, "enc:delay=%ld",
                         (long int)(frame->pts - (int64_t)call->video->x264_encoder->out_pic2->pts)
                        );
        }


        *i_frame_size = call->video->x264_encoder->out_pic2->size;
        *video_frame_record_timestamp = (uint64_t)call->video->x264_encoder->out_pic2->pts;

        result = 0;
    }

    av_frame_free(&frame);

    return result;
}

void vc_kill_libavcodec_encode(VCSession *vc)
{
    av_packet_free(&(vc->x264_encoder->out_pic2));
    avcodec_free_context(&(vc->libavcodec_encoder));
}
