

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
