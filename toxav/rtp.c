/*
 * Copyright © 2016-2018 The TokTok team.
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif /* HAVE_CONFIG_H */

#include "rtp.h"

#include "bwcontroller.h"
#include "video.h"
#include "audio.h"
#include "dummy_ntp.h"

#include "../toxcore/Messenger.h"
#include "../toxcore/network.h"
#include "../toxcore/logger.h"
#include "../toxcore/util.h"
#include "../toxcore/mono_time.h"

#include <assert.h>
#include <errno.h>
#include <stdlib.h>


#define DISABLE_H264_ENCODER_FEATURE    0


int TOXAV_SEND_VIDEO_LOSSLESS_PACKETS = 0;


// allocate_len is NOT including header!
static struct RTPMessage *new_message(const struct RTPHeader *header, size_t allocate_len, const uint8_t *data,
                                      uint16_t data_length)
{
    assert(allocate_len >= data_length);
    struct RTPMessage *msg = (struct RTPMessage *)calloc(1, sizeof(struct RTPMessage) + allocate_len);

    if (msg == nullptr) {
        return nullptr;
    }

    msg->len = data_length; // result without header
    msg->header = *header;
    memcpy(msg->data, data, msg->len);
    return msg;
}

/**
 * Instruct the caller to clear slot 0.
 */
#define GET_SLOT_RESULT_DROP_OLDEST_SLOT (-1)

/**
 * Instruct the caller to drop the incoming packet.
 */
#define GET_SLOT_RESULT_DROP_INCOMING (-2)

/**
 * Find the next free slot in work_buffer for the incoming data packet.
 *
 * - If the data packet belongs to a frame that's already in the work_buffer then
 *   use that slot.
 * - If there is no free slot return GET_SLOT_RESULT_DROP_OLDEST_SLOT.
 * - If the data packet is too old return GET_SLOT_RESULT_DROP_INCOMING.
 *
 * If there is a keyframe being assembled in slot 0, keep it a bit longer and
 * do not kick it out right away if all slots are full instead kick out the new
 * incoming interframe.
 */
static int8_t get_slot(Logger *log, struct RTPWorkBufferList *wkbl, bool is_keyframe,
                       const struct RTPHeader *header, bool is_multipart)
{

    if (wkbl->next_free_entry == 0) {
        // the work buffer is completely empty
        // just return the first slot then
        LOGGER_DEBUG(log, "get_slot:work buffer empty");
        return 0;
    }

    if (is_multipart) {
        // This RTP message is part of a multipart frame, so we try to find an
        // existing slot with the previous parts of the frame in it.
        for (uint8_t i = 0; i < wkbl->next_free_entry; ++i) {
            const struct RTPWorkBuffer *slot = &wkbl->work_buffer[i];

            if ((slot->buf->header.sequnum == header->sequnum) && (slot->buf->header.timestamp == header->timestamp)) {
                // Sequence number and timestamp match, so this slot belongs to
                // the same frame.
                //
                // In reality, these will almost certainly either both match or
                // both not match. Only if somehow there were 65535 frames
                // between, the timestamp will matter.
                LOGGER_DEBUG(log, "get_slot:found slot num %d", (int)i);
                return i;
            }
        }
    }

    // The message may or may not be part of a multipart frame.
    //
    // If it is part of a multipart frame, then this is an entirely new frame
    // for which we did not have a slot *or* the frame is so old that its slot
    // has been evicted by now.
    //
    //        |----------- time ----------->
    //        _________________
    // slot 0 |               |
    //        -----------------
    //                     _________________
    // slot 1              |               |
    //                     -----------------
    //                ____________
    // slot 2         |          | -> frame too old, drop
    //                ------------
    //
    //
    //
    //        |----------- time ----------->
    //        _________________
    // slot 0 |               |
    //        -----------------
    //                     _________________
    // slot 1              |               |
    //                     -----------------
    //                              ____________
    // slot 2                       |          | -> ok, start filling in a new slot
    //                              ------------

    // If there is a free slot:
    if (wkbl->next_free_entry < USED_RTP_WORKBUFFER_COUNT) {
        // If there is at least one filled slot:
        if (wkbl->next_free_entry > 0) {
            // Get the most recently filled slot.
            const struct RTPWorkBuffer *slot = &wkbl->work_buffer[wkbl->next_free_entry - 1];
        }

        // Not all slots are filled, and the packet is newer than our most
        // recent slot, so it's a new frame we want to start assembling. This is
        // the second situation in the above diagram.
        LOGGER_DEBUG(log, "get_slot:slot=%d", (int)wkbl->next_free_entry);
        return wkbl->next_free_entry;
    }

    return GET_SLOT_RESULT_DROP_OLDEST_SLOT;
}

/**
 * Returns an assembled frame (as much data as we currently have for this frame,
 * some pieces may be missing)
 *
 * If there are no frames ready, we return NULL. If this function returns
 * non-NULL, it transfers ownership of the message to the caller, i.e. the
 * caller is responsible for storing it elsewhere or calling free().
 */
static struct RTPMessage *process_frame(Logger *log, struct RTPWorkBufferList *wkbl, uint8_t slot_id)
{
    assert(wkbl->next_free_entry >= 0);

    LOGGER_DEBUG(log, "process_frame:slot_id=%d", (int)slot_id);

    if (wkbl->next_free_entry == 0) {
        // There are no frames in any slot.
        LOGGER_DEBUG(log, "process_frame:workbuffer empty");
        return nullptr;
    }

    // Either slot_id is 0 and slot 0 is a key frame, or there is no key frame
    // in slot 0 (and slot_id is anything).
    struct RTPWorkBuffer *const slot = &wkbl->work_buffer[slot_id];

    // Move ownership of the frame out of the slot into m_new.
    struct RTPMessage *const m_new = slot->buf;
    LOGGER_DEBUG(log, "process_frame:1:m_new=%p", (void *)m_new);

    if (m_new) {
        LOGGER_DEBUG(log, "process_frame:1:m_new seq#=%d", (int)m_new->header.sequnum);
    }

    slot->buf = nullptr;

    assert(wkbl->next_free_entry >= 1);

    if (slot_id != wkbl->next_free_entry - 1) {
        // The slot is not the last slot, so we created a gap. We move all the
        // entries after it one step up.
        LOGGER_DEBUG(log, "process_frame:We move all entries after it one step up");

        for (uint8_t i = slot_id; i < wkbl->next_free_entry - 1; ++i) {
            // Move entry (i+1) into entry (i).
            LOGGER_DEBUG(log, "process_frame:move %d -> %d", (int)(i + 1), (int)i);
            wkbl->work_buffer[i] = wkbl->work_buffer[i + 1];
        }
    }

    // We now have a free entry at the end of the array.
    LOGGER_DEBUG(log, "process_frame:cur next_free_entry=%d", (int)wkbl->next_free_entry);
    --wkbl->next_free_entry;
    LOGGER_DEBUG(log, "process_frame:new next_free_entry=%d", (int)wkbl->next_free_entry);

    // Clear the newly freed entry.
    const struct RTPWorkBuffer empty = {0};
    wkbl->work_buffer[wkbl->next_free_entry] = empty;

    if (m_new) {
        LOGGER_DEBUG(log, "process_frame:2:m_new seq#=%d", (int)m_new->header.sequnum);
    }

    // Move ownership of the frame to the caller.
    return m_new;
}

/**
 * @param log A logger.
 * @param wkbl The list of in-progress frames, i.e. all the slots.
 * @param slot_id The slot we want to fill the data into.
 * @param is_keyframe Whether the data is part of a key frame.
 * @param header The RTP header from the incoming packet.
 * @param incoming_data The pure payload without header.
 * @param incoming_data_length The length in bytes of the incoming data payload.
 */
static bool fill_data_into_slot(Logger *log, struct RTPWorkBufferList *wkbl, const uint8_t slot_id, bool is_keyframe,
                                const struct RTPHeader *header, const uint8_t *incoming_data, uint16_t incoming_data_length)
{
    // We're either filling the data into an existing slot, or in a new one that
    // is the next free entry.
    assert(slot_id <= wkbl->next_free_entry);
    struct RTPWorkBuffer *const slot = &wkbl->work_buffer[slot_id];

    assert(header != nullptr);
    // ** // assert(is_keyframe == (bool)(header->flags & RTP_KEY_FRAME));

    if (slot->received_len == 0) {
        assert(slot->buf == nullptr);

        // No data for this slot has been received, yet, so we create a new
        // message for it with enough memory for the entire frame.
        struct RTPMessage *msg = (struct RTPMessage *)calloc(1, sizeof(struct RTPMessage) + header->data_length_full);

        if (msg == nullptr) {
            LOGGER_DEBUG(log, "Out of memory while trying to allocate for frame of size %u\n",
                         (unsigned)header->data_length_full);
            // Out of memory: throw away the incoming data.
            return false;
        }

        // Unused in the new video receiving code, as it's 16 bit and can't hold
        // the full length of large frames. Instead, we use slot->received_len.
        msg->len = 0;
        msg->header = *header;

        slot->buf = msg;
        slot->is_keyframe = is_keyframe;
        slot->received_len = 0;

        assert(wkbl->next_free_entry < USED_RTP_WORKBUFFER_COUNT);
        ++wkbl->next_free_entry;
    }

    // We already checked this when we received the packet, but we rely on it
    // here, so assert again.
    if (header->offset_full >= header->data_length_full) {
        LOGGER_DEBUG(log, "offset_full:%d < data_length_full:%d\n", (int)header->offset_full, (int)header->data_length_full);
    }

    // ***** // assert(header->offset_full < header->data_length_full);

    // Copy the incoming chunk of data into the correct position in the full
    // frame data array.
    memcpy(
        slot->buf->data + header->offset_full,
        incoming_data,
        incoming_data_length
    );

    // Update the total received length of this slot.
    slot->received_len += incoming_data_length;

    // Update received length also in the header of the message, for later use.
    slot->buf->header.received_length_full = slot->received_len;

    LOGGER_DEBUG(log, "FPATH:slot num=%d:VSEQ:%d %d/%d", slot_id, (int)header->sequnum,
                 (int)slot->received_len, (int)header->data_length_full);

    return slot->received_len == header->data_length_full;
}


/**
 * Handle a single RTP video packet.
 *
 * The packet may or may not be part of a multipart frame. This function will
 * find out and handle it appropriately.
 *
 * @param session The current RTP session with:
 *   session->mcb == vc_queue_message() // this function is called from here
 *   session->mp == struct RTPMessage *
 *   session->cs == call->video.second // == VCSession created by vc_new() call
 * @param header The RTP header deserialised from the packet.
 * @param incoming_data The packet data *not* header, i.e. this is the actual
 *   payload.
 * @param incoming_data_length The packet length *not* including header, i.e.
 *   this is the actual payload length.
 * @param log A logger.
 *
 * @return -1 on error, 0 on success.
 */
static int handle_video_packet(RTPSession *session, const struct RTPHeader *header,
                               const uint8_t *incoming_data, uint16_t incoming_data_length, Logger *log)
{
    // Full frame length in bytes. The frame may be split into multiple packets,
    // but this value is the complete assembled frame size.
    const uint32_t full_frame_length = header->data_length_full;

    // Current offset in the frame. If this is the first packet of a multipart
    // frame or it's not a multipart frame, then this value is 0.
    const uint32_t offset = header->offset_full; // without header

    LOGGER_DEBUG(log, "FPATH:%d", (int)header->sequnum);

    // sanity checks ---------------
    if (full_frame_length == 0) {
        LOGGER_DEBUG(log, "EE:1:VSEQ:%d", (int)header->sequnum);
        return -1;
    }

    if (offset == full_frame_length) {
        LOGGER_DEBUG(log, "EE:2:VSEQ:%d", (int)header->sequnum);
        return -1;
    }

    if (offset > full_frame_length) {
        LOGGER_DEBUG(log, "EE:3:VSEQ:%d", (int)header->sequnum);
        return -1;
    }

    LOGGER_DEBUG(log, "II:4:VSEQ:%d", (int)header->sequnum);

    LOGGER_DEBUG(log, "FPATH:%d", (int)header->sequnum);

    // sanity checks ---------------

    // The sender tells us whether this is a key frame.
    const bool is_keyframe = 0; // (header->flags & RTP_KEY_FRAME) != 0;

    LOGGER_DEBUG(log, "-- handle_video_packet -- full lens=%u len=%u offset=%u is_keyframe=%s",
                 (unsigned)incoming_data_length, (unsigned)full_frame_length, (unsigned)offset, is_keyframe ? "K" : ".");
    LOGGER_DEBUG(log, "wkbl->next_free_entry:003=%d", session->work_buffer_list->next_free_entry);

    const bool is_multipart = (full_frame_length != incoming_data_length);

    /* The message was sent in single part */
    int8_t slot_id = get_slot(log, session->work_buffer_list, is_keyframe, header, is_multipart);
    LOGGER_DEBUG(log, "II:5:slot num=%d:VSEQ:%d", slot_id, (int)header->sequnum);

    LOGGER_DEBUG(log, "FPATH:%d slot=%d", (int)header->sequnum, slot_id);


    // get_slot told us to drop the packet, so we ignore it.
    if (slot_id == GET_SLOT_RESULT_DROP_INCOMING) {
        LOGGER_DEBUG(log, "FPATH:%d slot=%d", (int)header->sequnum, slot_id);
        LOGGER_DEBUG(log, "EE:6:VSEQ:%d", (int)header->sequnum);
        return -1;
    }

    // get_slot said there is no free slot.
    if (slot_id == GET_SLOT_RESULT_DROP_OLDEST_SLOT) {
        LOGGER_DEBUG(log, "there was no free slot, so we process the oldest frame");
        LOGGER_DEBUG(log, "II:7:slot num=%d:VSEQ:%d", slot_id, (int)header->sequnum);

        LOGGER_DEBUG(log, "FPATH:%d slot=%d", (int)header->sequnum, slot_id);

        // We now own the frame.
        struct RTPMessage *m_new = process_frame(log, session->work_buffer_list, 0);

        // The process_frame function returns NULL if there is no slot 0, i.e.
        // the work buffer list is completely empty. It can't be empty, because
        // get_slot just told us it's full, so process_frame must return non-null.
        assert(m_new != NULL);

        LOGGER_DEBUG(log, "FPATH:slot=%d VSEQ:%d:m_new=%p", slot_id, (int)m_new->header.sequnum, (void *)m_new);


        // LOGGER_DEBUG(log, "-- handle_video_packet -- CALLBACK-001a b0=%d b1=%d", (int)m_new->data[0], (int)m_new->data[1]);
        //**// update_bwc_values(log, session, m_new);
        // Pass ownership of m_new to the callback.
        session->mcb(session->m->mono_time, session->cs, m_new);
        // Now we no longer own m_new.
        m_new = NULL;

        // Now we must have a free slot, so we either get that slot, i.e. >= 0,
        // or get told to drop the incoming packet if it's too old.
        slot_id = get_slot(log, session->work_buffer_list, is_keyframe, header, /* is_multipart */false);

        LOGGER_DEBUG(log, "FPATH:9.0:slot num=%d:VSEQ:%d", slot_id, (int)header->sequnum);

        if (slot_id == GET_SLOT_RESULT_DROP_INCOMING) {
            // The incoming frame is too old, so we drop it.
            LOGGER_DEBUG(log, "FPATH:9:slot num=%d:VSEQ:%d", slot_id, (int)header->sequnum);
            return -1;
        }
    }

    // We must have a valid slot here.
    assert(slot_id >= 0);

    LOGGER_DEBUG(log, "fill_data_into_slot.");

    // fill in this part into the slot buffer at the correct offset
    if (!fill_data_into_slot(
                log,
                session->work_buffer_list,
                slot_id,
                is_keyframe,
                header,
                incoming_data,
                incoming_data_length)) {

        LOGGER_DEBUG(log, "FPATH:10:slot num=%d:VSEQ:%d", slot_id, (int)header->sequnum);

        return -1;
    } else {
        LOGGER_DEBUG(log, "FPATH:10c:slot num=%d:VSEQ:%d *message_complete*",
                     slot_id, (int)header->sequnum);
    }

    if (slot_id > 0) {
        // check if there are old messages lingering in the buffer
        struct RTPWorkBufferList *wkbl = session->work_buffer_list;
        struct RTPWorkBuffer *const slot0 = &wkbl->work_buffer[0];
        struct RTPMessage *const m_new0 = slot0->buf;
        struct RTPWorkBuffer *const slot2 = &wkbl->work_buffer[slot_id];
        struct RTPMessage *const m_new2 = slot2->buf;

#define LINGER_OLD_FRAMES_COUNT 2

        if ((m_new0) && (m_new2)) {
            if ((m_new0->header.sequnum + 2) < m_new2->header.sequnum) {
                LOGGER_DEBUG(log, "kick out:m_new0 seq#=%d", (int)m_new0->header.sequnum);
                // change slot_id to "0" to process oldest frame in buffer instead of current one
                struct RTPMessage *m_new = process_frame(log, session->work_buffer_list, slot_id);

                if (m_new) {
                    LOGGER_DEBUG(log, "FPATH:11x:slot num=%d:VSEQ:%d", slot_id, (int)m_new->header.sequnum);
                    session->mcb(session->m->mono_time, session->cs, m_new);
                    m_new = NULL;
                }

                slot_id = 0;
            }
        }
    }

    struct RTPMessage *m_new = process_frame(log, session->work_buffer_list, slot_id);

    if (m_new) {

        LOGGER_DEBUG(log, "FPATH:11:slot num=%d:VSEQ:%d", slot_id, (int)m_new->header.sequnum);

        // LOGGER_DEBUG(log, "-- handle_video_packet -- CALLBACK-003a b0=%d b1=%d", (int)m_new->data[0], (int)m_new->data[1]);
        //**//update_bwc_values(log, session, m_new);
        session->mcb(session->m->mono_time, session->cs, m_new);

        m_new = NULL;
    }

    LOGGER_DEBUG(log, "FPATH:12:slot num=%d:VSEQ:%d", slot_id, (int)header->sequnum);

    return 0;
}

/**
 * @return -1 on error, 0 on success.
 */
static int handle_rtp_packet(Messenger *m, uint32_t friendnumber, const uint8_t *data, uint16_t length, void *object)
{
    RTPSession *session = (RTPSession *)object;

    // Get the packet type.
    uint8_t packet_type = data[0];

    if (data[0] == PACKET_TOXAV_COMM_CHANNEL) {
        // LOGGER_ERROR(m->log, "RECVD:PACKET_TOXAV_COMM_CHANNEL:length=%d", (int)length);

        if (length >= 2) {
            if (data[1] == PACKET_TOXAV_COMM_CHANNEL_REQUEST_KEYFRAME) {
                if (session->cs) {
                    ((VCSession *)(session->cs))->send_keyframe_request_received = 1;
                }
            } else if (data[1] == PACKET_TOXAV_COMM_CHANNEL_HAVE_H264_VIDEO) {
                LOGGER_ERROR(m->log, "RECVD:PACKET_TOXAV_COMM_CHANNEL_HAVE_H264_VIDEO");

                if (session->cs) {
                    if (DISABLE_H264_ENCODER_FEATURE == 0) {
                        ((VCSession *)(session->cs))->h264_video_capabilities_received = 1;
                    }
                }
            } else if (data[1] == PACKET_TOXAV_COMM_CHANNEL_LESS_VIDEO_FPS) {
                if (session->cs) {
                    if ((data[2] > 1) && (data[2] < 10)) {
                        ((VCSession *)(session->cs))->skip_fps = data[2];
                        LOGGER_DEBUG(m->log, "RECVD:PACKET_TOXAV_COMM_CHANNEL_LESS_VIDEO_FPS skip=%d",
                                     (int)(((VCSession *)(session->cs))->skip_fps));
                    }

                    ((VCSession *)(session->cs))->skip_fps_duration_until_ts = current_time_monotonic(m->mono_time) +
                            TOXAV_SKIP_FPS_RELEASE_AFTER_MS;
                }
            } else if (data[1] == PACKET_TOXAV_COMM_CHANNEL_DUMMY_NTP_REQUEST) {

                uint32_t pkg_buf_len = (sizeof(uint32_t) * 3) + 2;
                uint8_t pkg_buf[pkg_buf_len];
                pkg_buf[0] = PACKET_TOXAV_COMM_CHANNEL;
                pkg_buf[1] = PACKET_TOXAV_COMM_CHANNEL_DUMMY_NTP_ANSWER;
                uint32_t tmp = current_time_monotonic(m->mono_time);
                pkg_buf[2] = data[2];
                pkg_buf[3] = data[3];
                pkg_buf[4] = data[4];
                pkg_buf[5] = data[5];
                //
                pkg_buf[6] = tmp >> 24 & 0xFF;
                pkg_buf[7] = tmp >> 16 & 0xFF;
                pkg_buf[8] = tmp >> 8  & 0xFF;
                pkg_buf[9] = tmp       & 0xFF;
                tmp = tmp + 1; // add 1 ms delay between receiving answer and sending response
                pkg_buf[10] = tmp >> 24 & 0xFF;
                pkg_buf[11] = tmp >> 16 & 0xFF;
                pkg_buf[12] = tmp >> 8  & 0xFF;
                pkg_buf[13] = tmp       & 0xFF;

                LOGGER_DEBUG(m->log, "RECVD:PACKET_TOXAV_COMM_CHANNEL_DUMMY_NTP_REQUEST: %d %d %d %d",
                             pkg_buf[6], pkg_buf[7], pkg_buf[8], pkg_buf[9]);

                int result = send_custom_lossless_packet(m, friendnumber, pkg_buf, pkg_buf_len);


            } else if (data[1] == PACKET_TOXAV_COMM_CHANNEL_DUMMY_NTP_ANSWER) {

                LOGGER_DEBUG(m->log, "RECVD:PACKET_TOXAV_COMM_CHANNEL_DUMMY_NTP_ANSWER: %d %d %d %d",
                             data[6], data[7], data[8], data[9]);


                ((VCSession *)(session->cs))->dummy_ntp_local_start =
                    ((uint32_t)(data[2]) << 24)
                    +
                    ((uint32_t)(data[3]) << 16)
                    +
                    ((uint32_t)(data[4]) << 8)
                    +
                    (data[5]);

                ((VCSession *)(session->cs))->dummy_ntp_remote_start =
                    ((uint32_t)data[6] << 24)
                    +
                    ((uint32_t)data[7] << 16)
                    +
                    ((uint32_t)data[8] << 8)
                    +
                    (data[9]);

                ((VCSession *)(session->cs))->dummy_ntp_remote_end =
                    ((uint32_t)data[10] << 24)
                    +
                    ((uint32_t)data[11] << 16)
                    +
                    ((uint32_t)data[12] << 8)
                    +
                    (data[13]);

                ((VCSession *)(session->cs))->dummy_ntp_local_end = current_time_monotonic(m->mono_time);

                LOGGER_DEBUG(m->log, "DNTP:%d %d %d %d",
                             ((VCSession *)(session->cs))->dummy_ntp_local_start,
                             ((VCSession *)(session->cs))->dummy_ntp_remote_start,
                             ((VCSession *)(session->cs))->dummy_ntp_remote_end,
                             ((VCSession *)(session->cs))->dummy_ntp_local_end);

                int64_t offset_ = dntp_calc_offset(((VCSession *)(session->cs))->dummy_ntp_remote_start,
                                                   ((VCSession *)(session->cs))->dummy_ntp_remote_end,
                                                   ((VCSession *)(session->cs))->dummy_ntp_local_start,
                                                   ((VCSession *)(session->cs))->dummy_ntp_local_end);

                uint32_t roundtrip_ = dntp_calc_roundtrip_delay(((VCSession *)(session->cs))->dummy_ntp_remote_start,
                                      ((VCSession *)(session->cs))->dummy_ntp_remote_end,
                                      ((VCSession *)(session->cs))->dummy_ntp_local_start,
                                      ((VCSession *)(session->cs))->dummy_ntp_local_end);

#define NETWORK_ROUND_TRIP_FUZZ_THRESHOLD_MS 150

                if (roundtrip_ > ((VCSession *)(session->cs))->rountrip_time_ms) {
                    if (roundtrip_ > ((((VCSession *)(session->cs))->rountrip_time_ms) + NETWORK_ROUND_TRIP_FUZZ_THRESHOLD_MS)) {
                        ((VCSession *)(session->cs))->rountrip_time_ms = ((VCSession *)(session->cs))->rountrip_time_ms +
                                (NETWORK_ROUND_TRIP_FUZZ_THRESHOLD_MS / 2);
                    } else {
                        ((VCSession *)(session->cs))->rountrip_time_ms = roundtrip_;
                    }
                } else if (roundtrip_ < ((VCSession *)(session->cs))->rountrip_time_ms) {
                    if ((roundtrip_ + NETWORK_ROUND_TRIP_FUZZ_THRESHOLD_MS) < ((VCSession *)(session->cs))->rountrip_time_ms) {
                        ((VCSession *)(session->cs))->rountrip_time_ms = ((VCSession *)(session->cs))->rountrip_time_ms -
                                (NETWORK_ROUND_TRIP_FUZZ_THRESHOLD_MS / 2);
                    } else {
                        ((VCSession *)(session->cs))->rountrip_time_ms = roundtrip_;
                    }
                }

                LOGGER_DEBUG(m->log, "DNTP:offset=%ld roundtrip=%u", (long)offset_, roundtrip_);
                // LOGGER_WARNING(m->log, "DNTP:A:offset new=%lld", ((VCSession *)(session->cs))->timestamp_difference_to_sender);

                int64_t *ptmp = &(((VCSession *)(session->cs))->timestamp_difference_to_sender);

                bool res4 = dntp_drift(ptmp, offset_, (int64_t)800);
                LOGGER_DEBUG(m->log, "DNTP:*B*:offset new=%lu",
                             (unsigned long)((VCSession *)(session->cs))->timestamp_difference_to_sender);
            }
        }

        return 0;
    }

    if (!session || length < RTP_HEADER_SIZE + 1) {
        LOGGER_WARNING(m->log, "No session or invalid length of received buffer!");
        return -1;
    }

    if (data[0] == PACKET_LOSSLESS_VIDEO) {
        packet_type = RTP_TYPE_VIDEO;
    }

    ++data;
    --length;

    // Unpack the header.
    struct RTPHeader header;
    rtp_header_unpack(data, &header);

    if (header.pt != packet_type % 128) {
        LOGGER_WARNING(m->log, "RTPHeader packet type and Tox protocol packet type did not agree: %d != %d",
                       header.pt, packet_type % 128);
        return -1;
    }

    if (header.pt != session->payload_type % 128) {
        LOGGER_WARNING(m->log, "RTPHeader packet type does not match this session's payload type: %d != %d",
                       header.pt, session->payload_type % 128);
        return -1;
    }

    if (header.offset_full >= header.data_length_full
            && (header.offset_full != 0 || header.data_length_full != 0)) {
        LOGGER_ERROR(m->log, "Invalid video packet: frame offset (%u) >= full frame length (%u)",
                     (unsigned)header.offset_full, (unsigned)header.data_length_full);
        return -1;
    }

    if (!(header.flags & RTP_LARGE_FRAME)) {
        if (header.offset_lower >= header.data_length_lower) {
            LOGGER_ERROR(m->log, "Invalid old protocol video packet: frame offset (%u) >= full frame length (%u)",
                         (unsigned)header.offset_lower, (unsigned)header.data_length_lower);
            return -1;
        }
    }

    LOGGER_DEBUG(m->log, "header.pt %d, video %d", (uint8_t)header.pt, (RTP_TYPE_VIDEO % 128));

    LOGGER_DEBUG(m->log, "rtp packet record time: %lu", (unsigned long)header.frame_record_timestamp);


    // check flag indicating that we have real record-timestamps for frames ---
    if ((header.flags & RTP_ENCODER_HAS_RECORD_TIMESTAMP) == 0) {
        if (header.pt == (RTP_TYPE_VIDEO % 128)) {
            LOGGER_DEBUG(m->log, "RTP_ENCODER_HAS_RECORD_TIMESTAMP:VV");
            ((VCSession *)(session->cs))->encoder_frame_has_record_timestamp = 0;
        } else if (header.pt == (RTP_TYPE_AUDIO % 128)) {
            LOGGER_DEBUG(m->log, "RTP_ENCODER_HAS_RECORD_TIMESTAMP:AA");
            ((ACSession *)(session->cs))->encoder_frame_has_record_timestamp = 0;
        }
    }

    if (header.pt == (RTP_TYPE_VIDEO % 128)) {
        ((VCSession *)(session->cs))->remote_client_video_capture_delay_ms = header.client_video_capture_delay_ms;
    }

    // set flag indicating that we have real record-timestamps for frames ---

    // HINT: ask sender for dummy ntp values -------------
    if (
        (
            ((header.sequnum % 100) == 0)
            ||
            (header.sequnum < 30)
        )
        && (header.offset_lower == 0)) {
        uint32_t pkg_buf_len = (sizeof(uint32_t) * 3) + 2;
        uint8_t pkg_buf[pkg_buf_len];
        pkg_buf[0] = PACKET_TOXAV_COMM_CHANNEL;
        pkg_buf[1] = PACKET_TOXAV_COMM_CHANNEL_DUMMY_NTP_REQUEST;
        uint32_t tmp = current_time_monotonic(m->mono_time);
        pkg_buf[2] = tmp >> 24 & 0xFF;
        pkg_buf[3] = tmp >> 16 & 0xFF;
        pkg_buf[4] = tmp >> 8  & 0xFF;
        pkg_buf[5] = tmp       & 0xFF;

        int result = send_custom_lossless_packet(m, friendnumber, pkg_buf, pkg_buf_len);
    }

    // HINT: ask sender for dummy ntp values -------------







    // The sender uses the new large-frame capable protocol and is sending a
    // video packet.
    if ((header.flags & RTP_LARGE_FRAME) && (header.pt == (RTP_TYPE_VIDEO % 128))) {

        if (session->incoming_packets_ts_last_ts == -1) {
            session->incoming_packets_ts[session->incoming_packets_ts_index] = 0;
            session->incoming_packets_ts_average = 0;
        } else {
            session->incoming_packets_ts[session->incoming_packets_ts_index] = current_time_monotonic(m->mono_time) -
                    session->incoming_packets_ts_last_ts;
        }

        session->incoming_packets_ts_last_ts = current_time_monotonic(m->mono_time);
        session->incoming_packets_ts_index++;

        if (session->incoming_packets_ts_index >= INCOMING_PACKETS_TS_ENTRIES) {
            session->incoming_packets_ts_index = 0;
        }

        uint32_t incoming_rtp_packets_delta_average = 0;

        for (int ii = 0; ii < INCOMING_PACKETS_TS_ENTRIES; ii++) {
            incoming_rtp_packets_delta_average = incoming_rtp_packets_delta_average + session->incoming_packets_ts[ii];
            // LOGGER_WARNING(m->log, "%d=%d", ii, (int)session->incoming_packets_ts[ii]);
        }

        incoming_rtp_packets_delta_average = incoming_rtp_packets_delta_average / INCOMING_PACKETS_TS_ENTRIES;

        if ((incoming_rtp_packets_delta_average > (2 * session->incoming_packets_ts_average))
                && (incoming_rtp_packets_delta_average > 15)) {
            // LOGGER_WARNING(m->log, "rtp_video_delta=%d", (int)incoming_rtp_packets_delta_average);
        }

        session->incoming_packets_ts_average = incoming_rtp_packets_delta_average;

        LOGGER_DEBUG(m->log, "rtp_video_delta=%d", (int)incoming_rtp_packets_delta_average);

        return handle_video_packet(session, &header, data + RTP_HEADER_SIZE, length - RTP_HEADER_SIZE, m->log);
    }

    // everything below here is for the old 16 bit protocol ------------------



    if (header.pt == (RTP_TYPE_AUDIO % 128)) {
        // HINT: we have an audio packet
        // LOGGER_ERROR(m->log, "AADEBUG:**** incoming audio packet ****");
    }


    /*
     * just process any incoming audio packets ----
     */
    if (header.data_length_lower == length - RTP_HEADER_SIZE) {
        /* The message is sent in single part */

        /*
         *   session->mcb == vc_queue_message() // this function is called from here
         *   session->mp == struct RTPMessage *
         *   session->cs == call->video.second // == VCSession created by vc_new() call
         */

        session->rsequnum = header.sequnum;
        session->rtimestamp = header.timestamp;

        LOGGER_DEBUG(m->log, "RTP: AUDIO singlepart message: len=%d seqnum=%d",
                     length, header.sequnum);

        return session->mcb(session->m->mono_time, session->cs, new_message(&header, length - RTP_HEADER_SIZE,
                            data + RTP_HEADER_SIZE, length - RTP_HEADER_SIZE));

    }

    /*
     * just process any incoming audio packets ----
     */


    LOGGER_DEBUG(m->log, "RTP: AUDIO multipart message");




    if (header.data_length_lower == length - RTP_HEADER_SIZE) {
        /* The message is sent in single part */

        /* Message is not late; pick up the latest parameters */
        session->rsequnum = header.sequnum;
        session->rtimestamp = header.timestamp;
        // bwc_add_recv(session->bwc, length);

        /* Invoke processing of active multiparted message */
        if (session->mp) {
            session->mcb(session->m->mono_time, session->cs, session->mp);
            session->mp = nullptr;
        }

        /* The message came in the allowed time;
         */

        return session->mcb(session->m->mono_time, session->cs, new_message(&header, length - RTP_HEADER_SIZE,
                            data + RTP_HEADER_SIZE, length - RTP_HEADER_SIZE));
    }

    /* The message is sent in multiple parts */

    if (session->mp) {
        /* There are 2 possible situations in this case:
         *      1) being that we got the part of already processing message.
         *      2) being that we got the part of a new/old message.
         *
         * We handle them differently as we only allow a single multiparted
         * processing message
         */
        if (session->mp->header.sequnum == header.sequnum &&
                session->mp->header.timestamp == header.timestamp) {
            /* First case */

            /* Make sure we have enough allocated memory */
            if (session->mp->header.data_length_lower - session->mp->len < length - RTP_HEADER_SIZE ||
                    session->mp->header.data_length_lower <= header.offset_lower) {
                /* There happened to be some corruption on the stream;
                 * continue wihtout this part
                 */
                return 0;
            }

            memcpy(session->mp->data + header.offset_lower, data + RTP_HEADER_SIZE,
                   length - RTP_HEADER_SIZE);
            session->mp->len += length - RTP_HEADER_SIZE;
            // bwc_add_recv(session->bwc, length);

            if (session->mp->len == session->mp->header.data_length_lower) {
                /* Received a full message; now push it for the further
                 * processing.
                 */
                session->mcb(session->m->mono_time, session->cs, session->mp);
                session->mp = nullptr;
            }
        } else {
            /* Second case */
            if (session->mp->header.timestamp > header.timestamp) {
                /* The received message part is from the old message;
                 * discard it.
                 */
                return 0;
            }

            /* Push the previous message for processing */
            session->mcb(session->m->mono_time, session->cs, session->mp);

            session->mp = nullptr;
            goto NEW_MULTIPARTED;
        }
    } else {
        /* In this case threat the message as if it was received in order
         */
        /* This is also a point for new multiparted messages */
NEW_MULTIPARTED:

        /* Message is not late; pick up the latest parameters */
        session->rsequnum = header.sequnum;
        session->rtimestamp = header.timestamp;
        // bwc_add_recv(session->bwc, length);

        /* Store message.
         */
        session->mp = new_message(&header, header.data_length_lower, data + RTP_HEADER_SIZE, length - RTP_HEADER_SIZE);
        memmove(session->mp->data + header.offset_lower, session->mp->data, session->mp->len);
    }

    return 0;
}

size_t rtp_header_pack(uint8_t *const rdata, const struct RTPHeader *header)
{
    uint8_t *p = rdata;
    *p = (header->ve & 3) << 6
         | (header->pe & 1) << 5
         | (header->xe & 1) << 4
         | (header->cc & 0xf);
    ++p;
    *p = (header->ma & 1) << 7
         | (header->pt & 0x7f);
    ++p;

    p += net_pack_u16(p, header->sequnum);
    p += net_pack_u32(p, header->timestamp);
    p += net_pack_u32(p, header->ssrc);
    p += net_pack_u64(p, header->flags);
    p += net_pack_u32(p, header->offset_full);
    p += net_pack_u32(p, header->data_length_full);
    p += net_pack_u32(p, header->received_length_full);

    // ---------------------------- //
    //      custom fields here      //
    // ---------------------------- //
    p += net_pack_u64(p, header->frame_record_timestamp);
    p += net_pack_u32(p, header->fragment_num);
    p += net_pack_u32(p, header->real_frame_num);
    p += net_pack_u32(p, header->encoder_bit_rate_used);
    p += net_pack_u32(p, header->client_video_capture_delay_ms);
    // ---------------------------- //
    //      custom fields here      //
    // ---------------------------- //

    for (size_t i = 0; i < RTP_PADDING_FIELDS; ++i) {
        p += net_pack_u32(p, 0);
    }

    p += net_pack_u16(p, header->offset_lower);
    p += net_pack_u16(p, header->data_length_lower);
    assert(p == rdata + RTP_HEADER_SIZE);
    return p - rdata;
}

size_t rtp_header_unpack(const uint8_t *data, struct RTPHeader *header)
{
    const uint8_t *p = data;
    header->ve = (*p >> 6) & 3;
    header->pe = (*p >> 5) & 1;
    header->xe = (*p >> 4) & 1;
    header->cc = *p & 0xf;
    ++p;

    header->ma = (*p >> 7) & 1;
    header->pt = *p & 0x7f;
    ++p;

    p += net_unpack_u16(p, &header->sequnum);
    p += net_unpack_u32(p, &header->timestamp);
    p += net_unpack_u32(p, &header->ssrc);
    p += net_unpack_u64(p, &header->flags);
    p += net_unpack_u32(p, &header->offset_full);
    p += net_unpack_u32(p, &header->data_length_full);
    p += net_unpack_u32(p, &header->received_length_full);

    // ---------------------------- //
    //      custom fields here      //
    // ---------------------------- //
    p += net_unpack_u64(p, &header->frame_record_timestamp);
    p += net_unpack_u32(p, (uint32_t *)&header->fragment_num);
    p += net_unpack_u32(p, &header->real_frame_num);
    p += net_unpack_u32(p, &header->encoder_bit_rate_used);
    p += net_unpack_u32(p, &header->client_video_capture_delay_ms);
    // ---------------------------- //
    //      custom fields here      //
    // ---------------------------- //

    p += sizeof(uint32_t) * RTP_PADDING_FIELDS;

    p += net_unpack_u16(p, &header->offset_lower);
    p += net_unpack_u16(p, &header->data_length_lower);
    assert(p == data + RTP_HEADER_SIZE);
    return p - data;
}

RTPSession *rtp_new(int payload_type, Messenger *m, uint32_t friendnumber,
                    BWController *bwc, void *cs, rtp_m_cb *mcb)
{
    assert(mcb != nullptr);
    assert(cs != nullptr);
    assert(m != nullptr);

    RTPSession *session = (RTPSession *)calloc(1, sizeof(RTPSession));

    if (!session) {
        LOGGER_WARNING(m->log, "Alloc failed! Program might misbehave!");
        return nullptr;
    }

    session->work_buffer_list = (struct RTPWorkBufferList *)calloc(1, sizeof(struct RTPWorkBufferList));

    if (session->work_buffer_list == nullptr) {
        LOGGER_ERROR(m->log, "out of memory while allocating work buffer list");
        free(session);
        return nullptr;
    }

    // First entry is free.
    session->work_buffer_list->next_free_entry = 0;

    session->ssrc = payload_type == RTP_TYPE_VIDEO ? 0 : random_u32();
    session->payload_type = payload_type;
    session->m = m;
    session->friend_number = friendnumber;

    // set NULL just in case
    session->mp = nullptr;
    session->first_packets_counter = 1;

    /* Also set payload type as prefix */
    session->bwc = bwc;
    session->cs = cs;
    session->mcb = mcb;

    for (int ii = 0; ii < INCOMING_PACKETS_TS_ENTRIES; ii++) {
        session->incoming_packets_ts[ii] = 0;
    }

    session->incoming_packets_ts_index = 0;
    session->incoming_packets_ts_last_ts = -1;
    session->incoming_packets_ts_average = 0;

    if (-1 == rtp_allow_receiving(session)) {
        LOGGER_WARNING(m->log, "Failed to start rtp receiving mode");
        free(session->work_buffer_list);
        free(session);
        return nullptr;
    }

    return session;
}

void rtp_kill(RTPSession *session)
{
    if (!session) {
        return;
    }

    LOGGER_DEBUG(session->m->log, "Terminated RTP session: %p", (void *)session);
    rtp_stop_receiving(session);

    LOGGER_DEBUG(session->m->log, "Terminated RTP session V3 work_buffer_list->next_free_entry: %d",
                 (int)session->work_buffer_list->next_free_entry);

    free(session->work_buffer_list);
    free(session);
}

int rtp_allow_receiving(RTPSession *session)
{
    if (session == nullptr) {
        return -1;
    }

    if (session->payload_type == RTP_TYPE_VIDEO) {
        if (m_callback_rtp_packet(session->m, session->friend_number, PACKET_LOSSLESS_VIDEO,
                                  handle_rtp_packet, session) == -1) {
            LOGGER_DEBUG(session->m->log, "Failed to register rtp receive handler");
            return -1;
        }
    }

    if (session->payload_type == RTP_TYPE_VIDEO) {
        if (m_callback_rtp_packet(session->m, session->friend_number, PACKET_TOXAV_COMM_CHANNEL,
                                  handle_rtp_packet, session) == -1) {
            LOGGER_DEBUG(session->m->log, "Failed to register rtp receive handler");
            return -1;
        }
    }

    if (m_callback_rtp_packet(session->m, session->friend_number, session->payload_type,
                              handle_rtp_packet, session) == -1) {
        LOGGER_WARNING(session->m->log, "Failed to register rtp receive handler");
        return -1;
    }

    LOGGER_DEBUG(session->m->log, "Started receiving on session: %p", (void *)session);
    return 0;
}

int rtp_stop_receiving(RTPSession *session)
{
    if (session == nullptr) {
        return -1;
    }

    m_callback_rtp_packet(session->m, session->friend_number, session->payload_type, NULL, NULL);

    if (session->payload_type == RTP_TYPE_VIDEO) {
        m_callback_rtp_packet(session->m, session->friend_number, PACKET_LOSSLESS_VIDEO, NULL, NULL);
    }

    if (session->payload_type == RTP_TYPE_VIDEO) {
        m_callback_rtp_packet(session->m, session->friend_number, PACKET_TOXAV_COMM_CHANNEL, NULL, NULL);
    }

    LOGGER_DEBUG(session->m->log, "Stopped receiving on session: %p", (void *)session);
    return 0;
}

/**
 * @param input is raw vpx data.
 * @param length is the length of the raw data.
 */
int rtp_send_data(RTPSession *session, const uint8_t *data, uint32_t length, bool is_keyframe,
                  uint64_t frame_record_timestamp, int32_t fragment_num,
                  uint32_t codec_used, uint32_t bit_rate_used,
                  uint32_t client_capture_delay_ms,
                  uint32_t video_frame_orientation_angle,
                  Logger *log)
{
    if (!session) {
        LOGGER_ERROR(log, "No session!");
        return -1;
    }

    uint8_t is_video_payload = 0;

    if (session->payload_type == RTP_TYPE_VIDEO) {
        is_video_payload = 1;
    }

    struct RTPHeader header = {0};

    header.ve = 2;  // this is unused in toxav

    header.pe = 0;

    header.xe = 0;

    header.cc = 0;

    header.ma = 0;

    header.pt = session->payload_type % 128;

    header.sequnum = session->sequnum;

    header.timestamp = frame_record_timestamp; // current_time_monotonic(session->m->mono_time);

    header.ssrc = session->ssrc;

    header.offset_lower = 0;

    header.data_length_lower = length;

    header.flags = 0; // just to be safe, initialize the flags to zero

    header.flags = RTP_LARGE_FRAME | RTP_ENCODER_HAS_RECORD_TIMESTAMP;

    if ((codec_used == TOXAV_ENCODER_CODEC_USED_H264) &&
            (is_video_payload == 1)) {
        header.flags = header.flags | RTP_ENCODER_IS_H264;
    }

    if (video_frame_orientation_angle == TOXAV_CLIENT_INPUT_VIDEO_ORIENTATION_90) {
        header.flags = header.flags | RTP_ENCODER_VIDEO_ROTATION_ANGLE_BIT0;
    } else if (video_frame_orientation_angle == TOXAV_CLIENT_INPUT_VIDEO_ORIENTATION_180) {
        header.flags = header.flags | RTP_ENCODER_VIDEO_ROTATION_ANGLE_BIT1;
    } else if (video_frame_orientation_angle == TOXAV_CLIENT_INPUT_VIDEO_ORIENTATION_270) {
        header.flags = header.flags | RTP_ENCODER_VIDEO_ROTATION_ANGLE_BIT0;
        header.flags = header.flags | RTP_ENCODER_VIDEO_ROTATION_ANGLE_BIT1;
    }

    LOGGER_DEBUG(session->m->log, "FRAMEFLAGS:%d angle=%d", (int)header.flags, video_frame_orientation_angle);


    header.frame_record_timestamp = frame_record_timestamp;

    header.fragment_num = fragment_num;

    header.real_frame_num = 0; // not yet used

    header.encoder_bit_rate_used = bit_rate_used;

    header.client_video_capture_delay_ms = client_capture_delay_ms;

    uint16_t length_safe = (uint16_t)length;

    if (length > UINT16_MAX) {
        length_safe = UINT16_MAX;
    }

    header.data_length_lower = length_safe;
    header.data_length_full = length; // without header
    header.offset_lower = 0;
    header.offset_full = 0;

    if (is_keyframe) {
        header.flags |= RTP_KEY_FRAME;
    }

    VLA(uint8_t, rdata, length + RTP_HEADER_SIZE + 1);
    memset(rdata, 0, SIZEOF_VLA(rdata));
    rdata[0] = session->payload_type;  // packet id == payload_type

    if (session->payload_type == RTP_TYPE_VIDEO) {
        if (TOXAV_SEND_VIDEO_LOSSLESS_PACKETS == 1) {
            // video payload
            rdata[0] = PACKET_LOSSLESS_VIDEO; // rewrite to lossless!
        }
    }

    if (MAX_CRYPTO_DATA_SIZE > (length + RTP_HEADER_SIZE + 1)) {
        /**
         * The length is lesser than the maximum allowed length (including header)
         * Send the packet in single piece.
         */
        rtp_header_pack(rdata + 1, &header);
        memcpy(rdata + 1 + RTP_HEADER_SIZE, data, length);


        if ((session->payload_type == RTP_TYPE_VIDEO) && (TOXAV_SEND_VIDEO_LOSSLESS_PACKETS == 1)) {
            if (-1 == send_custom_lossless_packet(session->m, session->friend_number, rdata, SIZEOF_VLA(rdata))) {
                LOGGER_WARNING(session->m->log, "RTP send failed (len: %zu)! std error: %s", SIZEOF_VLA(rdata), strerror(errno));
            }
        } else {
            if (-1 == m_send_custom_lossy_packet(session->m, session->friend_number, rdata, SIZEOF_VLA(rdata))) {
                LOGGER_WARNING(session->m->log, "RTP send failed (len: %zu)! std error: %s", SIZEOF_VLA(rdata), strerror(errno));
            }
        }
    } else {
        /**
         * The length is greater than the maximum allowed length (including header)
         * Send the packet in multiple pieces.
         */
        uint32_t sent = 0;
        uint16_t piece = MAX_CRYPTO_DATA_SIZE - (RTP_HEADER_SIZE + 1);

        while ((length - sent) + RTP_HEADER_SIZE + 1 > MAX_CRYPTO_DATA_SIZE) {
            rtp_header_pack(rdata + 1, &header);
            memcpy(rdata + 1 + RTP_HEADER_SIZE, data + sent, piece);

            if ((session->payload_type == RTP_TYPE_VIDEO) && (TOXAV_SEND_VIDEO_LOSSLESS_PACKETS == 1)) {
                if (-1 == send_custom_lossless_packet(session->m, session->friend_number,
                                                      rdata, piece + RTP_HEADER_SIZE + 1)) {
                    LOGGER_WARNING(session->m->log, "RTP send failed (len: %d)! std error: %s",
                                   piece + RTP_HEADER_SIZE + 1, strerror(errno));
                }
            } else {
                if (-1 == m_send_custom_lossy_packet(session->m, session->friend_number,
                                                     rdata, piece + RTP_HEADER_SIZE + 1)) {
                    LOGGER_WARNING(session->m->log, "RTP send failed (len: %d)! std error: %s",
                                   piece + RTP_HEADER_SIZE + 1, strerror(errno));
                }
            }

            sent += piece;
            header.offset_lower = sent;
            header.offset_full = sent; // raw data offset, without any header
        }

        /* Send remaining */
        piece = length - sent;

        if (piece) {
            rtp_header_pack(rdata + 1, &header);
            memcpy(rdata + 1 + RTP_HEADER_SIZE, data + sent, piece);

            if ((session->payload_type == RTP_TYPE_VIDEO) && (TOXAV_SEND_VIDEO_LOSSLESS_PACKETS == 1)) {
                if (-1 == send_custom_lossless_packet(session->m, session->friend_number, rdata,
                                                      piece + RTP_HEADER_SIZE + 1)) {
                    LOGGER_WARNING(session->m->log, "RTP send failed (len: %d)! std error: %s",
                                   piece + RTP_HEADER_SIZE + 1, strerror(errno));
                }
            } else {
                if (-1 == m_send_custom_lossy_packet(session->m, session->friend_number, rdata,
                                                     piece + RTP_HEADER_SIZE + 1)) {
                    LOGGER_WARNING(session->m->log, "RTP send failed (len: %d)! std error: %s",
                                   piece + RTP_HEADER_SIZE + 1, strerror(errno));
                }
            }
        }
    }

    ++session->sequnum;
    return 0;
}
