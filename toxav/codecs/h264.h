/**
 * @file h264.h  Generic H264 Packetization methods
 *
 * Copyright (C) 2010 Creytiv.com
 */

/** NAL unit types (RFC 3984, Table 1) */
enum {
	H264_NAL_UNKNOWN      = 0,
	/* 1-23   NAL unit  Single NAL unit packet per H.264 */
	H264_NAL_SLICE        = 1,
	H264_NAL_DPA          = 2,
	H264_NAL_DPB          = 3,
	H264_NAL_DPC          = 4,
	H264_NAL_IDR_SLICE    = 5,
	H264_NAL_SEI          = 6,
	H264_NAL_SPS          = 7,
	H264_NAL_PPS          = 8,
	H264_NAL_AUD          = 9,
	H264_NAL_END_SEQUENCE = 10,
	H264_NAL_END_STREAM   = 11,
	H264_NAL_FILLER_DATA  = 12,
	H264_NAL_SPS_EXT      = 13,
	H264_NAL_AUX_SLICE    = 19,

	H264_NAL_STAP_A       = 24,  /**< Single-time aggregation packet */
	H264_NAL_STAP_B       = 25,  /**< Single-time aggregation packet */
	H264_NAL_MTAP16       = 26,  /**< Multi-time aggregation packet  */
	H264_NAL_MTAP24       = 27,  /**< Multi-time aggregation packet  */
	H264_NAL_FU_A         = 28,  /**< Fragmentation unit             */
	H264_NAL_FU_B         = 29,  /**< Fragmentation unit             */
};

/**
 * H.264 Header defined in RFC 3984
 *
 * <pre>
      +---------------+
      |0|1|2|3|4|5|6|7|
      +-+-+-+-+-+-+-+-+
      |F|NRI|  Type   |
      +---------------+
 * </pre>
 */
struct h264_hdr {
	unsigned f:1;      /**< 1 bit  - Forbidden zero bit (must be 0) */
	unsigned nri:2;    /**< 2 bits - nal_ref_idc                    */
	unsigned type:5;   /**< 5 bits - nal_unit_type                  */
};

int h264_hdr_encode(const struct h264_hdr *hdr, struct mbuf *mb);
int h264_hdr_decode(struct h264_hdr *hdr, struct mbuf *mb);

/** Fragmentation Unit header */
struct h264_fu {
	unsigned s:1;      /**< Start bit                               */
	unsigned e:1;      /**< End bit                                 */
	unsigned r:1;      /**< The Reserved bit MUST be equal to 0     */
	unsigned type:5;   /**< The NAL unit payload type               */
};

int h264_fu_hdr_encode(const struct h264_fu *fu, struct mbuf *mb);
int h264_fu_hdr_decode(struct h264_fu *fu, struct mbuf *mb);

const uint8_t *h264_find_startcode(const uint8_t *p, const uint8_t *end);

int h264_packetize(uint64_t rtp_ts, const uint8_t *buf, size_t len,
		   size_t pktsize, videnc_packet_h *pkth, void *arg);
int h264_nal_send(bool first, bool last,
		  bool marker, uint32_t ihdr, uint64_t rtp_ts,
		  const uint8_t *buf, size_t size, size_t maxsz,
		  videnc_packet_h *pkth, void *arg);
const char *h264_nalunit_name(int type);
static inline bool h264_is_keyframe(int type)
{
	return type == H264_NAL_IDR_SLICE;
}

