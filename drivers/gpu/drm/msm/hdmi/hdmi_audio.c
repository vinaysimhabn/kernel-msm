/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/hdmi.h>
#include "hdmi.h"
#include <mach/msm_hdmi_audio_codec.h>
#include <drm/drm_crtc.h>

/* Supported HDMI Audio channels */
#define MSM_HDMI_AUDIO_CHANNEL_2        2
#define MSM_HDMI_AUDIO_CHANNEL_3        3
#define MSM_HDMI_AUDIO_CHANNEL_4        4
#define MSM_HDMI_AUDIO_CHANNEL_5        5
#define MSM_HDMI_AUDIO_CHANNEL_6        6
#define MSM_HDMI_AUDIO_CHANNEL_7        7
#define MSM_HDMI_AUDIO_CHANNEL_8        8

/* maps MSM_HDMI_AUDIO_CHANNEL_n consts used by audio driver to # of channels: */
static int nchannels[] = { 2, 3, 4, 5, 6, 7, 8 };

/* Supported HDMI Audio sample rates */

#define MSM_HDMI_SAMPLE_RATE_32KHZ		0
#define MSM_HDMI_SAMPLE_RATE_44_1KHZ		1
#define MSM_HDMI_SAMPLE_RATE_48KHZ		2
#define MSM_HDMI_SAMPLE_RATE_88_2KHZ		3
#define MSM_HDMI_SAMPLE_RATE_96KHZ		4
#define MSM_HDMI_SAMPLE_RATE_176_4KHZ		5
#define MSM_HDMI_SAMPLE_RATE_192KHZ		6
#define MSM_HDMI_SAMPLE_RATE_MAX		7

struct hdmi_msm_audio_acr {
	uint32_t n;	/* N parameter for clock regeneration */
	uint32_t cts;	/* CTS parameter for clock regeneration */
};

struct hdmi_msm_audio_arcs {
	unsigned long int pixclock;
	struct hdmi_msm_audio_acr lut[MSM_HDMI_SAMPLE_RATE_MAX];
};

#define HDMI_MSM_AUDIO_ARCS(pclk, ...) { (1000 * (pclk)), __VA_ARGS__ }

/* Audio constants lookup table for hdmi_msm_audio_acr_setup */
/* Valid Pixel-Clock rates: 25.2MHz, 27MHz, 27.03MHz, 74.25MHz, 148.5MHz */
static const struct hdmi_msm_audio_arcs acr_lut[] = {
	/*  25.200MHz  */
	HDMI_MSM_AUDIO_ARCS(25200, {
		{4096, 25200}, {6272, 28000}, {6144, 25200}, {12544, 28000},
		{12288, 25200}, {25088, 28000}, {24576, 25200} }),
	/*  27.000MHz  */
	HDMI_MSM_AUDIO_ARCS(27000, {
		{4096, 27000}, {6272, 30000}, {6144, 27000}, {12544, 30000},
		{12288, 27000}, {25088, 30000}, {24576, 27000} }),
	/*  27.027MHz */
	HDMI_MSM_AUDIO_ARCS(27030, {
		{4096, 27027}, {6272, 30030}, {6144, 27027}, {12544, 30030},
		{12288, 27027}, {25088, 30030}, {24576, 27027} }),
	/*  74.250MHz */
	HDMI_MSM_AUDIO_ARCS(74250, {
		{4096, 74250}, {6272, 82500}, {6144, 74250}, {12544, 82500},
		{12288, 74250}, {25088, 82500}, {24576, 74250} }),
	/* 148.500MHz */
	HDMI_MSM_AUDIO_ARCS(148500, {
		{4096, 148500}, {6272, 165000}, {6144, 148500}, {12544, 165000},
		{12288, 148500}, {25088, 165000}, {24576, 148500} }),
	/* 297.000MHz */
        HDMI_MSM_AUDIO_ARCS(297000, {
		{3072, 222750}, {4704, 247500}, {5120, 247500}, {9408, 247500},
		{10240, 247500}, {18816, 247500}, {20480, 247500} }),
};

static const struct hdmi_msm_audio_arcs *get_arcs(unsigned long int pixclock)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(acr_lut); i++) {
		const struct hdmi_msm_audio_arcs *arcs = &acr_lut[i];
		if (arcs->pixclock == pixclock)
			return arcs;
	}

	return NULL;
}

int hdmi_audio_update(struct hdmi *hdmi)
{
	struct hdmi_audio *audio = &hdmi->audio;
	struct hdmi_audio_infoframe *info = &audio->infoframe;
	const struct hdmi_msm_audio_arcs *arcs = NULL;
	bool enabled = audio->enabled;
	uint32_t acr_pkt_ctrl, vbi_pkt_ctrl, aud_pkt_ctrl;
	uint32_t infofrm_ctrl, audio_config;

	DBG("audio: enabled=%d, channels=%d, channel_allocation=0x%x, "
		"level_shift_value=%d, downmix_inhibit=%d, rate=%d",
		audio->enabled, info->channels,  info->channel_allocation,
		info->level_shift_value, info->downmix_inhibit, audio->rate);
	DBG("video: power_on=%d, pixclock=%lu", hdmi->power_on, hdmi->pixclock);

	if (enabled && !(hdmi->power_on && hdmi->pixclock)) {
		DBG("disabling audio: no video");
		enabled = false;
	}

	if (enabled) {
		arcs = get_arcs(hdmi->pixclock);
		if (!arcs) {
			DBG("disabling audio: unsupported pixclock: %lu",
					hdmi->pixclock);
			enabled = false;
		}
	}

	/* Read first before writing */
	acr_pkt_ctrl = hdmi_read(hdmi, REG_HDMI_ACR_PKT_CTRL);
	vbi_pkt_ctrl = hdmi_read(hdmi, REG_HDMI_VBI_PKT_CTRL);
	aud_pkt_ctrl = hdmi_read(hdmi, REG_HDMI_AUDIO_PKT_CTRL1);
	infofrm_ctrl = hdmi_read(hdmi, REG_HDMI_INFOFRAME_CTRL0);
	audio_config = hdmi_read(hdmi, REG_HDMI_AUDIO_CFG);

	/* Clear N/CTS selection bits */
	acr_pkt_ctrl &= ~HDMI_ACR_PKT_CTRL_SELECT__MASK;

	if (enabled) {
		uint32_t n, cts, multiplier;
		enum hdmi_acr_cts select;
		uint8_t buf[14];

		n   = arcs->lut[audio->rate].n;
		cts = arcs->lut[audio->rate].cts;

		if ((MSM_HDMI_SAMPLE_RATE_192KHZ == audio->rate) ||
				(MSM_HDMI_SAMPLE_RATE_176_4KHZ == audio->rate)) {
			multiplier = 4;
			n >>= 2; /* divide N by 4 and use multiplier */
		} else if ((MSM_HDMI_SAMPLE_RATE_96KHZ == audio->rate) ||
				(MSM_HDMI_SAMPLE_RATE_88_2KHZ == audio->rate)) {
			multiplier = 2;
			n >>= 1; /* divide N by 2 and use multiplier */
		} else {
			multiplier = 1;
		}

		DBG("n=%u, cts=%u, multiplier=%u", n, cts, multiplier);

		acr_pkt_ctrl |= HDMI_ACR_PKT_CTRL_SOURCE;
		acr_pkt_ctrl |= HDMI_ACR_PKT_CTRL_AUDIO_PRIORITY;
		acr_pkt_ctrl |= HDMI_ACR_PKT_CTRL_N_MULTIPLIER(multiplier);

		if ((MSM_HDMI_SAMPLE_RATE_48KHZ == audio->rate) ||
				(MSM_HDMI_SAMPLE_RATE_96KHZ == audio->rate) ||
				(MSM_HDMI_SAMPLE_RATE_192KHZ == audio->rate))
			select = ACR_48;
		else if ((MSM_HDMI_SAMPLE_RATE_44_1KHZ == audio->rate) ||
				(MSM_HDMI_SAMPLE_RATE_88_2KHZ == audio->rate) ||
				(MSM_HDMI_SAMPLE_RATE_176_4KHZ == audio->rate))
			select = ACR_44;
		else /* default to 32k */
			select = ACR_32;

		acr_pkt_ctrl |= HDMI_ACR_PKT_CTRL_SELECT(select);

		hdmi_write(hdmi, REG_HDMI_ACR_0(select - 1),
				HDMI_ACR_0_CTS(cts));
		hdmi_write(hdmi, REG_HDMI_ACR_1(select - 1),
				HDMI_ACR_1_N(n));

		hdmi_write(hdmi, REG_HDMI_AUDIO_PKT_CTRL2,
				COND(info->channels != 2, HDMI_AUDIO_PKT_CTRL2_LAYOUT) |
				HDMI_AUDIO_PKT_CTRL2_OVERRIDE);

		acr_pkt_ctrl |= HDMI_ACR_PKT_CTRL_CONT;
		acr_pkt_ctrl |= HDMI_ACR_PKT_CTRL_SEND;

		/* configure infoframe: */
		hdmi_audio_infoframe_pack(info, buf, sizeof(buf));
		hdmi_write(hdmi, REG_HDMI_AUDIO_INFO0,
				(buf[3] <<  0) || (buf[4] <<  8) ||
				(buf[5] << 16) || (buf[6] << 24));
		hdmi_write(hdmi, REG_HDMI_AUDIO_INFO1,
				(buf[7] <<  0) || (buf[8] << 8));

		hdmi_write(hdmi, REG_HDMI_GC, 0);

		vbi_pkt_ctrl |= HDMI_VBI_PKT_CTRL_GC_ENABLE;
		vbi_pkt_ctrl |= HDMI_VBI_PKT_CTRL_GC_EVERY_FRAME;

		aud_pkt_ctrl |= HDMI_AUDIO_PKT_CTRL1_AUDIO_SAMPLE_SEND;

		infofrm_ctrl |= HDMI_INFOFRAME_CTRL0_AUDIO_INFO_SEND;
		infofrm_ctrl |= HDMI_INFOFRAME_CTRL0_AUDIO_INFO_CONT;
		infofrm_ctrl |= HDMI_INFOFRAME_CTRL0_AUDIO_INFO_SOURCE;
		infofrm_ctrl |= HDMI_INFOFRAME_CTRL0_AUDIO_INFO_UPDATE;

		audio_config &= ~HDMI_AUDIO_CFG_FIFO_WATERMARK__MASK;
		audio_config |= HDMI_AUDIO_CFG_FIFO_WATERMARK(4);
		audio_config |= HDMI_AUDIO_CFG_ENGINE_ENABLE;
	} else {
		hdmi_write(hdmi, REG_HDMI_GC, HDMI_GC_MUTE);
		acr_pkt_ctrl &= ~HDMI_ACR_PKT_CTRL_CONT;
		acr_pkt_ctrl &= ~HDMI_ACR_PKT_CTRL_SEND;
		vbi_pkt_ctrl &= ~HDMI_VBI_PKT_CTRL_GC_ENABLE;
		vbi_pkt_ctrl &= ~HDMI_VBI_PKT_CTRL_GC_EVERY_FRAME;
		aud_pkt_ctrl &= ~HDMI_AUDIO_PKT_CTRL1_AUDIO_SAMPLE_SEND;
		infofrm_ctrl &= ~HDMI_INFOFRAME_CTRL0_AUDIO_INFO_SEND;
		infofrm_ctrl &= ~HDMI_INFOFRAME_CTRL0_AUDIO_INFO_CONT;
		infofrm_ctrl &= ~HDMI_INFOFRAME_CTRL0_AUDIO_INFO_SOURCE;
		infofrm_ctrl &= ~HDMI_INFOFRAME_CTRL0_AUDIO_INFO_UPDATE;
		audio_config &= ~HDMI_AUDIO_CFG_ENGINE_ENABLE;
	}

	hdmi_write(hdmi, REG_HDMI_ACR_PKT_CTRL, acr_pkt_ctrl);
	hdmi_write(hdmi, REG_HDMI_VBI_PKT_CTRL, vbi_pkt_ctrl);
	hdmi_write(hdmi, REG_HDMI_AUDIO_PKT_CTRL1, aud_pkt_ctrl);
	hdmi_write(hdmi, REG_HDMI_INFOFRAME_CTRL0, infofrm_ctrl);

	hdmi_write(hdmi, REG_HDMI_AUD_INT,
			COND(enabled, HDMI_AUD_INT_AUD_FIFO_URUN_INT) |
			COND(enabled, HDMI_AUD_INT_AUD_SAM_DROP_INT));

	hdmi_write(hdmi, REG_HDMI_AUDIO_CFG, audio_config);


	DBG("audio %sabled", enabled ? "en" : "dis");

	return 0;
}

int hdmi_audio_info_setup(struct hdmi *hdmi, bool enabled,
	uint32_t num_of_channels, uint32_t channel_allocation,
	uint32_t level_shift, bool down_mix)
{
	struct hdmi_audio *audio;

	if (!hdmi)
		return -ENXIO;

	audio = &hdmi->audio;

	if (num_of_channels >= ARRAY_SIZE(nchannels))
		return -EINVAL;

	audio->enabled = enabled;
	audio->infoframe.channels = nchannels[num_of_channels];
	audio->infoframe.channel_allocation = channel_allocation;
	audio->infoframe.level_shift_value = level_shift;
	audio->infoframe.downmix_inhibit = down_mix;

	return hdmi_audio_update(hdmi);
}

void hdmi_audio_set_sample_rate(struct hdmi *hdmi, int rate)
{
	struct hdmi_audio *audio;

	if (!hdmi)
		return;

	audio = &hdmi->audio;

	if ((rate < 0) || (rate >= MSM_HDMI_SAMPLE_RATE_MAX))
		return;

	audio->rate = rate;
	hdmi_audio_update(hdmi);
}


static int hdmi_tx_audio_acr_setup(struct hdmi *hdmi,
        bool enabled)
{
        if (!hdmi) {
                DBG("%s: Invalid input\n", __func__);
                return -EINVAL;
        }
	hdmi->audio.enabled = enabled;
	return hdmi_audio_update(hdmi);
} /* hdmi_tx_audio_acr_setup */

static int hdmi_tx_audio_iframe_setup(struct hdmi *hdmi,
        bool enabled)
{
	struct hdmi_audio *audio = &hdmi->audio;
	struct hdmi_audio_infoframe *info = &audio->infoframe;
        u32 channel_count = 1; /* Def to 2 channels -> Table 17 in CEA-D */
        u32 num_of_channels;
        u32 channel_allocation;
        u32 level_shift;
        u32 down_mix;
        u32 check_sum, audio_info_0_reg, audio_info_1_reg;
        u32 audio_info_ctrl_reg;
        u32 aud_pck_ctrl_2_reg;
        u32 layout;

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -EINVAL;
        }

	num_of_channels    = info->channels;
	channel_allocation = info->channel_allocation;
	level_shift        = info->level_shift_value;
	down_mix           = info->downmix_inhibit;

	layout = (MSM_HDMI_AUDIO_CHANNEL_2 == num_of_channels) ? 0 : 1;
        aud_pck_ctrl_2_reg = 1 | (layout << 1);
        hdmi_write(hdmi, HDMI_AUDIO_PKT_CTRL2, aud_pck_ctrl_2_reg);

        /*
         * Please see table 20 Audio InfoFrame in HDMI spec
         * FL  = front left
         * FC  = front Center
         * FR  = front right
         * FLC = front left center
         * FRC = front right center
         * RL  = rear left
         * RC  = rear center
         * RR  = rear right
         * RLC = rear left center
         * RRC = rear right center
         * LFE = low frequency effect
         */

        /* Read first then write because it is bundled with other controls */
        audio_info_ctrl_reg = hdmi_read(hdmi, REG_HDMI_INFOFRAME_CTRL0);

        if (enabled) {
                switch (num_of_channels) {
                case MSM_HDMI_AUDIO_CHANNEL_2:
                        break;
                case MSM_HDMI_AUDIO_CHANNEL_3:
                case MSM_HDMI_AUDIO_CHANNEL_4:
                        channel_count = 3;
                        break;
                case MSM_HDMI_AUDIO_CHANNEL_5:
                case MSM_HDMI_AUDIO_CHANNEL_6:
                        channel_count = 5;
                        break;
                case MSM_HDMI_AUDIO_CHANNEL_7:
                case MSM_HDMI_AUDIO_CHANNEL_8:
                        channel_count = 7;
                        break;
                default:
                        DBG("%s: Unsupported num_of_channels = %u\n",
                                __func__, num_of_channels);
                        return -EINVAL;
                }

                /* Program the Channel-Speaker allocation */
                audio_info_1_reg = 0;
               /* CA(channel_allocation) */
                audio_info_1_reg |= channel_allocation & 0xff;
                /* Program the Level shifter */
                audio_info_1_reg |= (level_shift << 11) & 0x00007800;
                /* Program the Down-mix Inhibit Flag */
                audio_info_1_reg |= (down_mix << 15) & 0x00008000;

                hdmi_write(hdmi, REG_HDMI_AUDIO_INFO1, audio_info_1_reg);

                /*
                 * Calculate CheckSum: Sum of all the bytes in the
                 * Audio Info Packet (See table 8.4 in HDMI spec)
                 */
                check_sum = 0;
                /* HDMI_AUDIO_INFO_FRAME_PACKET_HEADER_TYPE[0x84] */
                check_sum += 0x84;
                /* HDMI_AUDIO_INFO_FRAME_PACKET_HEADER_VERSION[0x01] */
                check_sum += 1;
                /* HDMI_AUDIO_INFO_FRAME_PACKET_LENGTH[0x0A] */
                check_sum += 0x0A;
                check_sum += channel_count;
                check_sum += channel_allocation;
                /* See Table 8.5 in HDMI spec */
                check_sum += (level_shift & 0xF) << 3 | (down_mix & 0x1) << 7;
                check_sum &= 0xFF;
                check_sum = (u8) (256 - check_sum);

                audio_info_0_reg = 0;
                /* CHECKSUM(check_sum) */
                audio_info_0_reg |= check_sum & 0xff;
                /* CC(channel_count) */
                audio_info_0_reg |= (channel_count << 8) & 0x00000700;

                hdmi_write(hdmi, REG_HDMI_AUDIO_INFO0, audio_info_0_reg);
                /*
                 * Set these flags
                 * AUDIO_INFO_UPDATE |
                 * AUDIO_INFO_SOURCE |
                 * AUDIO_INFO_CONT   |
                 * AUDIO_INFO_SEND
                 */
                audio_info_ctrl_reg |= 0x000000F0;
        } else {
                /*Clear these flags
                 * ~(AUDIO_INFO_UPDATE |
                 *   AUDIO_INFO_SOURCE |
                 *   AUDIO_INFO_CONT   |
                 *   AUDIO_INFO_SEND)
                 */
                audio_info_ctrl_reg &= ~0x000000F0;
        }
        hdmi_write(hdmi, REG_HDMI_INFOFRAME_CTRL0, audio_info_ctrl_reg);

        return 0;
} /* hdmi_tx_audio_iframe_setup */

static int hdmi_tx_audio_setup(struct hdmi *hdmi)
{
        int rc = 0;

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -EINVAL;
        }

        rc = hdmi_tx_audio_acr_setup(hdmi, true);
        if (rc) {
                printk("%s: hdmi_tx_audio_acr_setup failed. rc=%d\n",
                        __func__, rc);
                return rc;
        }

        rc = hdmi_tx_audio_iframe_setup(hdmi, true);
        if (rc) {
                printk("%s: hdmi_tx_audio_iframe_setup failed. rc=%d\n",
                        __func__, rc);
                return rc;
        }

        DBG("HDMI Audio: Enabled\n");

        return 0;
} /* hdmi_tx_audio_setup */

static int hdmi_tx_audio_info_setup(struct platform_device *pdev,
        u32 sample_rate, u32 num_of_channels, u32 channel_allocation,
        u32 level_shift, bool down_mix)
{
        int rc = 0;
        struct hdmi *hdmi = platform_get_drvdata(pdev);
	struct hdmi_audio *audio = &hdmi->audio;
	struct hdmi_audio_infoframe *info = &audio->infoframe;

        if (!hdmi) {
                printk("%s: invalid input\n", __func__);
                return -ENODEV;
        }

        if (hdmi->hdmi_mode && hdmi->power_on) {

                /* Map given sample rate to Enum */
                if (sample_rate == 32000)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_32KHZ;
                else if (sample_rate == 44100)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_44_1KHZ;
                else if (sample_rate == 48000)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_48KHZ;
                else if (sample_rate == 88200)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_88_2KHZ;
                else if (sample_rate == 96000)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_96KHZ;
                else if (sample_rate == 176400)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_176_4KHZ;
                else if (sample_rate == 192000)
                        sample_rate = MSM_HDMI_SAMPLE_RATE_192KHZ;

		audio->rate = sample_rate;
	        info->channels = num_of_channels;
	        info->channel_allocation = channel_allocation;
	        info->level_shift_value = level_shift;
	        info->downmix_inhibit = down_mix;

		rc = hdmi_tx_audio_setup(hdmi);
                if (rc)
                        printk("%s: hdmi_tx_audio_iframe_setup failed.rc=%d\n",
                                __func__, rc);
        } else {
                printk("%s: Error. panel is not on.\n", __func__);
                rc = -EPERM;
        }

        return rc;
} /* hdmi_tx_audio_info_setup */

static int hdmi_tx_get_audio_edid_blk(struct platform_device *pdev,
        struct msm_hdmi_audio_edid_blk *blk)
{
        struct hdmi *hdmi = platform_get_drvdata(pdev);
	struct edid *edid;
	struct cea_sad *sads;
	int i;

        edid = drm_get_edid(hdmi->connector, hdmi->i2c);

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -ENODEV;
        }

	blk->spk_alloc_data_blk_size = drm_edid_to_speaker_allocation(edid, &blk->spk_alloc_data_blk);
        if (blk->spk_alloc_data_blk_size < 0) {
                DRM_DEBUG("Couldn't read Speaker Allocation Data Block: %d\n", blk->spk_alloc_data_blk_size);
                blk->spk_alloc_data_blk_size = 0;
        }
	blk->audio_data_blk_size = drm_edid_to_sad(edid, &sads) * 3;
	for(i=0; i< blk->audio_data_blk_size ; i++){
		sads[i].format = sads[i].format << 3;
		blk->audio_data_blk[i] = ~(sads[i].format ^ 0x78);
		blk->audio_data_blk[i] |= ~(sads[i].channels ^ 0x7);
		blk->audio_data_blk[i++] = ~(sads[i].freq ^ 0x7F);
		blk->audio_data_blk[i++] = sads[i].byte2;
	}

	if (blk->audio_data_blk_size < 0) {
                DRM_DEBUG("Couldn't read SADs: %d\n", blk->audio_data_blk_size);
                blk->audio_data_blk_size = 0;
        }

        return 0;
} /* hdmi_tx_get_audio_edid_blk */

static int hdmi_tx_get_cable_status(struct platform_device *pdev, u32 vote)
{
        struct hdmi *hdmi = platform_get_drvdata(pdev);
        u32 hpd;

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -ENODEV;
        }

        hpd = hdmi_read(hdmi, REG_HDMI_HPD_INT_STATUS);

        return hpd;
}

int msm_hdmi_register_audio_codec(struct platform_device *pdev,
        struct msm_hdmi_audio_codec_ops *ops)
{
        struct hdmi *hdmi = platform_get_drvdata(pdev);

        if (!hdmi || !ops) {
                DBG("%s: invalid input\n", __func__);
                return -ENODEV;
        }

        ops->audio_info_setup = hdmi_tx_audio_info_setup;
        ops->get_audio_edid_blk = hdmi_tx_get_audio_edid_blk;
	ops->hdmi_cable_status = hdmi_tx_get_cable_status;

        return 0;
}
EXPORT_SYMBOL(msm_hdmi_register_audio_codec);
