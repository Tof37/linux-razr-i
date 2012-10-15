/*
 * Copyright (c)  2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
*/

#include "mdfld_panel_generic.h"
#include "mdfld_dsi_esd.h"

int mdfld_panel_generic_dsi_dbi_power_on(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct panel_funcs *p_funcs = dbi_output->p_funcs;
	struct mdfld_dsi_hw_registers *regs = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int err = 0;

	if (!dev_priv->dsi_init_done)
		return 0;

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;


	/*
	 * Different panel may have different ways to have
	 * panel turned on. Support it!
	 */
	if (p_funcs && p_funcs->power_on) {
		p_funcs->dsi_controller_init(dsi_config,
				dsi_config->pipe, true);

		if (p_funcs->power_on(dsi_config)) {
			DRM_ERROR("Failed to power on panel\n");
			err = -EAGAIN;
			goto power_on_err;
		}
	}

power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

int mdfld_panel_generic_dsi_dbi_power_off(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct panel_funcs *p_funcs = dbi_output->p_funcs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int err = 0;

	if (!dev_priv->dsi_init_done)
		return 0;

	if (!dsi_config)
		return -EINVAL;

	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	ctx->lastbrightnesslevel = psb_brightness;

	/*
	 * Different panel may have different ways to have
	 * panel turned off. Support it!
	 */
	if (p_funcs && p_funcs->power_off) {
		if (p_funcs->power_off(dsi_config)) {
			DRM_ERROR("Failed to power off panel\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}

power_off_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

int mdfld_panel_generic_dsi_dbi_set_power(struct drm_encoder *encoder, bool on)
{
	int ret = 0;
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct panel_funcs *p_funcs = dbi_output->p_funcs;
	struct mdfld_dsi_connector *dsi_connector =
		mdfld_dsi_encoder_get_connector(dsi_encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	u32 reg_offset = 0;
	int pipe = (dbi_output->channel_num == 0) ? 0 : 2;

	PSB_DEBUG_ENTRY("pipe %d : %s, panel on: %s\n", pipe, on ? "On" : "Off",
			dbi_output->dbi_panel_on ? "True" : "False");

	if (pipe == 2) {
		if (on)
			dev_priv->dual_mipi = true;
		else
			dev_priv->dual_mipi = false;

		reg_offset = MIPIC_REG_OFFSET;
	} else {
		if (!on)
			dev_priv->dual_mipi = false;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("hw begin failed\n");
		return -EAGAIN;
	}

	mutex_lock(&dsi_config->context_lock);

	if (on) {
		if (dbi_output->dbi_panel_on)
			goto out_err;

		ret = mdfld_panel_generic_dsi_dbi_power_on(encoder);
		if (ret) {
			DRM_ERROR("power on error\n");
			goto out_err;
		}

		dbi_output->dbi_panel_on = true;

		if (pipe == 2)
			dev_priv->dbi_panel_on2 = true;
		else
			dev_priv->dbi_panel_on = true;

		if (dev_priv->platform_rev_id != MDFLD_PNW_A0)
			mdfld_enable_te(dev, pipe);

		/* wake up error detector if ESD enabled */
		if (p_funcs->esd_detection)
			mdfld_dsi_error_detector_wakeup(dsi_connector);
		else
			PSB_DEBUG_ENTRY("ESD detection disabled\n");

		dsi_config->dsi_hw_context.panel_on = 1;
	} else {
		if (!dbi_output->dbi_panel_on && !dbi_output->first_boot)
			goto out_err;

		dbi_output->dbi_panel_on = false;
		dbi_output->first_boot = false;

		if (pipe == 2)
			dev_priv->dbi_panel_on2 = false;
		else
			dev_priv->dbi_panel_on = false;

		if (dev_priv->platform_rev_id != MDFLD_PNW_A0)
			mdfld_disable_te(dev, pipe);

		ret = mdfld_panel_generic_dsi_dbi_power_off(encoder);
		if (ret) {
			DRM_ERROR("power on error\n");
			goto out_err;
		}

		dsi_config->dsi_hw_context.panel_on = 0;
	}

out_err:
	mutex_unlock(&dsi_config->context_lock);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	if (ret)
		DRM_ERROR("failed\n");
	else
		PSB_DEBUG_ENTRY("successfully\n");

	return ret;
}

void mdfld_panel_generic_dsi_dbi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct mdfld_dsi_connector *dsi_connector = dsi_config->connector;
	int pipe = dsi_connector->pipe;
	int err = 0;

	PSB_DEBUG_ENTRY("type %s\n", (pipe == 2) ? "MIPI2" : "MIPI");
	PSB_DEBUG_ENTRY("h %d v %d\n", mode->hdisplay, mode->vdisplay);

	/* TODO: this looks ugly, try to move it to CRTC mode setting */
	if (pipe == 2)
		dev_priv->pipeconf2 |= PIPEACONF_DSR;
	else
		dev_priv->pipeconf |= PIPEACONF_DSR;

	if (err)
		DRM_ERROR("mode set failed\n");
	else
		PSB_DEBUG_ENTRY("mode set done successfully\n");

	return;
}


void mdfld_panel_generic_dsi_dbi_prepare(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);

	PSB_DEBUG_ENTRY("\n");

	dbi_output->mode_flags |= MODE_SETTING_IN_ENCODER;
	dbi_output->mode_flags &= ~MODE_SETTING_ENCODER_DONE;

	mdfld_panel_generic_dsi_dbi_set_power(encoder, false);
	gbdispstatus = false;
}

void mdfld_panel_generic_dsi_dbi_commit(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder =
		MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	/*DSI DPU was still on debugging, will remove this option later*/
#ifdef CONFIG_MDFLD_DSI_DPU
	struct psb_drm_dpu_rect rect;
#endif

	PSB_DEBUG_ENTRY("\n");

	mdfld_panel_generic_dsi_dbi_set_power(encoder, true);
	gbdispstatus = true;
	dbi_output->mode_flags &= ~MODE_SETTING_IN_ENCODER;

#ifdef CONFIG_MDFLD_DSI_DPU
	rect.x = rect.y = 0;
	rect.width = 864;
	rect.height = 480;
#endif

	dbi_output->mode_flags |= MODE_SETTING_ENCODER_DONE;
}


void mdfld_panel_generic_dsi_dbi_dpms(struct drm_encoder *encoder, int mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_device *dev = dbi_output->dev;
	static bool bdispoff;

	PSB_DEBUG_ENTRY("%s:\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));

	if (mode == DRM_MODE_DPMS_ON) {
		/*
		 * FIXME: in case I am wrong!
		 * we don't need to exit dsr here to wake up plane/pipe/pll
		 * if everything goes right, hw_begin will resume them all
		 * during set_power.
		 */
		if (bdispoff)
			mdfld_dsi_dbi_exit_dsr(dev, MDFLD_DSR_2D_3D, 0, 0);

		mdfld_panel_generic_dsi_dbi_set_power(encoder, true);

		if (gbgfxsuspended)
			gbgfxsuspended = false;

		bdispoff = false;
		gbdispstatus = true;
	} else {
		/*
		 * I am not sure whether this is the perfect place to
		 * turn rpm on since we still have a lot of CRTC turnning
		 * on work to do.
		 */
		mdfld_panel_generic_dsi_dbi_set_power(encoder, false);
		bdispoff = true;
		gbdispstatus = false;
	}
}

void mdfld_panel_generic_dsi_dbi_save(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (!encoder)
		return;

	if (!dev_priv->dsi_init_done)
		return;

	/*turn off*/
	mdfld_panel_generic_dsi_dbi_set_power(encoder, false);
}

void mdfld_panel_generic_dsi_dbi_update_fb(
	struct mdfld_dsi_dbi_output *dbi_output,
	int pipe)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dbi_output->base.base.crtc;
	struct psb_intel_crtc *psb_crtc =
		(crtc) ? to_psb_intel_crtc(crtc) : NULL;
	u32 dpll_reg = MRST_DPLL_A;
	u32 dspcntr_reg = DSPACNTR;
	u32 pipeconf_reg = PIPEACONF;
	u32 dsplinoff_reg = DSPALINOFF;
	u32 dspsurf_reg = DSPASURF;

	if (!dev_priv->dsi_init_done)
		return;

	/* if mode setting on-going, back off */
	if ((dbi_output->mode_flags & MODE_SETTING_ON_GOING) ||
			(psb_crtc &&
			 (psb_crtc->mode_flags & MODE_SETTING_ON_GOING)) ||
			!(dbi_output->mode_flags & MODE_SETTING_ENCODER_DONE))
		return;

	if (pipe == 2) {
		dspcntr_reg = DSPCCNTR;
		pipeconf_reg = PIPECCONF;
		dsplinoff_reg = DSPCLINOFF;
		dspsurf_reg = DSPCSURF;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("hw begin failed\n");
		return;
	}

	/* check DBI FIFO status */
	if (!(REG_READ(dpll_reg) & DPLL_VCO_ENABLE) ||
	   !(REG_READ(dspcntr_reg) & DISPLAY_PLANE_ENABLE) ||
	   !(REG_READ(pipeconf_reg) & DISPLAY_PLANE_ENABLE)) {
		goto update_fb_out0;
	}

	/* refresh plane changes */
	REG_WRITE(dsplinoff_reg, REG_READ(dsplinoff_reg));
	REG_WRITE(dspsurf_reg, REG_READ(dspsurf_reg));
	REG_READ(dspsurf_reg);

	mdfld_dsi_send_dcs(sender,
			   write_mem_start,
			   NULL,
			   0,
			   CMD_DATA_SRC_PIPE,
			   MDFLD_DSI_SEND_PACKAGE);

	dbi_output->dsr_fb_update_done = true;


update_fb_out0:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}
