/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
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
 * Authors:
 *  jim liu <jim.liu@intel.com>
 *  Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"

#ifdef CONFIG_GFX_RTPM
 #include <linux/pm_runtime.h>
#endif

int enable_gfx_rtpm = 0;

int enter_dsr = 0;
struct mdfld_dsi_dbi_output *gdbi_output;

#ifdef CONFIG_GFX_RTPM
static void psb_runtimepm_wq_handler(struct work_struct *work);
DECLARE_DELAYED_WORK(rtpm_work, psb_runtimepm_wq_handler);

void psb_runtimepm_wq_handler(struct work_struct *work)
{
	struct drm_psb_private * dev_priv =  gpDrmDevice->dev_private;

	if(drm_psb_ospm && !enable_gfx_rtpm) {
		printk(KERN_ALERT "Enable GFX runtime_pm \n");

		dev_priv->rpm_enabled = 1;

		enable_gfx_rtpm = 1;

	        pm_runtime_enable(&gpDrmDevice->pdev->dev);
		pm_runtime_set_active(&gpDrmDevice->pdev->dev);

	        pm_runtime_allow(&gpDrmDevice->pdev->dev);
	}
}
#endif


/**
 * set refreshing area
 */
int mdfld_dsi_dbi_update_area(struct mdfld_dsi_dbi_output * dbi_output,
				u16 x1, u16 y1, u16 x2, u16 y2)
{
	struct mdfld_dsi_pkg_sender * sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	u8 param[4];
	u8 cmd;
	int err;

	if(!sender) {
		DRM_ERROR("Cannot get PKG sender\n");
		return -EINVAL;
	}
#if 1
	/*set column*/
	cmd = set_column_address;
	param[0] = x1 >> 8;
	param[1] = x1;
	param[2] = x2 >> 8;
	param[3] = x2;

	err = mdfld_dsi_send_dcs(sender,
				 cmd,
				 param,
				 4,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);
	if(err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		goto err_out;
	}

	/*set page*/
	cmd = set_page_addr;
	param[0] = y1 >> 8;
	param[1] = y1;
	param[2] = y2 >> 8;
	param[3] = y2;

	err = mdfld_dsi_send_dcs(sender,
				 cmd,
				 param,
				 4,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);
	if(err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		goto err_out;
	}
#else
	u32 sc1_set_column_address[] = {0x0200002a, 0x0000001b};
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_column_address, 8, 0);


	u32 sc1_set_page_address[] = {0x0300002b, 0x000000bf};
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_page_address, 8, 0);
#endif

	/* mdelay(100); */

	/*err = mdfld_dsi_dbi_send_dcs(dbi_output, set_column_address, param,
		4, CMD_DATA_SRC_SYSTEM_MEM);
	if (err) {
		DRM_ERROR("%s - sent write_mem_start faild\n", __func__);
		goto err_out;
	}
	err = mdfld_dsi_dbi_cb_ready(dbi_output);

	if (err)
	{
		printk(KERN_ALERT "[DISPLAY] Enter %s, Timeout waiting for"
			"Command complete on pipe\n", __func__);
		goto err_out;
	}

	err = mdfld_dsi_dbi_send_dcs(dbi_output, set_page_addr, param,
		4, CMD_DATA_SRC_SYSTEM_MEM);
	if(err) {
		DRM_ERROR("%s - sent write_mem_start faild\n", __func__);
		goto err_out;
	}*/
	/*err = mdfld_dsi_dbi_cb_ready(dbi_output);
	if(err)
	{
		printk(KERN_ALERT "[DISPLAY] Enter %s, Timeout waiting for"
			"Command complete on pipe\n", __func__);
		goto err_out;
	}*/
	/*update screen*/
	err = mdfld_dsi_send_dcs(sender,
				 write_mem_start,
				 NULL,
				 0,
				 CMD_DATA_SRC_PIPE,
				 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		goto err_out;
        }

        mdfld_dsi_cmds_kick_out(sender);
err_out:
        return err;
}

/**
 * set panel's power state
 */ 
int mdfld_dsi_dbi_update_power(struct mdfld_dsi_dbi_output * dbi_output, int mode)
{
	struct drm_device * dev = dbi_output->dev;
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct mdfld_dsi_pkg_sender * sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	u8 param[4];
	u32 err = 0;

	PSB_DEBUG_ENTRY("\n");

	if(!sender) {
		DRM_ERROR("Cannot get PKG sender\n");
		return -EINVAL;
	}
	
	if(mode == DRM_MODE_DPMS_ON) {
		param[0] = 0x00;
		param[1] = 0x00;
		param[2] = 0x00;

		/* set display on */
		err = mdfld_dsi_send_dcs(sender,
				set_display_on,
				param,
				3,
				CMD_DATA_SRC_SYSTEM_MEM,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("DCS 0x%x sent failed\n", set_display_on);
			goto power_err;
		}
	} else {
		param[0] = 0x00;
		param[1] = 0x00;
		param[2] = 0x00;

		/* set display off */
		err = mdfld_dsi_send_dcs(sender,
				set_display_off,
				param,
				3,
				CMD_DATA_SRC_SYSTEM_MEM,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("DCS 0x%x sent failed\n", set_display_off);
			goto power_err;
		}
		mdelay(70);
	}
	
	mdfld_dsi_cmds_kick_out(sender);

power_err:
	return err;
}

/**
 * send a generic DCS command with a parameter list
 */ 
int mdfld_dsi_dbi_send_dcs(struct mdfld_dsi_dbi_output * dbi_output, 
					 u8 dcs, 
					 u8 * param, 
					 u32 num, 
					 u8 data_src)
{
	struct mdfld_dsi_pkg_sender * sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	int ret;

	if(!sender) {
		DRM_ERROR("Cannot get PKG sender\n");
		return -EINVAL;
	}
	
	ret = mdfld_dsi_send_dcs(sender,
				 dcs,
				 param,
				 num,
				 data_src,
				 MDFLD_DSI_QUEUE_PACKAGE);

	return ret;
}


/**
 * Enter DSR 
 */
void mdfld_dsi_dbi_enter_dsr (struct mdfld_dsi_dbi_output * dbi_output, int pipe)
{
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	struct drm_crtc *crtc = dbi_output->base.base.crtc;
	struct psb_intel_crtc *psb_crtc =
		(crtc) ? to_psb_intel_crtc(crtc) : NULL;

	int retry;
	uint8_t panel_desc = 0;
	struct panel_funcs *p_funcs  = NULL;
	struct mdfld_dsi_pkg_sender *sender = NULL;

	if (!dbi_output)
		return;

	if (!dev_priv->um_start)
		return;

	gdbi_output = dbi_output;
	if ((dbi_output->mode_flags & MODE_SETTING_IN_DSR) ||
		(dbi_output->mode_flags & MODE_SETTING_ON_GOING) ||
		(psb_crtc && psb_crtc->mode_flags & MODE_SETTING_ON_GOING) ||
		(dev_priv && dev_priv->bhdmiconnected))
		goto fun_exit;

	PSB_DEBUG_ENTRY("-->\n");

	sender = mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	retry = 100;
	while (retry && !(REG_READ(sender->mipi_gen_fifo_stat_reg) & BIT27)) {
		udelay(500);
		retry--;
	}

	/*if DBI FIFO timeout, do not enter dsr*/
	if (!retry) {
		DRM_INFO("can not enter dsr currently\n");
		goto fun_exit ;
	}

	/*disable the te*/
	mdfld_disable_te(dev, pipe);

	if (!ospm_enter_dsr(dbi_output, pipe)) {
		mdfld_enable_te(dev, pipe);
		return ;
	}

	/*update mode state to IN_DSR*/
	dbi_output->mode_flags |= MODE_SETTING_IN_DSR;

	if (pipe == 2)
		enter_dsr = 1;

	PSB_DEBUG_ENTRY("<--\n");

fun_exit:
	return;
}

#ifndef CONFIG_MDFLD_DSI_DPU
static void mdfld_dbi_output_exit_dsr(struct mdfld_dsi_dbi_output *dbi_output,
	int pipe, void *p_surfaceAddr, bool check_hw_on_only)
{
	struct drm_device * dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	struct drm_crtc *crtc = dbi_output->base.base.crtc;
	struct psb_intel_crtc *psb_crtc =
		(crtc) ? to_psb_intel_crtc(crtc) : NULL;

	uint8_t panel_desc = 0;

	if (!dbi_output)
		return;

	/*if mode setting on-going, back off*/
	if (!(dbi_output->mode_flags & MODE_SETTING_IN_DSR) ||
		(dbi_output->mode_flags & MODE_SETTING_ON_GOING) ||
		(psb_crtc && psb_crtc->mode_flags & MODE_SETTING_ON_GOING))
		return;

	PSB_DEBUG_ENTRY("-->\n");

	ospm_exit_dsr(dev);

	/*clean IN_DSR flag*/
	dbi_output->mode_flags &= ~MODE_SETTING_IN_DSR;

	PSB_DEBUG_ENTRY("<--\n");
}

int mdfld_dsi_dbi_async_check_fifo_empty(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_pkg_sender *sender = NULL;
	int err = 0;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = 0 ? dbi_outputs[1] : dbi_outputs[0];
	if (!dbi_output)
		return 0;

	sender = mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);

	err = mdfld_dsi_check_fifo_empty(sender);
	return err;
}
/*
* use hw te to update fb
*/
int mdfld_dsi_dbi_async_flip_fb_update(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_pkg_sender *sender = NULL;
	int ret = IMG_TRUE;
	int err = 0;
	u32 damage_mask = 0;

	u32 dsplinoff_reg = DSPALINOFF;
	u32 dspsurf_reg = DSPASURF;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = pipe ? dbi_outputs[1] : dbi_outputs[0];
	if (!dbi_output)
		return IMG_FALSE;

	if (dbi_output->mode_flags & MODE_SETTING_IN_DSR) {
		DRM_ERROR("Display Controller in DSR\n");
		ret = IMG_FALSE;
		goto fun_exit;
	}
	if (pipe == 0)
		damage_mask = dev_priv->dsr_fb_update & MDFLD_DSR_DAMAGE_MASK_0;
	else if (pipe == 2)
		damage_mask = dev_priv->dsr_fb_update & MDFLD_DSR_DAMAGE_MASK_2;

	if (damage_mask) {
		sender = mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);

		/* refresh plane changes */
		REG_WRITE(dsplinoff_reg, REG_READ(dsplinoff_reg));
		REG_WRITE(dspsurf_reg, REG_READ(dspsurf_reg));
		REG_READ(dspsurf_reg);

		err = mdfld_dsi_send_dcs(sender,
				write_mem_start,
				NULL,
				0,
				CMD_DATA_SRC_PIPE,
				MDFLD_DSI_SEND_PACKAGE);

		if (err) {
			DRM_ERROR(
			"Error returned from mdfld_dsi_send_dcs: %d\n", ret);
			ret = IMG_FALSE;
			goto fun_exit;
		}
		dev_priv->dsr_fb_update &= ~damage_mask;
	}
fun_exit:
	return ret;
}

/**
 * Exit from DSR 
 */
void mdfld_dsi_dbi_exit_dsr (struct drm_device *dev, u32 update_src, void *p_surfaceAddr, bool check_hw_on_only)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info * dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output ** dbi_output;
	int i;

	mutex_lock(&dev_priv->dsr_mutex);

	dbi_output = dsr_info->dbi_outputs;

#ifdef CONFIG_PM_RUNTIME
	 if(drm_psb_ospm && !enable_gfx_rtpm) {
//                pm_runtime_allow(&gpDrmDevice->pdev->dev);
//		schedule_delayed_work(&rtpm_work, 120 * 1000);
        }
#endif

	/*for each output, exit dsr*/
	for(i=0; i<dsr_info->dbi_output_num; i++) {
		/*if panel has been turned off, skip*/
		if (!dbi_output[i] || !dbi_output[i]->dbi_panel_on)
			continue;
		if(dbi_output[i]->mode_flags & MODE_SETTING_IN_DSR) {
			enter_dsr = 0;
			mdfld_dbi_output_exit_dsr(dbi_output[i], dbi_output[i]->channel_num ? 2 : 0, p_surfaceAddr, check_hw_on_only);
		}
	}
	
	dev_priv->dsr_fb_update |= update_src;
	dev_priv->dsr_idle_count = 0;
	/*start timer if A0 board*/
	if ((get_panel_type(dev, 0) == GI_SONY_CMD)||(get_panel_type(dev, 0) == H8C7_CMD))
		;  /* mdfld_dbi_dsr_timer_start(dsr_info); */
	else if (dev_priv->platform_rev_id == MDFLD_PNW_A0)
		mdfld_dbi_dsr_timer_start(dsr_info);

	mutex_unlock(&dev_priv->dsr_mutex);
}

static bool mdfld_dbi_is_in_dsr(struct drm_device * dev)
{
	if(REG_READ(MRST_DPLL_A) & DPLL_VCO_ENABLE)
		return false;
	if((REG_READ(PIPEACONF) & PIPEACONF_ENABLE) ||
	   (REG_READ(PIPECCONF) & PIPEACONF_ENABLE))
		return false;
	if((REG_READ(DSPACNTR) & DISPLAY_PLANE_ENABLE) ||
	   (REG_READ(DSPCCNTR) & DISPLAY_PLANE_ENABLE))
		return false;

	return true;
}

/* Perodically update dbi panel */
void mdfld_dbi_update_panel (struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs;
	struct mdfld_dsi_dbi_output *dbi_output;
	int i;
	u32 damage_mask = 0;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = pipe ? dbi_outputs[1] : dbi_outputs[0];

	if (!dbi_output)
		return;

	mutex_lock(&dev_priv->dsr_mutex);

	if (pipe == 0)
		damage_mask = dev_priv->dsr_fb_update & MDFLD_DSR_DAMAGE_MASK_0;
	else if (pipe == 2)
		damage_mask = dev_priv->dsr_fb_update & MDFLD_DSR_DAMAGE_MASK_2;

	/*if FB is damaged and panel is on update on-panel FB*/
	if (damage_mask && dbi_output->dbi_panel_on) {
		dbi_output->dsr_fb_update_done = false;

		if (dbi_output->p_funcs->update_fb)
			dbi_output->p_funcs->update_fb(dbi_output, pipe);

		if (dev_priv->b_dsr_enable && dbi_output->dsr_fb_update_done)
			dev_priv->dsr_fb_update &= ~damage_mask;

		dbi_output->dsr_idle_count = 0;
	} else {
		dbi_output->dsr_idle_count++;
	}

	/*try to enter DSR*/
	if (dbi_outputs[0]->dsr_idle_count >= MAX_IDLE_COUNT_FOR_DSR) {
		/* && dbi_outputs[1]->dsr_idle_count > 1) { */
		for(i=0; i<dsr_info->dbi_output_num; i++) {
			if (!mdfld_dbi_is_in_dsr(dev) && dbi_outputs[i] &&
					!(dbi_outputs[i]->mode_flags &
						MODE_SETTING_ON_GOING)) {
				mdfld_dsi_dbi_enter_dsr(dbi_outputs[i],
						dbi_outputs[i]->channel_num ?
						2 : 0);
			}
		}
		/*schedule rpm suspend after gfxrtdelay*/
#ifdef CONFIG_GFX_RTPM
		if(!dev_priv->rpm_enabled
			|| !enter_dsr
			/*|| (REG_READ(HDMIB_CONTROL) & HDMIB_PORT_EN) */
			|| pm_schedule_suspend(&dev->pdev->dev, gfxrtdelay))
			PSB_DEBUG_ENTRY("Runtime PM schedule suspend failed, rpm %d\n", dev_priv->rpm_enabled);
#endif
	}

	mutex_unlock(&dev_priv->dsr_mutex);

}

#ifdef CONFIG_DRM_PANEL_REG_ACCESS
int mdfld_dsi_dbi_panel_reg_read(struct drm_device *dev,
				u8 address, u16 size, u8 *buf, u8 use_hs_mode)
{
	int r = 0;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output *dbi_output = dsr_info->dbi_outputs[0];
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);

	u32 data[8];
	u16 u32_len;

	if (size > sizeof(data))
		return -EINVAL;

	u32_len = size / 4;
	if (size % 4)
		u32_len++;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
		if (use_hs_mode)
			r = mdfld_dsi_read_mcs_hs(sender, address,
						&data, u32_len);
		else
			r = mdfld_dsi_read_mcs_lp(sender, address,
						&data, u32_len);

		if (r == u32_len) {
			r = 0;
			memcpy(buf, data, size);
		} else if ((r != u32_len) && (r >= 0))
			r = -EFAULT;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		r = -EAGAIN;
		DRM_ERROR("Failed to turn on display island\n");
	}

	return r;
}

int mdfld_dsi_dbi_panel_reg_write(struct drm_device *dev,
				u8 address, u16 size, u8 *buf, u8 use_hs_mode)
{
	int r = 0;
	u8 *data;
	u32 complete_size;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output *dbi_output = dsr_info->dbi_outputs[0];
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);

	/* Combine address and buf into continous memory */
	complete_size = size + 1;
	data = kmalloc(complete_size, GFP_KERNEL);
	if (!data) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	data[0] = address;
	memcpy(&data[1], buf, size);

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		if (use_hs_mode)
			r = mdfld_dsi_send_mcs_long_hs(sender, data,
						complete_size,
						MDFLD_DSI_SEND_PACKAGE);
		else
			r = mdfld_dsi_send_mcs_long_lp(sender, data,
						complete_size,
						MDFLD_DSI_SEND_PACKAGE);

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		r = -EAGAIN;
		DRM_ERROR("Failed to turn on display island\n");
	}

	kfree(data);
	return r;
}
#endif

/*timers for DSR*/
static void mdfld_dsi_dbi_dsr_timer_func(unsigned long data)
{
	struct drm_device * dev = (struct drm_device *)data;
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info * dsr_info = dev_priv->dbi_dsr_info;
	struct timer_list * dsr_timer = &dsr_info->dsr_timer;
	unsigned long flags;

	mdfld_dbi_update_panel (dev, 0);

	drm_handle_vblank(dev, 0);

	if (dev_priv->psb_vsync_handler != NULL)
		(*dev_priv->psb_vsync_handler)(dev, 0);

	if (dsr_info->dsr_idle_count > 1)
		return;

	spin_lock_irqsave(&dsr_info->dsr_timer_lock, flags);
	if(!timer_pending(dsr_timer)){
		dsr_timer->expires = jiffies + MDFLD_DSR_DELAY;
		add_timer(dsr_timer);
	}
	spin_unlock_irqrestore(&dsr_info->dsr_timer_lock, flags);
}

static int mdfld_dsi_dbi_dsr_timer_init(struct drm_device * dev)
{
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info * dsr_info = dev_priv->dbi_dsr_info;
	struct timer_list * dsr_timer = &dsr_info->dsr_timer;
	unsigned long flags;
		
	PSB_DEBUG_ENTRY("\n");

	spin_lock_init(&dsr_info->dsr_timer_lock);
	spin_lock_irqsave(&dsr_info->dsr_timer_lock, flags);
	
	init_timer(dsr_timer);
	
	dsr_timer->data = (unsigned long)dev;
	dsr_timer->function = mdfld_dsi_dbi_dsr_timer_func;
	dsr_timer->expires = jiffies + MDFLD_DSR_DELAY;
	
	spin_unlock_irqrestore(&dsr_info->dsr_timer_lock, flags);
	
	PSB_DEBUG_ENTRY("successfully\n");
	
	return 0;
}

void mdfld_dbi_dsr_timer_start(struct mdfld_dbi_dsr_info * dsr_info)
{
	struct timer_list * dsr_timer = &dsr_info->dsr_timer;
	unsigned long flags;
	
	spin_lock_irqsave(&dsr_info->dsr_timer_lock, flags);
	if(!timer_pending(dsr_timer)){
		dsr_timer->expires = jiffies + MDFLD_DSR_DELAY;
		add_timer(dsr_timer);
	}
	spin_unlock_irqrestore(&dsr_info->dsr_timer_lock, flags);
}

int mdfld_dbi_dsr_init(struct drm_device * dev) 
{
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info * dsr_info = dev_priv->dbi_dsr_info;
	
	if(!dsr_info || IS_ERR(dsr_info)) {
		dsr_info = kzalloc(sizeof(struct mdfld_dbi_dsr_info), GFP_KERNEL);
		if(!dsr_info) {
			DRM_ERROR("No memory\n");
			return -ENOMEM;
		}
		
		dev_priv->dbi_dsr_info = dsr_info;
	}

	if (get_panel_type(dev, 0) == GI_SONY_CMD)
		mdfld_dsi_dbi_dsr_timer_init(dev);
	else if (dev_priv->platform_rev_id == MDFLD_PNW_A0)
		/*init dsr refresh timer*/
		mdfld_dsi_dbi_dsr_timer_init(dev);
	
	PSB_DEBUG_ENTRY("successfully\n");
	
	return 0;
}

void mdfld_dbi_dsr_exit(struct drm_device * dev)
{
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info * dsr_info = dev_priv->dbi_dsr_info;
	
	if(!dsr_info) {
		return;
	}
	
	/*delete dsr timer*/
	del_timer_sync(&dsr_info->dsr_timer);
	
	/*free dsr info*/
	kfree(dsr_info);
	
	dev_priv->dbi_dsr_info = NULL;
}
#endif

void mdfld_dsi_controller_dbi_init(struct mdfld_dsi_config * dsi_config, int pipe)
{
	struct drm_device * dev = dsi_config->dev;

	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	uint32_t dpll = 0;

	struct panel_funcs *p_funcs = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_encoder *encoder = NULL;

	PSB_DEBUG_ENTRY("pipe[%d]\n", pipe);

	if (dsi_config == NULL)
		return;

	/*Initialize the dsi controller*/
	encoder = dsi_config->encoders[MDFLD_DSI_ENCODER_DBI];
	if (encoder != NULL) {
		dbi_output = MDFLD_DSI_DBI_OUTPUT(encoder);
		if (dbi_output != NULL)
			p_funcs = dbi_output->p_funcs;
		if (p_funcs && p_funcs->dsi_controller_init &&
			(dbi_output->mode_flags & MODE_SETTING_IN_DSR))
			p_funcs->dsi_controller_init(dsi_config, pipe, 1);
	}

#if 0
	/*un-ready device*/
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000000);
	
	REG_WRITE(0x61190, 0x80810006);
	
	/*TODO: figure out how to setup these registers*/
	REG_WRITE((MIPIA_DPHY_PARAM_REG + reg_offset), 0x150c3408);
	REG_WRITE((MIPIA_CLK_LANE_SWITCH_TIME_CNT_REG + reg_offset), 0x000a0014);
	REG_WRITE((MIPIA_DBI_BW_CTRL_REG + reg_offset), 0x00000400);
	REG_WRITE((MIPIA_DBI_FIFO_THROTTLE_REG + reg_offset), 0x00000001);
	REG_WRITE((MIPIA_HS_LS_DBI_ENABLE_REG + reg_offset), 0x00000000);
	
	/*enable all interrupts*/
	REG_WRITE((MIPIA_INTR_EN_REG + reg_offset), 0xffffffff);
	/*max value: 20 clock cycles of txclkesc*/
	REG_WRITE((MIPIA_TURN_AROUND_TIMEOUT_REG + reg_offset), 0x0000001f);
	/*min 21 txclkesc, max: ffffh*/
	REG_WRITE((MIPIA_DEVICE_RESET_TIMER_REG + reg_offset), 0x0000ffff);
	/*min: 7d0 max: 4e20*/
	REG_WRITE((MIPIA_INIT_COUNT_REG + reg_offset), 0x00000fa0);
		
	/*set up func_prg*/
	val |= lane_count;
	val |= (dsi_config->channel_num << DSI_DBI_VIRT_CHANNEL_OFFSET);
	val |= DSI_DBI_COLOR_FORMAT_OPTION2;
	REG_WRITE((MIPIA_DSI_FUNC_PRG_REG + reg_offset), val);
	
	REG_WRITE((MIPIA_HS_TX_TIMEOUT_REG + reg_offset), 0x3fffff);
	REG_WRITE((MIPIA_LP_RX_TIMEOUT_REG + reg_offset), 0xffff);

	/*de-assert dbi_stall when half of DBI FIFO is empty*/
	//REG_WRITE((MIPIA_DBI_FIFO_THROTTLE_REG + reg_offset), 0x00000000);
	
	REG_WRITE((MIPIA_HIGH_LOW_SWITCH_COUNT_REG + reg_offset), 0x46);
	REG_WRITE((MIPIA_EOT_DISABLE_REG + reg_offset), 0x00000000);
	REG_WRITE((MIPIA_LP_BYTECLK_REG + reg_offset), 0x00000004);
	REG_WRITE((MIPIA_DEVICE_READY_REG + reg_offset), 0x00000001);
#endif
}

/*
 * Init DSI DBI encoder. 
 * Allocate an mdfld_dsi_encoder and attach it to given @dsi_connector
 * return pointer of newly allocated DBI encoder, NULL on error
 */ 
struct mdfld_dsi_encoder *mdfld_dsi_dbi_init(struct drm_device *dev,
			struct mdfld_dsi_connector *dsi_connector,
			struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL;
	struct drm_display_mode *fixed_mode = NULL;
	struct psb_gtt *pg = dev_priv ? (dev_priv->pg) : NULL;

#ifdef CONFIG_MDFLD_DSI_DPU
	struct mdfld_dbi_dpu_info *dpu_info =
		dev_priv ? (dev_priv->dbi_dpu_info) : NULL;
#else
	struct mdfld_dbi_dsr_info *dsr_info =
		dev_priv ? (dev_priv->dbi_dsr_info) : NULL;
#endif	
	int pipe;
	int ret;

	PSB_DEBUG_ENTRY("\n");

	if (!pg || !dsi_connector || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return NULL;
	}

	dsi_config = mdfld_dsi_get_config(dsi_connector);
	pipe = dsi_connector->pipe;

	/*panel hard-reset*/
	if (p_funcs->reset) {
		/* ret = p_funcs->reset(dsi_config, pipe); */
		ret = p_funcs->reset(dsi_config, RESET_FROM_BOOT_UP);
		if (ret) {
			DRM_ERROR("Panel %d hard-reset failed\n", pipe);
			return NULL;
		}
	}


/* FIXME JLIU7 */
#if 0
	/*panel drvIC init*/
	if (p_funcs->drv_ic_init)
		p_funcs->drv_ic_init(dsi_config, pipe);
#endif

	/*detect panel connection stauts*/
	if (p_funcs->detect) {
		ret = p_funcs->detect(dsi_config, pipe);
		if (ret) {
			PSB_DEBUG_ENTRY("Fail to detect Panel %d\n",
					pipe);
			dsi_connector->status =
				connector_status_disconnected;
		} else {
			PSB_DEBUG_ENTRY("Panel %d is connected\n",
					pipe);
			dsi_connector->status =
				connector_status_connected;
		}
	} else {
		/*use the default config*/
		if (pipe == 0)
			dsi_connector->status =
				connector_status_connected;
		else
			dsi_connector->status =
				connector_status_disconnected;
	}

	/*init DSI controller*/
	if (p_funcs->dsi_controller_init) {
		if (!is_panel_vid_or_cmd(dev))
			p_funcs->dsi_controller_init(dsi_config, pipe, false);
		else
			p_funcs->dsi_controller_init(dsi_config, pipe, true);
	}
	if (dsi_connector->status == connector_status_connected) {
		if (pipe == 0)
			dev_priv->panel_desc |= DISPLAY_A;
		if (pipe == 2)
			dev_priv->panel_desc |= DISPLAY_C;
	}
	/* mdfld_dsi_controller_dbi_init(dsi_config, pipe); */

	/* TODO: get panel info from DDB */
	dbi_output = kzalloc(sizeof(struct mdfld_dsi_dbi_output), GFP_KERNEL);
	if(!dbi_output) {
		DRM_ERROR("No memory\n");
		return NULL;
	}

	if(dsi_connector->pipe == 0) {
		dbi_output->channel_num = 0;
		dev_priv->dbi_output = dbi_output;
	} else if (dsi_connector->pipe == 2) {
		dbi_output->channel_num = 1;
		dev_priv->dbi_output2 = dbi_output;
	} else {
		DRM_ERROR("only support 2 DSI outputs\n");
		goto out_err1;
	}
	
	dbi_output->dev = dev;
	dbi_output->p_funcs = p_funcs;

	/*get fixed mode*/
	fixed_mode = dsi_config->fixed_mode;

	dbi_output->panel_fixed_mode = fixed_mode;

	/*create drm encoder object*/
	connector = &dsi_connector->base.base;
	encoder = &dbi_output->base.base;
	drm_encoder_init(dev,
			encoder,
			p_funcs->encoder_funcs,
			DRM_MODE_ENCODER_MIPI);
	drm_encoder_helper_add( encoder,
				p_funcs->encoder_helper_funcs);

	/*attach to given connector*/
	drm_mode_connector_attach_encoder(connector, encoder);

	/*set possible crtcs and clones*/
	if(dsi_connector->pipe) {
		encoder->possible_crtcs = (1 << 2);
		encoder->possible_clones = (1 << 1);
	} else {
		encoder->possible_crtcs = (1 << 0);
		encoder->possible_clones = (1 << 0);
	}

	dev_priv->dsr_fb_update = 0;
	dev_priv->b_dsr_enable = false;
	dev_priv->b_async_flip_enable = false;
	dev_priv->exit_idle = mdfld_dsi_dbi_exit_dsr;
	dev_priv->async_flip_update_fb = mdfld_dsi_dbi_async_flip_fb_update;
	dev_priv->async_check_fifo_empty = mdfld_dsi_dbi_async_check_fifo_empty;
#if defined(CONFIG_MDFLD_DSI_DPU) || defined(CONFIG_MDFLD_DSI_DSR)
	dev_priv->b_dsr_enable_config = true;
#endif /*CONFIG_MDFLD_DSI_DSR*/

	dbi_output->first_boot = true;
	dbi_output->mode_flags = MODE_SETTING_IN_ENCODER;

#ifdef CONFIG_MDFLD_DSI_DPU
	/*add this output to dpu_info*/

	if (dsi_connector->status == connector_status_connected) {
		if (dsi_connector->pipe == 0)
			dpu_info->dbi_outputs[0] = dbi_output;
		else
			dpu_info->dbi_outputs[1] = dbi_output;

		dpu_info->dbi_output_num++;
	}

#else /*CONFIG_MDFLD_DSI_DPU*/	
	if (dsi_connector->status == connector_status_connected) {
		/*add this output to dsr_info*/
		if (dsi_connector->pipe == 0)
			dsr_info->dbi_outputs[0] = dbi_output;
		else
			dsr_info->dbi_outputs[1] = dbi_output;

		dsr_info->dbi_output_num++;
	}
#endif

	PSB_DEBUG_ENTRY("successfully\n");

	return &dbi_output->base;
	
out_err1: 
	if(dbi_output) {
		kfree(dbi_output);
	}

	return NULL;
}
