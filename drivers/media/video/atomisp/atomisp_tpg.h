/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __TPG_H__
#define __TPG_H__

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

struct atomisp_tpg_device {
	struct v4l2_subdev sd;
	struct atomisp_device *isp;
	struct media_pad pads[1];
};

void atomisp_tpg_cleanup(struct atomisp_device *isp);
int atomisp_tpg_init(struct atomisp_device *isp);
void atomisp_tpg_unregister_entities(struct atomisp_tpg_device *tpg);
int atomisp_tpg_register_entities(struct atomisp_tpg_device *tpg,
			struct v4l2_device *vdev);

#endif
