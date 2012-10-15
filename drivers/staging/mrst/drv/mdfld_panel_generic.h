#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"

int mdfld_panel_generic_dsi_dbi_power_on(struct drm_encoder *encoder);
int mdfld_panel_generic_dsi_dbi_power_off(struct drm_encoder *encoder);
void mdfld_panel_generic_dsi_dbi_dpms(struct drm_encoder *encoder, int mode);
int mdfld_panel_generic_dsi_dbi_set_power(struct drm_encoder *encoder, bool on);
void mdfld_panel_generic_dsi_dbi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode);
void mdfld_panel_generic_dsi_dbi_prepare(struct drm_encoder *encoder);
void mdfld_panel_generic_dsi_dbi_commit(struct drm_encoder *encoder);
void mdfld_panel_generic_dsi_dbi_save(struct drm_encoder *encoder);
void mdfld_panel_generic_dsi_dbi_update_fb(
	struct mdfld_dsi_dbi_output *dbi_output,
	int pipe);
