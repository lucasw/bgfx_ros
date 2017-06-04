$input v_view, v_normal, v_shadowcoord, v_color0

/*
 * Copyright 2013-2014 Dario Manesku. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
 */

#include <bgfx_shader.sh>

#define SHADOW_PACKED_DEPTH 1
#include "fs_sms_shadow.sh"
