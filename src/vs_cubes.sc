$input a_position, a_normal, a_color0
$output v_normal, v_color0

/*
 * Copyright 2011-2016 Branimir Karadzic. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
 */

#include <bgfx_shader.sh>

void main()
{
	gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
  v_normal = mul(u_modelView, a_normal).xyz;
	v_color0 = a_color0;
}
