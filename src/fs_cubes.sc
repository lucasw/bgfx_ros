$input v_normal, v_color0

/*
 * Copyright 2011-2016 Branimir Karadzic. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
 */

#include <bgfx_shader.sh>

uniform vec4 light_dir;

void main()
{
  // this is in eye coords, need to transform it into the worlk
  float ndotl = -dot(normalize(v_normal), light_dir.xyz);
  // TODO(lwalter)
  // float spec = pow(nodtl
	gl_FragColor = vec4(v_color0.xyz * ndotl, 1.0);
  if (gl_FragColor.x < 0.2)
    gl_FragColor.x = 0.2;
}
