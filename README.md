

A ros node for rendering with bgfx: https://github.com/bkaradzic/bgfx

Rendered images are displayed via an Image topic.

Control of camera position will be via tf or Pose topics,
no direct mouse control except in upstream nodes that can provide mouse control over tf or Poses.

`visualization_msgs Marker` topics and perhaps other geometry messages will be supported.

`Marker` display via service calls will also be supported.

Lower level than rviz, ogre, or gazebo rendering.
Dynamic reconfigure and service call will be able to change graphics settings.

Perhaps support shaders via `String` messages that are then compiled and updated for live coding.


# Install

Build bgfx in ~/other/bgfx:

```
mkdir ~/other
cd ~/other
git clone https://github.com/bkaradzic/bgfx.git
git clone https://github.com/bkaradzic/bx.git
cd bgfx
make linux-release64
make linux-debug64
```


Libraries are in `bgfx/.build/linux64_gcc/bin`

~/other/install/lib is already on `LD_LIBRARY_PATH`, so symlink:

```
  cd ~/other/install/lib
  ln -s ../../bgfx/.build/linux64_gcc/bin/libbgfx-shared-libDebug.so libbgfx-shared-libDebug.so
  ln -s ../../bgfx/.build/linux64_gcc/bin/libbgfx-shared-libRelease.so libbgfx-shared-libRelease.so
```

Now can run a.out (test application without ros).

# Use graphics card

On amd laptop, can switch to graphics card like this instead of using intel::

  DRI_PRIME=1 rosrun bgfx_ros bgfx_ros __name:=bgfx_ros
