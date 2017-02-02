
Currently have bgfx built in ~/other/bgfx

Libraries are in `bgfx/.build/linux64_gcc/bin`

~/other/install/lib is already on `LD_LIBRARY_PATH`, so symlink:

```
  cd ~/other/install/lib
  ln -s ../../bgfx/.build/linux64_gcc/bin/libbgfx-shared-libDebug.so libbgfx-shared-libDebug.so
  ln -s ../../bgfx/.build/linux64_gcc/bin/libbgfx-shared-libRelease.so libbgfx-shared-libRelease.so
```

Now can run a.out (test application without ros).
