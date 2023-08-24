# FR3 Pose Publisher

## Compile `read_fr3_state`

Use the following steps to compile `read_fr3_state`

```bash
mkdir build
cd build
cmake .. --DFranka_DIR:PATH=/path/to/libfranka/build
cmake --build .
cmake --install .
```

The add `libfranka.so.0.10` and `read_fr3_state.cpython-39-x86_64-linux-gnu.so` to your `LD_LIBRARY_PATH` (you could also add this to your `.bashrc` or `.zshrc`), **don't forget to replace `/path/to` with your actual path**

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/libfranka/build
export PYTHONPATH=$PYTHONPATH:/path/to/fr3_pose_publisher/install
```