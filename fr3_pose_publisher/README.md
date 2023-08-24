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