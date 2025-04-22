```
cat data/foundation-policy-v0.1-data.tar.gz.part_* > data/foundation-policy-v0.1-data.tar.gz
```

```
cd rl-tools/docker
./build_all.sh
```

```
docker run -it --rm -v $(pwd)/rl-tools:/rl-tools -v data:/data rltools/rltools:ubuntu24.04_mkl_gcc_base
```

```
mkdir build
MKL_ROOT=/opt/intel/oneapi/mkl/latest cmake /rl-tools -B /build -DCMAKE_BUILD_TYPE=Release -DRL_TOOLS_BACKEND_ENABLE_MKL=ON -DRL_TOOLS_ENABLE_TARGETS=ON -DRL_TOOLS_EXPERIMENTAL=ON -DRL_TOOLS_ENABLE_HDF5=ON -DRL_TOOLS_ENABLE_JSON=ON -DRL_TOOLS_ENABLE_TENSORBOARD=ON && cmake --build /build --target foundation_policy_post_training
cd /rl-tools
/build/src/foundation_policy/foundation_policy_post_training
```