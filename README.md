```
git clone https://github.com/rl-tools/foundation-policy.git
cd foundation-policy
git submodule update --init rl-tools
cd rl-tools
git submodule update --init --recursive -- external/highfive external/json external/tensorboard
cd ..
```

```
cat data/foundation-policy-v1-data.tar.gz.part_* > data/foundation-policy-v1-data.tar.gz
cd rl-tools
tar -xvf ../data/foundation-policy-v1-data.tar.gz
cd ..
```

The following can be skipped because the image is also available the Docker Hub and will be automatically downloaded by `docker run` we just include it here for complete reproducibility.
```
cd rl-tools/docker
./build_all.sh
cd ../../
```

```
docker run -it --rm -v $(pwd)/rl-tools:/rl-tools -v data:/data rltools/rltools:ubuntu24.04_mkl_gcc_base
```

```
mkdir build
MKL_ROOT=/opt/intel/oneapi/mkl/latest cmake -S /rl-tools -B /build -DCMAKE_BUILD_TYPE=Release -DRL_TOOLS_BACKEND_ENABLE_MKL=ON -DRL_TOOLS_ENABLE_TARGETS=ON -DRL_TOOLS_EXPERIMENTAL=ON -DRL_TOOLS_ENABLE_HDF5=ON -DRL_TOOLS_ENABLE_JSON=ON -DRL_TOOLS_ENABLE_TENSORBOARD=ON
cmake --build /build --target foundation_policy_pre_training_sample_dynamics_parameters --target foundation_policy_pre_training --target foundation_policy_post_training -j$(nproc)
cd /rl-tools
export RL_TOOLS_EXTRACK_EXPERIMENT=$(date '+%Y-%m-%d_%H-%M-%S')
echo "Experiment: $RL_TOOLS_EXTRACK_EXPERIMENT"
/build/src/foundation_policy/foundation_policy_pre_training_sample_dynamics_parameters
seq 0 999 | xargs -I {} /build/src/foundation_policy/foundation_policy_pre_training ./src/foundation_policy/dynamics_parameters/{}.json
/build/src/foundation_policy/foundation_policy_post_training
```

`foundation_policy_post_training` uses the teacher checkpoints from `foundation-policy-v0.1-data` by default. If you want to use the teacher checkpoints trained by `foundation_policy_pre_training`:
```
cd rl-tools/src/foundation_policy
./extract_checkpoints.sh > checkpoints_$RL_TOOLS_EXTRACK_EXPERIMENT.txt
```
Then change the experiment name to the content of `$RL_TOOLS_EXTRACK_EXPERIMENT` in `post_training/main.cpp` and the experiment directory from `1k-experiments` to `experiments` such that it will find the newly trained checkpoints referred to by the `checkpoint_xxx.txt`.


### ELRS forwarding

Use e.g. Radiomaster Ranger Micro. (This is tested with `3.5.5`, flash using the ExpressLRS Configurator):
1. Connect to the WiFi hosted by it
2. Go to 10.0.0.1
3. Set the passphrase to match the one on your desired model
4. Set the same passphrase in betaflight by copying the 6 numbers from the site: `set expresslrs_uid = {6 numbers}` then `save` (both in the CLI)
5. Go to 10.0.0.1/hardware.html
6. Disable the Backpack
7. Set `CSRF.RX=3` and `CSRF.TX=1` (this works for both Radiomaster Ranger Micro and 
Jumper Aion ELRS 2.4G TX Nano and is reported to work for other modules like the BetaFPV ones as well)
8. run `python choreo/elrs.py /dev/ttyUSB0 921600 --ch 1337` (replace path with the assigned port on you PC). This should send a `1337` value to the receiver. Note that this is sent in these ranges
```
RC_CHANNEL_MIN = 172
RC_CHANNEL_MID = 992
RC_CHANNEL_MAX = 1811
```
and should be normalized to 1000 - 2000 in betaflight
