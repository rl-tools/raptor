<div align="center">
<img src="./media/logo.svg"></img>
</div>

# RAPTOR: A Foundation Policy for Quadrotor Control

## Usage
If you want to use your own simulator:
```bash
pip install foundation-policy==1.0.1
```
```python
from foundation_policy import Raptor
policy = Raptor()
policy.reset()
action = [0, 0, 0, 0]
for simulation_step in range(1000):
    observation = np.array([[*sim.position, *R(sim.orientation).flatten(), *sim.linear_velocity, *sim.angular_velocity, *sim.action]])
    action = policy.evaluate_step(observation)[0] # the policy works on batches by default
    simulation.step(action) # simulation dt=10 ms
```
Note that the axis conventions are FLU (x = forward, y = left, z = up). Please convert position, orientation, linear velocity and angular velocity into these conventions. The angular velocity is in the body frame. The action motor conventions are [front-right, back-right, back-left, front-left]. 


### Usage: L2F
The following instructions show how to use [l2f](https://github.com/rl-tools/l2f), the simulator used for training the foundation policy:
```bash
pip install l2f==2.0.18 ui-server==0.0.13 foundation-policy==1.0.1
```
Run `ui-server` in background and open [http://localhost:13337](http://localhost:13337)
```bash
ui-server
```
Then run the following code:
```python
from copy import copy
import numpy as np
import asyncio, websockets, json
import l2f
from l2f import vector8 as vector
from foundation_policy import Raptor

policy = Raptor()
device = l2f.Device()
rng = vector.VectorRng()
env = vector.VectorEnvironment()
ui = l2f.UI()
params = vector.VectorParameters()
state = vector.VectorState()
observation = np.zeros((env.N_ENVIRONMENTS, env.OBSERVATION_DIM), dtype=np.float32)
next_state = vector.VectorState()

vector.initialize_rng(device, rng, 0)
vector.initialize_environment(device, env)
vector.sample_initial_parameters(device, env, params, rng)
vector.sample_initial_state(device, env, params, state, rng)

def configure_3d_model(parameters_message):
    parameters_message = json.loads(parameters_message)
    for d in parameters_message["data"]:
        d["ui"] = {
            "model": "95d22881d444145176db6027d44ebd3a15e9699a",
            "name": "x500"
        }
    return json.dumps(parameters_message)

async def render(websocket, state, action):
    ui_state = copy(state)
    for i, s in enumerate(ui_state.states):
        s.position[0] += i * 0.1 # Spacing for visualization
    state_action_message = vector.set_state_action_message(device, env, params, ui, ui_state, action)
    await websocket.send(state_action_message)

async def main():
    uri = "ws://localhost:13337/backend" # connection to the UI server
    async with websockets.connect(uri) as websocket:
        handshake = json.loads(await websocket.recv(uri))
        assert(handshake["channel"] == "handshake")
        namespace = handshake["data"]["namespace"]
        ui.ns = namespace
        ui_message = vector.set_ui_message(device, env, ui)
        parameters_message = vector.set_parameters_message(device, env, params, ui)
        # parameters_message = configure_3d_model(parameters_message) # use this for a more realistic 3d model
        await websocket.send(ui_message)
        await websocket.send(parameters_message)
        await asyncio.sleep(1)
        await render(websocket, state, np.zeros((8, 4)))
        await asyncio.sleep(2)
        policy.reset()
        for _ in range(500):
            vector.observe(device, env, params, state, observation, rng)
            action = policy.evaluate_step(observation[:, :22])
            dts = vector.step(device, env, params, state, action, next_state, rng)
            state.assign(next_state)
            await render(websocket, state, action)
            await asyncio.sleep(dts[-1])

if __name__ == "__main__":
    asyncio.run(main())
```

## Deployment
For deployment onto flight controllers please refer to the reference implementations for PX4, Betaflight, Crazyflie and M5StampFly which are tracked in the [embedded_platforms](https://github.com/rl-tools/rl-tools/tree/3dea1bc877a8593dcd8349f6fdc4e362f025a0ca/embedded_platforms) folder.

The general idea is to use the provided adapter like [this](https://github.com/rl-tools/px4/blob/8350ec059af043dc794361b778794446808b2dea/external_modules/src/modules/rl_tools_policy/rl_tools_adapter.cpp):
```c++
#include <rl_tools/operations/arm.h>

#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/inference/executor/executor.h>

#include "blob/policy.h"

namespace rlt = rl_tools;

namespace other{
    using DEV_SPEC = rlt::devices::DefaultARMSpecification;
    using DEVICE = rlt::devices::arm::OPT<DEV_SPEC>;
}

struct RL_TOOLS_INFERENCE_APPLICATIONS_L2F_CONFIG{
    using DEVICE = other::DEVICE;
    using TI = typename other::DEVICE::index_t;
    using RNG = other::DEVICE::SPEC::RANDOM::ENGINE<>;
    static constexpr TI TEST_SEQUENCE_LENGTH_ACTUAL = 5;
    static constexpr TI TEST_BATCH_SIZE_ACTUAL = 2;
    using ACTOR_TYPE_ORIGINAL = rlt::checkpoint::actor::TYPE;
    using POLICY_TEST = rlt::checkpoint::actor::TYPE::template CHANGE_BATCH_SIZE<TI, 1>::template CHANGE_SEQUENCE_LENGTH<TI, 1>;
    using POLICY = ACTOR_TYPE_ORIGINAL::template CHANGE_BATCH_SIZE<TI, 1>;
    using T = typename POLICY::SPEC::T;
    static auto& policy() {
        return rlt::checkpoint::actor::module;
    }
    static constexpr TI ACTION_HISTORY_LENGTH = 1;
    static constexpr TI CONTROL_INTERVAL_INTERMEDIATE_NS = 2.5 * 1000 * 1000; // Inference is at 500hz
    static constexpr TI CONTROL_INTERVAL_NATIVE_NS = 10 * 1000 * 1000; // Training is 100hz
    static constexpr TI TIMING_STATS_NUM_STEPS = 100;
    static constexpr bool FORCE_SYNC_INTERMEDIATE = true;
    static constexpr TI FORCE_SYNC_NATIVE = 4;
    static constexpr bool DYNAMIC_ALLOCATION = false;
    using WARNING_LEVELS = rlt::inference::executor::WarningLevelsDefault<T>;
};

// #define RL_TOOLS_DISABLE_TEST
#include <rl_tools/inference/applications/l2f/c_backend.h>
```

Make sure to configure the control interval properly. The `CONTROL_INTERVAL_NATIVE_NS` should always be $10$ ms because the foundation policy was trained at $100$ Hz. The policy can be run at higher frequencies, though (e.g. $400$ Hz) in which case the native state progression in the policy is only triggered every $4$ steps as signified by `FORCE_SYNC_NATIVE`. 

This interface provides simple functions for inference like e.g.
```c++
auto executor_status = rl_tools_inference_applications_l2f_control(current_time * 1000, &observation, &action);
```
`rl_tools_inference_applications_l2f_control` should be called at the interval configured in `CONTROL_INTERVAL_INTERMEDIATE_NS`. 

The foundation policy that has been exported as C++ code should be included (e.g. `#include <blob/policy.h>` in the example) and it can be downloaded [here](https://github.com/rl-tools/px4-blob/blob/d081d8ca4a558d90f864e352442da14a4c7dd866/policy.h).


## Training

```bash
git clone https://github.com/rl-tools/raptor.git
cd raptor
git submodule update --init rl-tools
cd rl-tools
git submodule update --init --recursive -- external/highfive external/json external/tensorboard
cd ..
```

```bash
cat data/foundation-policy-v1-data.tar.gz.part_* > data/foundation-policy-v1-data.tar.gz
cd rl-tools
tar -xvf ../data/foundation-policy-v1-data.tar.gz
cd ..
```

The following can be skipped because the image is also available the Docker Hub and will be automatically downloaded by `docker run` we just include it here for complete reproducibility.
```bash
cd rl-tools/docker
./build_all.sh
cd ../../
```

```bash
docker run -it --rm -v $(pwd)/rl-tools:/rl-tools -v data:/data rltools/rltools:ubuntu24.04_mkl_gcc_base
```

```bash
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
```bash
cd rl-tools/src/foundation_policy
./extract_checkpoints.sh > checkpoints_$RL_TOOLS_EXTRACK_EXPERIMENT.txt
```
Then change the experiment name to the content of `$RL_TOOLS_EXTRACK_EXPERIMENT` in `post_training/main.cpp` and the experiment directory from `1k-experiments` to `experiments` such that it will find the newly trained checkpoints referred to by the `checkpoint_xxx.txt`.


### macOS
On macOS, use Accelerate instead of MKL and build natively replacing the CMake configure and build commands with:
```
cd rl-tools
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DRL_TOOLS_BACKEND_ENABLE_ACCELERATE=ON -DRL_TOOLS_ENABLE_TARGETS=ON -DRL_TOOLS_EXPERIMENTAL=ON -DRL_TOOLS_ENABLE_HDF5=ON -DRL_TOOLS_ENABLE_JSON=ON -DRL_TOOLS_ENABLE_TENSORBOARD=ON
cmake --build . --target foundation_policy_pre_training_sample_dynamics_parameters --target foundation_policy_pre_training --target foundation_policy_post_training
cd ..
./build/src/foundation_policy/foundation_policy_post_training
```

### CMake 4+

Use `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`