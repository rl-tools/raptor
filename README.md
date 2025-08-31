# RAPTOR: A Foundation Policy for Quadrotor Control

## Usage
If you want to use your own simulator:
```bash
pip install foundation-policy==1.0.0
```
```python
from foundation_policy import QuadrotorPolicy
policy = QuadrotorPolicy()
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
pip install l2f==2.0.18 ui-server==0.0.13 foundation-policy==1.0.0
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
from foundation_policy import QuadrotorPolicy

policy = QuadrotorPolicy()
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

## Training

```bash
git clone https://github.com/rl-tools/foundation-policy.git
cd foundation-policy
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
