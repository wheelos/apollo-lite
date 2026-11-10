# whl-nanoradar-visualizer

visualization tool for nanoradar

## Installation

### install requirements

```bash
pip install -r requirements.txt
```

## Usage

### before running visualizer

the visualizer requires the output proto files of nanoradar, and subscribes the
nanoradar messages from its channels. so you need to build the required proto files and run nanoradar first.

#### build protobuf files

```bash
bash apollo.sh build cyber wheelos_msgs radar
source setup.bash
```

#### build nanoradar and run it

Start the NanoRadar driver DAG supplied by the `hardware.radar` source or
installed runtime. Use the reviewed DAG and configuration shipped with that
module; this repository does not define a NanoRadar driver launch file.

### run visualizer

```bash
python3 whl_nanoradar_visualizer.py -c /apollo/sensor/nanoradar/front
```

> the `-c` or `--channel` option specifies the channel to subscribe to. multiple channels can be specified by repeating the `-c` option.
> the `-p` or `--port` option specifies the port to run the visualizer on, default is `8765`.

### view the visualizer

open a web browser and go to `http://localhost:8765/`, if running on a remote machine, replace `localhost` with the IP address of the machine running the visualizer.
