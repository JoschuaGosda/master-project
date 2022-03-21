# abb_egm_pyclient

Tested on RobotWare version:

- 6.08.01.

## Setup

1. Clone this repo.
2. Install `protoc` from protobuf. There are [prebuilt binaries available from
   GitHub](https://github.com/protocolbuffers/protobuf/releases/latest) if `protoc` is not available in your package manager.
   3.Find `egm.proto` in either
   `%LOCALAPPDATA%\ABB\RobotWare\RobotWare_6.XXXXX\utility\Template\EGM\egm.proto`
   or on the robot.

3. Run protoc to generate protobuf classes for python. Substitute `$SRC_DIR` for
   the location of `egm.proto`

```bash
protoc --python_out=abb_egm_pyclient $SRC_DIR/egm.proto
```

4. Install this package in your environment of choice.

```bash
cd abb_egm_pyclient
pip install -e .
```

## Usage

Run the EGM on your robot controller. [Here are some instructions if needed](https://gitlab.control.lth.se/tetov/abb_egm_instructions).

And on your computer (and your python environment):

```bash
python -m abb_egm_pyclient.run --help
```
