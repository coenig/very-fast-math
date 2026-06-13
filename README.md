# Model Checking for ADAS
This is the open source project `very fast math`, first announced in [Towards Safe Autonomous Driving: Model Checking a Behavior Planner during Development](https://link.springer.com/chapter/10.1007/978-3-031-57249-4_3). 
We hereby make all the sources of the toolchain public which have so far been published in binary format only ([on Zenodo](https://zenodo.org/records/10013662)).

Thank you for using
~~~
         ___           
.--.--..'  _|.--------.
|  |  ||   _||        |
 \___/ |__|  |__|__|__|
   very fast math
~~~

## VFM Library
`vfm` is a formal verification software for [ADAS](https://en.wikipedia.org/wiki/Advanced_driver-assistance_system) with the [nuXmv model checker](https://nuxmv.fbk.eu/) in its core. It can 
- parse C++ code of an automated driving function (or other) and translate it into a transition system for the nuXmv model checker;
- optionally integrate it with an environment model, providing a discrete traffic simulation for the driving function to be verified against; the results can be converted into scenarios using [OSM](https://wiki.openstreetmap.org/wiki/OSM_file_formats) / [OSC2](https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/welcome.html).

In the **ultra-cooperative driving mode**, a fleet of cars can be steered by provably obeying a set of requirements.

## Examples
### MC-generated traffic situation on highway
<img src="examples/cex.png" alt="Image from a counterexample sequence generated through model checking" width="1000"/>

### MC-generated track and EGO behavior
<img src="examples/cex.gif" width="1000" />

### Ultra-cooperative driving: live steering of a fleet by model checker
<img src="examples/ucd_example.gif" width="1000" />

Steered by the MC, a fleet of cars provably* cooperates to accomplish a given formal SPEC. In the example, two cars pass each other on a narrow road with parked cars ("Nudging"). Entrance file: `morty/morty.py`. (* Given a few assumtions, but decreasing... :wink: )

## How to build
`vfm` is implemented in `C++` and can be built with CMake (stable) or Bazel (experimental). With CMake, simply run the `build.bash` script. *(On Windows, you can alternatively open the top-level `CMakeLists.txt` with Visual Studio and build the `vfm` target.)*

Run `vfm(.exe)` from the `bin` folder.

### Build dependencies
| Dependency | Minimum version | Platform | Notes |
|---|---|---|---|
| CMake | 3.21+ | Both | Required for VS 2022+ generator support |
| GCC/G++ | 7+ | Linux | C++17 support required |
| Visual Studio | 2019+ | Windows | Build Tools or full IDE |
| Git Bash / MSYS2 | — | Windows | Only for using `build.bash` to compile |

### Troubleshoot
There are no additional dependencies, except `gtest` if you want to run the tests, and `opengl` if you want to compile fltk agains it. These dependencies are technically optional, but in the recent versions they are required for the build script to work. Should you receive errors, do:
```
sudo apt-get update
sudo apt-get install libgtest-dev
sudo apt-get install libglew-dev
```

## M²oRTy (ultra-cooperative driving)
For the UCD framework you need additionally `gymnasium` and `highway-env` (as well as python3 with pip which we assume is there):
```
pip install gymnasium
pip install "gymnasium[other]"
pip install highway-env
```
(TODO: Probably `pip install moviepy` suffices rather than installing all 'other' of gymnasium.)

### Running M²oRTy
Run from the project root directory, for example:

```bash
python -m morty.morty --num_runs 1 --steps_per_run 300 --headless --force --record_video
```
Run with `--help` to see all available options.

This should work on Linux and Windows.

## Authors
Lukas Koenig,
Alexander Georgescu,
Christian Heinzemann,
Christian Schildwaechter,
Michaela Klauck,
Alberto Griggio,
Alberto Bombardelli,
Henning Koch et al.
