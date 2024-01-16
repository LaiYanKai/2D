# Citation
Please cite the paper [R2: Optimal vector-based and any-angle 2D path planning with non-convex obstacles](https://doi.org/10.1016/j.robot.2023.104606).
Note that the paper have some errors that slipped past the proofing stage:
* Pg 5, Eq 3, add a logical not to `IsTaut`. Should be:
  `Progressed(𝛱, 𝜏, 𝐧_𝑟) ∧ ¬IsTaut(𝛱, 𝜏, 𝐧_𝑟). `
* Pg 13, Table 2, caption should be:
  `Average speed ups for 3, 10, 20, and 30 turning points`

# 2D Any-angle Path-finding 
Contains selected any-angle 2D path planners written in C++17 by Lai Yan Kai.
Can be run from VSCode, and can be built on Ubuntu 20.04 LTS (Native or WSL2) with gcc9 and CMake.
Primarily features novel algorithms like R2. 

All algorithms are run from benchmarks available at https://movingai.com/benchmarks/grids.html. 
The benchmarks are by default placed in the `data` folder, and scaled benchmarks used in the R2 paper are available in this folder.
The results produced from runs are by default created in the `results` folder. 

**R2**: An optimal, novel vector-based any-angle algorithm. Delays line-of-sight checks and expands only the most promising turning points to return paths quickly on large sparse maps, or maps with few disjoint obstacles.
Both algorithms have diagonal blocking, in which no passage is allowed through vertices when their adjacent cells (four of them, occupied/free cells) are arranged in a checkerboard pattern.

**VG2B/VG2N**: Visibility Graph algorithm, primarily used for cost comparisons. *VG2B* has diagonal blocking . *VG2N* does not have diagonal blocking.

**ANYA2B/ANYA2N**: An attempt at ANYA. The original author's implementation is available at https://bitbucket.org/dharabor/pathfinding. This implementation is faster for short queries, but the author's implementation outpaces this implementation for longer queries. Open-list uses Insertion sort. *ANYA2B* has diagonal blocking, and *ANYA2N* does not have diagonal blocking. *ANYA2N* has bugs and should not be used.

**TS2B/TS2N**: Theta* for 2D. Open-list uses Insertion sort. *TS2B* and *TS2N* are respectively, diagonally blocked and non-diagonally blocked implementations.

**P2D Library**: Contains C++ line-of-sight algorithms, occupancy grid implementation, open-list algorithms, scenario loaders, result printers, debugging macros, and math functions. The library is used by the algorithms.
Contains Python 3 scripts to output detailed statistics like path cost, run time, and path coordinates from result files, and compare the statistics.

MATLAB scripts are available in the P2D library to generate the scaled maps, show plots for debugging and generate results for the R2 paper. As they are for internal use, they will not be described.

# Setup and Build 
Git clone and cd into the `2D` folder with:
```bash
git clone https://github.com/LaiYanKai/2D.git
cd 2D
```
All subsequent steps assume that you are in this `2D` folder.

## Using Visual Studio Code
1. Open this folder in Visual Studio Code. 
Please google *VSCode WSL2 remote* or *VSCode in Ubuntu* for help on this step.

2. The `.vscode/launch.json` file is configured to build and run in Release mode. To run in Debug mode, comment the lines for the Release mode and uncomment the lines for the Debug mode.

3. Use `F5` to build and run from VSCode. 
Using `Ctrl+Shift+B` will build in Release mode without running. To change `Ctrl+Shift+B` to build in Debug mode by default, change the `isDefault` option in `.vscode/tasks.json`.

## Using CMake
1. `cd` to this folder, and assign permissions to bash scripts for building and running: 
```bash
chmod +x *.sh
```
2. Build in Release (faster) mode with:
```bash
./build.sh Release
```
or `./build.sh Debug` Debug mode needs to be built. More later on how debug mode is used to show how R2 explores the map for a scenario.

# Executing (run.sh)
After building, executables will appear in the build folder.
The bash script [run.sh](https://github.com/LaiYanKai/2D/blob/main/run.sh) is provided for ease of testing multiple parameters.
The bash script runs the Release mode executable by default, which is  `build/Release/run/run_exe`.

## Providing Benchmarks
The benchmarks should be unzipped into the `data` folder. Some *.zip* files containing the benchmarks are already available in their subdirectories in the `data` folder, fetched in early 2023. The *.zip* files should be unzipped directly into their containing folders. 
For example, the file *arena.map.scen* should be located in *data/dao/* folder after unzipping *dao-scen.zip* in the *data/dao* folder.

## Parameters
To change the parameters given to the executable, the multiline command at the bottom of `run.sh` can be modified. The parameters are:

**--map_dir**: The folder containing the *.map* files of the benchmark. If *--name* includes a sub-directory, the sub-directory will be appended to this folder's path. `Empty` by default.

**--scen_dir**: The folder containing the *.map.scen* files of the benchmark. If *--name* includes a sub-directory, the sub-directory will be appended to this folder's path. `Empty` by default.

**--algs**: At least one of `ANYA2B`, `ANYA2N`, `R2`, `TS2B`, `TS2N`, `VG2B`, `VG2N` must be specified.

**--names**: The subdirectories (if any) and names of the benchmark *.map* and *.map.scen* files to run. This parameter will determine the result file paths. At least one name must be specified.

**--num_expts**: The number of times to run the experiments. If it is greater than 1, a suffix `.#` will be appended to the *--algs* parameter of the result file, where `#` is the experiment number. If *num_expts* is 1, no suffixes will be added. `1` by default.

**--ids**: The id of the scenario to run, starting from 0. If `-1` is given, all scenarios in the benchmark (*.map.scen* file) are run. `-1` by default.

**--result_dir**: If --ids is -1, this is the folder where result files will be stored. If *--name* includes a sub-directory, the sub-directory will be appended to this folder's path. `Empty` by default.

## Run
An example to run with `R2` on the scaled map `arena_scale2` used in the R2 paper is available in `run.sh` and `.vscode/launch.json`. 
The executable may be run easily from VSCode using `F5`. 
**Otherwise**, to run from the terminal:
```
./run.sh
```

## Terminal Output and Timing
The terminal output shows some detailed statistics for each scenario, including 
- The result file subdirectory and name
- Scenario id
- Wall-time duration passed (min) since the first scenario
- Start and goal points
- Shortest path cost
- Number of turning points
- High-resolution clock (ns) measurement for each scenario. 

The wall-time duration does not fully correspond to the high-resolution clock due to overheads to print the detailed information to terminal and get the next scenario. 
The high-resolution clock measures the full length of time that each algorithm's `run` method is run (see [P2D/include/P2D/scenarios.hpp](https://github.com/LaiYanKai/2D/blob/main/P2D/include/P2D/scenarios.hpp#L201)). 
The measurement includes any memory clearing and pointer deletions at the end of each run.

## Examples
### Test All Scenarios Once
Suppose `VG2B` and `R2` are to be tested on all scenarios for the files:
- data/da2/ht_mansion2b.map.scen
- data/da2/ht_mansion2b.map
- data/dao/arena.map.scen
- data/dao/arena.map

and the result files stored in the `results` folder in their respective sub-folders.

The parameters should be: `--algs VG2B R2 --result_dir results --map_dir data --scen_dir data --names da2/ht_mansion2b dao/arena`.
The result files are:
- results/da2/ht_mansion2b.VG2B.results
- results/da2/ht_mansion2b.R2.results
- results/dao/arena.VG2B.results
- results/dao/arena.R2.results

### Test All Scenarios Multiple Times
This example is useful for obtaining average run times for each scenario.
Suppose `R2` is to be tested on all scenarios three times for the files:
- data/sc1/Aftershock.map.scen
- data/sc1/Aftershock.map

and the result files stored in the `results` folder in their respective sub-folders.

The parameters are: `--algs R2 --result_dir results --map_dir data --scen_dir data --names sc1/Aftershock --num_expts 3`. The results files are:
- results/sc1/Aftershock.R2.0.results
- results/sc1/Aftershock.R2.1.results
- results/sc1/Aftershock.R2.2.results

### Test Multiple or One Scenario
If `--ids` is supplied with numbers other than `-1`, the result files will not be created. This example is useful for comparing costs, and should not be used for comparing run times.
Suppose `R2` is to be run for scenario ids 5 and 100:
- Aftershock.map.scen
- Aftershock.map

The parameters are: `--algs R2 --names Aftershock --ids 5 100`. The results are printed onto the terminal.

# Show Detailed Results (show_results.sh)
The result files that are generated stores only the high-resolution clock (ns) measurements of each scenario and the shortest path coordinates.
To get more detailed results, run [show_results.sh](https://github.com/LaiYanKai/2D/blob/main/show_results.sh). The bash script is linked to the Python3 script in [P2D/scripts/show_results.py](https://github.com/LaiYanKai/2D/blob/main/P2D/scripts/show_results.py), and is provided for convenience.

The parameters can be modified at the multiline command at the bottom of `show_results.sh`.

**--ids**: The id of the scenario to show. If `-1` is given, all scenarios are shown. `-1` by default.

**--dir**: The directory containing the result files. `Empty` by default.

**--name**: The sub-directory and stem of the files.

**--algs**: The algorithms to show. If multiple experiments were run, the algorithm names in the result files' paths will be suffixed with `.#` where `#` is the experiment number (e.g. `R2.0`). Simply insert the suffix to the algorithm name for this parameter (e.g. `R2.0`).

**--print**: Pipe the detailed statistics into a file instead of the terminal. Printed to terminal by default.

**--path**: Use the direct path instead of the parameters *--algs*, *--name* and *--dir*.

Run with:
```
./show_results.sh
```

# Compare Costs (compare_costs.sh)
Unless urgently needed, this will be marked as TODO.

# Verbosity
Verbosity is supported for `R2`, and `ANYA2B` and `ANYA2N`, where details about how the algorithms are run are printed to the terminal and a log file.
Verbosity is disabled by default. 

To enable,
in [P2D/include/P2D/_debugoptions.hpp](https://github.com/LaiYanKai/2D/blob/main/P2D/include/P2D/_debugoptions.hpp), change `P2D_VERBOSE` to `1`, otherwise `0`.
The information will be output to the **terminal**, and the **log file** indicated in `LOG_PATH` which is `dbg.log` by default.

Rebuild by running `F5` or `Ctrl+Shift+B` in VSCode, or run `./build.sh Release`. The program may be built and run in Debug mode from VSCode to examine the information step-by-step.

# Using Algorithm Libraries
All algorithms are built into their own libraries. All algorithm libraries use the P2D library, and the P2D library has to be linked.
For example, to use *R2* and *VG2B*, link the libraries in your folder's CMakeLists.txt (see [run/CMakeLists.txt](https://github.com/LaiYanKai/2D/blob/main/run/CMakeLists.txt)):
```CMake
target_link_libraries(run_exe PRIVATE P2D_LIB R2_LIB VG2_LIB )
```
Then, include the subdirectories in the workspace (see [CMakeLists.txt](https://github.com/LaiYanKai/2D/blob/main/CMakeLists.txt)):
```CMake
add_subdirectory(P2D)
add_subdirectory(R2)
add_subdirectory(VG2)
# add_subdirectory(your_folder)
```

Examples to run the algorithms in C++ can be found in [run/main.cpp](https://github.com/LaiYanKai/2D/blob/main/run/main.cpp).
Include the libraries with 
```cpp
#include "R2/R2.hpp"
#include "VG2/VG2.hpp"
```

First, initialise the grid with
```cpp
P2D::Grid grid;
P2D::getMap(grid, fpath_map);
```
where `fpath_map`  is the `std::filesystem::path` of the `.map` file.
It is possible to supply a `bool *` pointer and size information to the `P2D::Grid` constructor or `P2D::Grid::init` method instead of using `getMap`. See [P2D/include/P2D/grid.hpp](https://github.com/LaiYanKai/2D/blob/main/P2D/include/P2D/grid.hpp#L47).

All algorithms are in the namespace `P2D`, and in their respective algorithm namespace. Their classes and namespace are identical.

To initialise *R2*:
```cpp
P2D::R2::R2 r2(&grid);
```


VG2B saves the graph into a file, and loads it the next time it is run.
To initialise `VG2B`
```cpp
P2D::VG2::VG2<true> vg2b(&grid, fp_vg);
```
where `fp_vg` is the `std::filesystem::path` to save the file. The template parameter `true` is for *VG2B*, `false` for *VG2N*.

To run the algorithms:
```cpp
P2D::V2 start_coord(x1, y1);
P2D::V2 goal_coord(x2, y2);
r2.run(start_coord, goal_coord);
vg2b.run(start_coord, goal_coord);
```
where `V2` is the 2D integer vector class, and `x1`, `y1`, `x2`, `y2` are the start and goal coordinates. Note that the coordinates from the `.map.scen` files must be swapped to correspond to the standard right-hand frame of the map shown in `.map`. The code in this repository uses the right-hand frame.
