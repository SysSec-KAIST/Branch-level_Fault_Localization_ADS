# Overview
This repository is the artifact for *Branch-Level Fault Localization in ADS Planning
via Temporal Coverage Analysis* (ISSTA 2026). We focus on the debugging problem for planning failures, rather than on failure detection itself. To study this problem concretely, we ground our analysis in planning failures reported in recent work on Apollo, an industry-grade module-based ADS [arxiv link](https://arxiv.org/abs/2601.09171). Given a recorded driving failure, it
ranks the branches of an ADS planning module by how likely each is to be responsible
for the observed failure, narrowing the manual search for the fault.
 
The planning module of a production ADS such as Baidu Apollo is rule-based and holds
thousands of conditional branches, and because it runs in a closed-loop manner, coverage aggregated
over a whole run barely differs between failing and non-failing runs, which is why
conventional spectrum-based fault localization does not apply.
 
This artifact instead collects **branch coverage per planning frame** and analyzes the
**coverage sequence against the temporal window** in which the symptom develops, **ranking
branches whose activation pattern shifts in step with the failure**. We evaluated it on 221 real failures reproduced on Baidu Apollo v7.0.0 and used it to report four previously unknown failures.

For more details, please refer to our [paper](https://syssec.kaist.ac.kr/pub/2026/issta2026.pdf)
 
This artifact depends on:

- [apollo_debug](https://github.com/RomainLettuce/apollo_debug/tree/SEFL): our
  instrumented Apollo fork, pinned at a specific commit
- [SORA-SVL](https://github.com/YuqiHuai/SORA-SVL): local replacement for the
  discontinued SVL asset cloud

See also the [companion site](https://sites.google.com/view/adsfaultlocalization).

## Repository Structure

- `reproduce/`: scripts for replaying and reproducing failure cases in Apollo/LGSVL.
- `localization/`: implementation of Suspicious Frame Localization (SFL) and Suspicious Branch Localization (SBL).
- `localization/Data/`: input metadata used by the localization scripts, including solution and failure-case information. solution.json provides the chromosomes (simulation input parameters) of 221 failures.
- `localization/sample/`: sample diagnostic traces and coverage data for testing the localization scripts without reproducing the full simulator environment.

The omitted raw logs are not required to inspect the implementation or reproduce the localization pipeline on the provided sample cases. Reproducing the full set of 221 failures requires running the provided reproduction scripts in the Apollo 7.0 + LGSVL + SORA-SVL environment described below.

## Experimental Setup

* AD system side
   * OS: Ubuntu 20.04.1 LTS
   * GPU: NVIDIA GeForce RTX 2080 Ti
   * Apollo version: Baidu Apollo r7.0.0
   * Enabled modules: Localization, Perception, Transform, Routing, Prediction,
     Planning, Traffic Light, Control, Recorder
   * Prediction module modification: Perception input replaced with 3D ground truth
     (gt_perception)
* Simulator side
   * OS: Ubuntu 22.04.5 LTS
   * GPU: NVIDIA GeForce RTX 3090
   * Simulator: LGSVL Simulator 2021.3, integrated via
     [SORA-SVL](https://github.com/YuqiHuai/SORA-SVL)
   * Ego vehicle: Lincoln 2017 MKZ (Apollo 7.0 sensor configuration)
   * Map: SanFrancisco_correct
   * Simulation mode: API-only, driven by the LGSVL Python API
   * Bridge: CyberRT bridge (localhost:9090) between the simulator and Apollo

## Setup
###  (Step 1) Baidu Apollo
- Clone custom Apollo ADS (customized for instrumentation) [apollo_debug](https://github.com/RomainLettuce/apollo_debug/tree/SEFL)
- Copy the base_map.bin file of SanFrancisco_Correct to the map module of apollo.
- If you need the base_map.bin file of SanFrancisco_Correct, please contact the authors.
```bash
cd ~
git clone https://github.com/RomainLettuce/apollo_debug -b SEFL apollo
mkdir ~/apollo/modules/map/data/SanFrancisco_correct
cp base_map.bin ~/apollo/modules/map/data/SanFrancisco_correct
```
- Instrument Apollo with the provided Python script
```bash
python ~/apollo/scripts/instrument_coverage.py --instrument --build --include ~/apollo/modules/planning/tasks
python ~/apollo/scripts/instrument_coverage.py --instrument --build --include ~/apollo/modules/planning/scenarios
python ~/apollo/scripts/instrument_coverage.py --instrument --build --include ~/apollo/modules/planning/traffic_rules
```
* Build & enter to apollo container
```bash
./dev_start.sh
./dev_into.sh
```
* Generate a map data
```bash
bash generate_map.sh SanFrancisco_correct
```
* Build apollo
```bash
bash apollo_build.sh
```
* Replace perception as ground-truth
```bash
cd ~/apollo

sed -i.bak 's|"/apollo/perception/obstacles"|"/apollo/perception/obstacles_gt"|' \
  modules/prediction/dag/prediction.dag \
  modules/prediction/conf/prediction_conf.pb.txt
```
* Start bootstrap and bridge
```bash
cd /apollo
bash scripts/bootstrap_lgsvl.sh
cyber_bridge
```
 
### (Step 2) SORA-SVL (Local Cloud Server for Simulator)
Setup Local cloud server for LGSVL simulator: [SORA-SVL](https://github.com/YuqiHuai/SORA-SVL)
### (Step 3) LGSVL Simulator
- Download custom LGSVL simulator
[simulator](https://github.com/RomainLettuce/simulator_TCADS)
- You can execute the simulator by  clicking `simulator.x86_64`
- Make sure that you set `cloud_url` in config.yaml as the URL of your SORA-SVL cloud server.
### (Step 4) PythonAPI (For LGSVL)
- Download Python API for SVL simulator 
[PythonAPI](https://github.com/RomainLettuce/PythonAPI/tree/SEFL)
* Unzip the downloaded repo at `~/PythonAPI` outside the Apollo container.
* Install the Python API:
```bash
cd ~/PythonAPI
python3 -m pip install -r requirements.txt --user .
```
### (Step 5) Reproduce Failures
- Reproduce failure cases by executing the shell script
```bash
cd ./reproduce
bash reproduce_failures.sh
```
Note that you should configure  `input_replay.py` to use your Apollo's ip and ports for bridge and dreamview
### (Step 6) Fault Localization
- Requirement
```bash
sudo apt-get update
sudo apt-get install -y protobuf-compiler
```
```bash
pip install protobuf
```
- Run `SFL.py`.

  -   **Input**: diagnostic traces, failure-case metadata, and solution.json.
  -   **Output**: suspicious frame information for each failure case.
```bash
cd ./localization
python SFL.py --solution_json ./Data/solution.json --extra_sys_path ~/apollo/.cache/bazel/540135163923dd7d5820f3ee4b306b32/execroot/apollo/bazel-out/k8-fastbuild/bin
```
- Then, run `SBL.py` to get a ranked list of suspicious branch groups.

  - **Input**: branch coverage traces and suspicious-frame information generated by SFL.
  - **Output**: ranked suspicious branch groups for each failure case.
```bash
python SBL.py --root ~/apollo/data/coverage
```
Sample inputs are provided in the `sample/` directory, and all argument defaults are configured for these sample cases.
Please check the argument of each file for full dataset usage.

