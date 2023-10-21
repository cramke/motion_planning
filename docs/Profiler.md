# Profiling with Hotspot

## Usage
Launch Hotpot and use "Record Data" function with elevated rights using their GUI.
[Hotspot Documentation](https://github.com/KDAB/hotspot#recording-with-perf-without-super-user-rights)

## Prerequisites
1. Install [Hotspot](https://github.com/KDAB/hotspot)

## Not Recommended 
It works but requires root access to run the profiler.
1. Run `cargo build`
2. Run `sudo perf record --call-graph dwarf ./target/debug/mpl`
3. Run `sudo hotspot perf.data`
