sudo cpufreq-set -c 1 -g "performance"
taskset 0x2 ./build/franka_panda_cartesian_motion 172.16.0.2
