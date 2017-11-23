#include "ur_modern_driver/ros/trajectory_executer.h"
#include <endian.h>
#include <cmath>
#include <ros/ros.h>


TrajectoryExecuter::TrajectoryExecuter(URCommander &commander)
  : running_(false)
  , program_running_(false)
  , commander_(commander)
{
}

void TrajectoryExecuter::update_program_running(bool program_running)
{
    program_running_ = program_running;
}

