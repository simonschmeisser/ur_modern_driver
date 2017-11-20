#include "ur_modern_driver/ros/trajectory_executer.h"
#include <endian.h>
#include <cmath>
#include <ros/ros.h>


TrajectoryExecuter::TrajectoryExecuter(URCommander &commander)
  : running_(false)
  , commander_(commander)
{
}

