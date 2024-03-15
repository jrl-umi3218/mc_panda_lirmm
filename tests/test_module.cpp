#include <mc_rbdyn/RobotLoader.h>
#include <mc_panda_lirmm/panda_lirmm.h>

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    mc_rtc::log::critical("Usage: {} [MODULE_DIR]", argv[0]);
    return 1;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({argv[1]});
  bool res = true;
  auto check_module =
      [&res](mc_panda_lirmm::PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame)
  {
    auto moduleName = NameFromParams(robot, pump, foot, hand, fixSensorFrame);
    auto rm = mc_rbdyn::RobotLoader::get_robot_module(moduleName);
    mc_rtc::log::info("{} provides robot {} which has {} dof", moduleName, rm->name, rm->mb.nrDof());
    res = rm ? 0 : 1;
  };

  mc_panda_lirmm::ForAllVariants(check_module);
}
