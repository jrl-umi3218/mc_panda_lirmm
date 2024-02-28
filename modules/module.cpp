#include "module.h"
#include <RBDyn/parsers/urdf.h>
#include "config.h"
#include <Eigen/Core>
#include <sch/S_Object/S_Box.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

// Compute the inertia matrix for a box
Eigen::Matrix3d computeBoxInertiaMatrix(const Eigen::Vector3d & size, double mass)
{
  // Compute moments of inertia
  double x = size.x();
  double y = size.y();
  double z = size.z();
  double Ixx = (mass / 12.0) * (y * y + z * z);
  double Iyy = (mass / 12.0) * (x * x + z * z);
  double Izz = (mass / 12.0) * (x * x + y * y);

  // Construct the inertia matrix
  Eigen::Vector6d inertiaV;
  inertiaV << Ixx, 0.0, 0.0, Iyy, 0.0, Izz;

  Eigen::Matrix3d inertiaM;
  // clang-format off
    inertiaM << inertiaV(0), inertiaV(1), inertiaV(2),
                        0.0, inertiaV(3), inertiaV(4),
                        0.0,         0.0, inertiaV(5);
  // clang-format on
  inertiaM = inertiaM.selfadjointView<Eigen::Upper>();

  return inertiaM;
}

// Compute the center of mass (CoM) for a box
Eigen::Vector3d computeBoxCenterOfMass(const Eigen::Vector3d & size, double mass)
{
  // Compute CoM coordinates
  double x = size.x();
  double y = size.y();
  double z = size.z();
  double com_x = (1.0 * x + 1.0 * y + 2.0 * (x / 2.0)) / (1.0 + 1.0 + 2.0);
  double com_y = (1.0 * y + 1.0 * z + 2.0 * (y / 2.0)) / (1.0 + 1.0 + 2.0);
  double com_z = (1.0 * z + 1.0 * x + 2.0 * (z / 2.0)) / (1.0 + 1.0 + 2.0);

  // Construct the CoM vector
  Eigen::Vector3d com_vector(com_x, com_y, com_z);
  return com_vector;
}

PandaLIRMM::PandaLIRMM(const std::string & robot_name) : mc_robots::PandaRobotModule(false, false, false)
{
  this->name = robot_name;
}

void PandaLIRMM::addBox(const std::string & box_name,
                        const std::string & parent_link,
                        const sva::PTransformd & parentToBox,
                        const Eigen::Vector3d & box_size,
                        double box_mass)
{
  Eigen::Vector3d com = computeBoxCenterOfMass(box_size, box_mass);
  Eigen::Matrix3d inertiaM = computeBoxInertiaMatrix(box_size, box_mass);

  rbd::parsers::Geometry geom;
  geom.type = rbd::parsers::Geometry::BOX;
  geom.data = [&]()
  {
    rbd::parsers::Geometry::Box box;
    box.size = box_size;
    return box;
  }();

  _visual[box_name] = {{box_name, sva::PTransformd::Identity(), geom, {}}};
  _collisionObjects[box_name] = {box_name, std::make_shared<sch::S_Box>(box_size.x(), box_size.y(), box_size.z())};
  _collisionTransforms[box_name] = sva::PTransformd::Identity();

  mbg.addBody({box_mass, com, inertiaM, box_name});
  auto jointName = "link0_to_" + box_name;
  mbg.addJoint({rbd::Joint::Type::Fixed, true, jointName});
  mbg.linkBodies("panda_link0", parentToBox, box_name, sva::PTransformd::Identity(), jointName);

  double i = 0.02;
  double s = 0.01;
  double d = 0;
  _minimalSelfCollisions.emplace_back("convex_panda_link4", "robot_stand", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link5", "robot_stand", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link6", "robot_stand", i, s, d);
  _minimalSelfCollisions.emplace_back("convex_panda_link7", "robot_stand", i, s, d);
}

void PandaLIRMM::create_urdf()
{
  const auto & robot_name = this->name;
  mb = mbg.makeMultiBody(mb.body(0).name(), true);
  mbc = rbd::MultiBodyConfig(mb);
  mbc.zero(mb);

  this->urdf_path = (bfs::temp_directory_path() / (robot_name + ".urdf")).string();
  {
    rbd::parsers::Limits limits;
    limits.lower = _bounds[0];
    limits.upper = _bounds[1];
    limits.velocity = _bounds[3];
    limits.torque = _bounds[5];
    std::ofstream ofs(this->urdf_path);
    ofs << rbd::parsers::to_urdf({mb, mbc, mbg, limits, _visual, _collision, robot_name});
    mc_rtc::log::info("Wrote URDF to {}", this->urdf_path);
  }
}

Panda2LIRMM::Panda2LIRMM() : PandaLIRMM("panda2_lirmm")
{
  const auto & robot_name = this->name;

  auto size = Eigen::Vector3d{0.75, 1.0, 0.75};
  double mass = 10;
  auto link0_to_robot_stand = sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z() / 2});
  addBox("robot_stand", "panda_link0", link0_to_robot_stand, size, mass);
  _default_attitude = {1, 0, 0, 0, 0, 0, size.z()};
  create_urdf();
}

Panda5LIRMM::Panda5LIRMM() : PandaLIRMM("panda5_lirmm")
{
  const auto & robot_name = this->name;

  double mass = 40;
  auto size = Eigen::Vector3d{0.25, 0.27, 0.716};
  // XXX this is an approximate position
  auto link0_to_robot_stand = sva::PTransformd(Eigen::Vector3d{-size.x() / 4, 0, -size.z() / 2});
  addBox("robot_stand", "panda_link0", link0_to_robot_stand, size, mass);
  _default_attitude = {1, 0, 0, 0, 0, 0, size.z()};
  create_urdf();
}

Panda7LIRMM::Panda7LIRMM() : PandaLIRMM("panda7_lirmm")
{
  const auto & robot_name = this->name;
  double mass = 10;
  auto size = Eigen::Vector3d{1.0, 0.75, 0.75};
  auto link0_to_robot_stand = sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z() / 2});
  addBox("robot_stand", "panda_link0", link0_to_robot_stand, size, mass);
  _default_attitude = {1, 0, 0, 0, 0, 0, size.z()};
  create_urdf();
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Panda2LIRMM", "Panda5LIRMM", "Panda7LIRMM"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("Panda2LIRMM")
    if(n == "Panda2LIRMM")
    {
      return new mc_robots::Panda2LIRMM();
    }
    else if(n == "Panda5LIRMM")
    {
      return new mc_robots::Panda5LIRMM();
    }
    else if(n == "Panda7LIRMM")
    {
      return new mc_robots::Panda7LIRMM();
    }
    else
    {
      mc_rtc::log::error("PandaLIRMM module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
