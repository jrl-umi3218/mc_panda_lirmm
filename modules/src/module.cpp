#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule_visual.h>

#include <mc_panda/config.h>
#include <mc_panda/panda.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>
#include "config.h"

namespace mc_panda_lirmm
{

enum class PandaLIRMMRobots
{
  Panda2LIRMM, // no table
  Panda2LIRMM_Table1, // base on table, position 1
  Panda2LIRMM_Table2, // base on table, position 2

  Panda7LIRMM, // original attachement position on the table
  Panda7LIRMM_Table1, // base on table, position 1
  Panda7LIRMM_Table2, // base on table, position 2

  Panda6LIRMM, // no support
  Panda6LIRMM_MetalSupport
};

static std::string to_string(PandaLIRMMRobots robots)
{
  using R = PandaLIRMMRobots;
  switch(robots)
  {
    case R::Panda2LIRMM:
      return "Panda2LIRMM";
    case R::Panda2LIRMM_Table1:
      return "Panda2LIRMM_Table1";
    case R::Panda2LIRMM_Table2:
      return "Panda2LIRMM_Table2";
    case R::Panda7LIRMM:
      return "Panda7LIRMM";
    case R::Panda7LIRMM_Table1:
      return "Panda7LIRMM_Table1";
    case R::Panda7LIRMM_Table2:
      return "Panda7LIRMM_Table2";
    case R::Panda6LIRMM:
      return "Panda6LIRMM";
    case R::Panda6LIRMM_MetalSupport:
      return "Panda6LIRMM_MetalSupport";
    default:
      return "UnknownPandaLIRMM";
  }
}

inline static std::string ModuleNameFromParams(mc_panda::PandaRobots pandaRobot,
                                               PandaLIRMMRobots lirmmRobot,
                                               mc_panda::Tools tool,
                                               bool calibrated)
{
  auto pandaRm = mc_panda::ModuleNameFromParams(pandaRobot, tool, ""); // ex: FR3_Default
  auto prefix = to_string(lirmmRobot);
  /* Panda2LIRMM_Table2 _ FR3_Default */
  auto name = to_string(lirmmRobot) + "_" + pandaRm;
  if(calibrated)
  {
    /* Panda2LIRMM_Table2 _ FR3_Default _ Calibrated */
    name += "_calibrated";
  }
  return name;
}

inline static std::string RobotNameFromParams(mc_panda::PandaRobots pandaRobot,
                                              PandaLIRMMRobots lirmmRobot,
                                              mc_panda::Tools tool,
                                              bool calibrated)
{
  auto name = ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated);
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
  return name;
}

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  using R = mc_panda_lirmm::PandaLIRMMRobots;
  using T = mc_panda::Tools;
  for(auto endEffector : {T::Default, T::Pump, T::Foot, T::Hand})
  {
    for(mc_panda::PandaRobots pandaRobot : {mc_panda::PandaRobots::FR1, mc_panda::PandaRobots::FR3})
    {
      for(R robot : {R::Panda2LIRMM, R::Panda2LIRMM_Table1, R::Panda2LIRMM_Table2, R::Panda7LIRMM,
                     R::Panda7LIRMM_Table1, R::Panda7LIRMM_Table2, R::Panda6LIRMM, R::Panda6LIRMM_MetalSupport})
      {
        cb(pandaRobot, robot, endEffector, false); // non calibrated variants (using urdf from mc_panda)
      }
    }

    // calibrated variants
    // XXX: for now we only have a variant for panda_default generated, and calibration has only be done for Panda2
    // The smart thing to do here would be to refactor mc_panda to use ConnectModules and remove the extra urdf
    // panda_foot.urdf, panda_pump.urdf...
    cb(mc_panda::PandaRobots::FR3, R::Panda2LIRMM, endEffector,
       true); // calibrated variants (using urdf from ./calibrated_urdf/)
    cb(mc_panda::PandaRobots::FR3, R::Panda2LIRMM_Table2, endEffector,
       true); // calibrated variants (using urdf from ./calibrated_urdf/)
    cb(mc_panda::PandaRobots::FR3, R::Panda7LIRMM, endEffector,
       true); // calibrated variants (using urdf from ./calibrated_urdf/)
    cb(mc_panda::PandaRobots::FR3, R::Panda7LIRMM_Table2, endEffector,
       true); // calibrated variants (using urdf from ./calibrated_urdf/)
  }
}

} // namespace mc_panda_lirmm

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda_lirmm;
    ForAllVariants(
        [&names](mc_panda::PandaRobots pandaRobot, PandaLIRMMRobots lirmmRobot, mc_panda::Tools tool, bool calibrated)
        { names.push_back(mc_panda_lirmm::ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated)); });
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaLIRMM")

    static auto variant_factory = []()
    {
      std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> variant_factory;
      using namespace mc_panda_lirmm;
      ForAllVariants(
          [&variant_factory](mc_panda::PandaRobots pandaRobot, PandaLIRMMRobots lirmmRobot, mc_panda::Tools tool,
                             bool calibrated)
          {
            using R = PandaLIRMMRobots;

            std::string lirmm_variant_name =
                mc_panda_lirmm::ModuleNameFromParams(pandaRobot, lirmmRobot, tool, calibrated);
            std::string panda_variant_name = mc_panda::ModuleNameFromParams(pandaRobot, tool);
            std::string robot_name = mc_panda_lirmm::RobotNameFromParams(pandaRobot, lirmmRobot, tool, calibrated);
            // Use connect approach for these variants
            Eigen::Vector3d size;
            using Color = mc_rtc::gui::Color;
            Color color;
            double mass;
            auto calibrated_urdf_path =
                (fs::path{mc_panda_lirmm::CALIBRATED_PANDA_LIRMM_URDF_PATH} / "panda2lirmm").string();
            mc_rbdyn::RobotModule * pandaRm =
                calibrated ? mc_panda::create(panda_variant_name, calibrated_urdf_path,
                                              mc_panda::PANDA_DESCRIPTION_PATH, calibrated_urdf_path)
                           : mc_panda::create(panda_variant_name);
            auto box_to_robot = sva::PTransformd::Identity();
            if(lirmmRobot == R::Panda2LIRMM_Table1)
            {
              size = Eigen::Vector3d{0.75, 1.0, 0.75};
              color = Color::Gray;
              mass = 10;
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()});
            }
            else if(lirmmRobot == R::Panda2LIRMM_Table2)
            {
              size = Eigen::Vector3d{0.75, 1.0, 0.75};
              color = Color::Gray;
              mass = 10;

              // Moved by x:-23.8, y:-9.6 w.r.t original table attachement
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()});
              box_to_robot.translation().x() += 0.238;
              box_to_robot.translation().y() += 0.096;
            }
            else if(lirmmRobot == R::Panda6LIRMM_MetalSupport)
            {
              size = Eigen::Vector3d{0.25, 0.27, 0.716};
              color = Color::Cyan;
              mass = 40;
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 4), 0, -size.z()});
            }
            else if(lirmmRobot == R::Panda7LIRMM_Table1)
            {
              size = Eigen::Vector3d{1.0, 0.75, 0.75};
              color = Color::LightGray;
              mass = 10;
              box_to_robot = sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z()});
            }
            else if(lirmmRobot == R::Panda7LIRMM_Table2)
            {
              size = Eigen::Vector3d{1.0, 0.75, 0.75};
              color = Color::LightGray;
              mass = 10;
              // moved by x:-57.25, y:-1.0cm w.r.t to original table attachement
              box_to_robot = sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z()});
              box_to_robot.translation().x() -= 0.5725;
              box_to_robot.translation().y() -= 0.01;
            }

            variant_factory[lirmm_variant_name] = [=]()
            {
              if(lirmmRobot == R::Panda2LIRMM || lirmmRobot == R::Panda6LIRMM || lirmmRobot == R::Panda7LIRMM)
              {
                return pandaRm;
              }

              auto boxYaml =
                  fmt::format(R"(
    name: robot_support
    origin:
      translation: [0, 0, {0}]
      rotation: [0, 0, 0]
    material:
      color:
        r: {5}
        g: {6}
        b: {7}
        a: {8}
    geometry:
      box:
        size: [{1}, {2}, {3}]
    inertia:
      mass: {4}
    fixed: true
    )",
                              size.z() / 2, size.x(), size.y(), size.z(), mass, color.r, color.g, color.b, color.a);

              auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);
              auto rmV = mc_rbdyn::robotModuleFromVisual("robot_support", boxConfig);

              auto base = "world";
              if(pandaRobot == mc_panda::PandaRobots::FR1)
              {
                base = "world";
              }
              else if(pandaRobot == mc_panda::PandaRobots::FR3)
              {
                base = "base";
              }
              return new mc_rbdyn::RobotModule(rmV->connect(
                  *pandaRm, "robot_support", base, "",
                  mc_rbdyn::RobotModule::ConnectionParameters{}.name(robot_name).X_other_connection(box_to_robot)));
            };
          });
      mc_rtc::log::info("mc_panda_lirmm robot module declares the following variants:\n{}",
                        mc_rtc::io::to_string(variant_factory, [](const auto & v) { return v.first; }));
      return variant_factory;
    }();

    // Load the desired robot variant from the factory
    auto it = variant_factory.find(n);
    if(it != variant_factory.end())
    {
      return it->second();
    }
    else
    {
      mc_rtc::log::error("PandaLIRMM module Cannot create an object of type {}", n);
      mc_rtc::log::info("Available variants are: {}",
                        mc_rtc::io::to_string(variant_factory, [](const auto & v) { return v.first; }));
      return nullptr;
    }
  }
}
