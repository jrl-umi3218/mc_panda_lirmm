#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule_visual.h>

#include <mc_panda/config.h>
#include <mc_panda/panda.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>
#include "config.h"

namespace mc_panda
{ //  TODO: this should be moved to mc_panda

static std::string pandaVariant(bool pump, bool foot, bool hand)
{
  if(pump && !foot && !hand)
  {
    // mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_pump'");
    return "pump";
  }
  if(!pump && foot && !hand)
  {
    // mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_foot'");
    return "foot";
  }
  if(!pump && !foot && hand)
  {
    // mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_hand'");
    return "hand";
  }
  if(!pump && !foot && !hand)
  {
    // mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_default'");
    return "default";
  }
  mc_rtc::log::error("PandaRobotModule does not provide this panda variant...");
  return "";
}

static std::string pandaVariantUppercase(bool pump, bool foot, bool hand, bool calibrated)
{
  // default, foot, hand...
  std::string v = calibrated ? "Calibrated::" : "";
  auto variant = pandaVariant(pump, foot, hand);
  if(variant.size())
  {
    // Default, Foot, Hand...
    variant[0] = std::toupper(variant[0]);
    v += variant;
  }
  return v;
}

/**
 * Default naming scheme for mc_panda::PandaRobotModule variants
 * These are used here to get the main robot module to be attached to the support
 * on which they are attached
 * TODO: move it to mc_panda and use it there
 */
inline static std::string NameFromParams(bool pump, bool foot, bool hand, bool calibrated)
{
  auto name = "Panda" + pandaVariantUppercase(pump, foot, hand, calibrated);
  return name;
}
} // namespace mc_panda

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
      return "Panda2LIRMM::Table1";
    case R::Panda2LIRMM_Table2:
      return "Panda2LIRMM::Table2";
    case R::Panda7LIRMM:
      return "Panda7LIRMM";
    case R::Panda7LIRMM_Table1:
      return "Panda7LIRMM::Table1";
    case R::Panda7LIRMM_Table2:
      return "Panda7LIRMM::Table2";
    case R::Panda6LIRMM:
      return "Panda6LIRMM";
    case R::Panda6LIRMM_MetalSupport:
      return "Panda6LIRMM_MetalSupport";
    default:
      return "UnknownPandaLIRMM";
  }
}

inline static std::string NameFromParams(PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool calibrated)
{
  auto variant = mc_panda::pandaVariantUppercase(pump, foot, hand, calibrated);
  if(variant.size())
  {
    variant = "::" + variant;
  }
  return to_string(robot) + variant;
}

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  auto endEffectorOptions = std::vector<std::array<bool, 3>>{
      {false, false, false}, {true, false, false}, {false, true, false}, {false, false, true}};

  using R = PandaLIRMMRobots;
  for(R robot : {R::Panda2LIRMM, R::Panda2LIRMM_Table1, R::Panda2LIRMM_Table2, R::Panda7LIRMM, R::Panda7LIRMM_Table1,
                 R::Panda7LIRMM_Table2, R::Panda6LIRMM, R::Panda6LIRMM_MetalSupport})
  {
    for(auto endEffector : endEffectorOptions)
    {
      cb(robot, endEffector[0], endEffector[1], endEffector[2],
         false); // non calibrated variants (using urdf from mc_panda)
    }
  }

  // calibrated variants
  // XXX: for now we only have a variant for panda_default generated, and calibration has only be done for Panda2
  // The smart thing to do here would be to refactor mc_panda to use ConnectModules and remove the extra urdf
  // panda_foot.urdf, panda_pump.urdf...
  cb(R::Panda2LIRMM, false, false, false, true); // calibrated variants (using urdf from ./calibrated_urdf/)
}

} // namespace mc_panda_lirmm

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda_lirmm;
    ForAllVariants([&names](PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool calibrated)
                   { names.push_back(mc_panda_lirmm::NameFromParams(robot, pump, foot, hand, calibrated)); });
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
          [&variant_factory](PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool calibrated)
          {
            using R = PandaLIRMMRobots;

            std::string variant_name = mc_panda_lirmm::NameFromParams(robot, pump, foot, hand, calibrated);
            // Use connect approach for these variants
            Eigen::Vector3d size;
            using Color = mc_rtc::gui::Color;
            Color color;
            double mass;
            auto calibrated_urdf_path =
                (fs::path{mc_panda_lirmm::CALIBRATED_PANDA_LIRMM_URDF_PATH} / "panda2lirmm").string();
            mc_rbdyn::RobotModule * pandaRm =
                calibrated ? new mc_panda::PandaRobotModule(pump, foot, hand, calibrated_urdf_path,
                                                            mc_panda::PANDA_DESCRIPTION_PATH, calibrated_urdf_path)
                           : new mc_panda::PandaRobotModule(pump, foot, hand);
            auto robot_name = std::string{};
            auto box_to_robot = sva::PTransformd::Identity();
            if(robot == R::Panda2LIRMM_Table1)
            {
              size = Eigen::Vector3d{0.75, 1.0, 0.75};
              color = Color::Gray;
              mass = 10;
              robot_name = "panda2_lirmm_default";
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()});
            }
            else if(robot == R::Panda2LIRMM_Table2)
            {
              size = Eigen::Vector3d{0.75, 1.0, 0.75};
              color = Color::Gray;
              mass = 10;
              robot_name = "panda2_lirmm2_default";

              // Moved by x:-23.8, y:-9.6 w.r.t original table attachement
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()});
              box_to_robot.translation().x() += 0.238;
              box_to_robot.translation().y() += 0.096;
            }
            else if(robot == R::Panda6LIRMM_MetalSupport)
            {
              size = Eigen::Vector3d{0.25, 0.27, 0.716};
              color = Color::Cyan;
              mass = 40;
              robot_name = "panda6_lirmm_default";
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 4), 0, -size.z()});
            }
            else if(robot == R::Panda7LIRMM_Table1)
            {
              size = Eigen::Vector3d{1.0, 0.75, 0.75};
              color = Color::LightGray;
              mass = 10;
              robot_name = "panda7_lirmm_default";
              box_to_robot = sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z()});
            }
            else if(robot == R::Panda7LIRMM_Table2)
            {
              size = Eigen::Vector3d{1.0, 0.75, 0.75};
              color = Color::LightGray;
              mass = 10;
              robot_name = "panda7_lirmm_default";
              // moved by x:-57.25, y:-1.0cm w.r.t to original table attachement
              box_to_robot = sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z()});
              box_to_robot.translation().x() -= 0.5725;
              box_to_robot.translation().y() -= 0.01;
            }

            variant_factory[variant_name] = [=]()
            {
              if(robot == R::Panda2LIRMM || robot == R::Panda6LIRMM || robot == R::Panda7LIRMM)
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

              return new mc_rbdyn::RobotModule(rmV->connect(
                  *pandaRm, "robot_support", "world", "",
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
