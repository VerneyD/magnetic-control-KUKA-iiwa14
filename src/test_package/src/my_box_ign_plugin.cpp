#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/wrench.pb.h>
#include <iostream>

namespace my_plugins
{
class MyBoxIgnPlugin
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity &,
    const std::shared_ptr<const sdf::Element> &sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &) override
  {
    this->linkName1 = sdf->Get<std::string>("target_link1", "link1").first;
    this->linkName2 = sdf->Get<std::string>("target_link2", "link2").first;

    std::cout << "[Plugin] Searching for links: " << linkName1 << " and " << linkName2 << std::endl;

    _ecm.Each<ignition::gazebo::components::Name,
              ignition::gazebo::components::Link>(
      [&](const ignition::gazebo::Entity &entity,
          const ignition::gazebo::components::Name *name,
          const ignition::gazebo::components::Link *) -> bool
      {
        if (name->Data() == this->linkName1)
        {
          this->linkEntity1 = entity;
          std::cout << "[Plugin] Found link1: " << linkName1 << std::endl;
        }
        if (name->Data() == this->linkName2)
        {
          this->linkEntity2 = entity;
          std::cout << "[Plugin] Found link2: " << linkName2 << std::endl;
        }
        return true;
      });

    if (linkEntity1 == ignition::gazebo::kNullEntity ||
        linkEntity2 == ignition::gazebo::kNullEntity)
    {
      std::cerr << "[Plugin] ERROR: Could not find both links in simulation." << std::endl;
    }

    _ecm.CreateComponent(linkEntity1, ignition::gazebo::components::LinearVelocity());
    _ecm.CreateComponent(linkEntity2, ignition::gazebo::components::LinearVelocity());
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (linkEntity1 == ignition::gazebo::kNullEntity || linkEntity2 == ignition::gazebo::kNullEntity)
      return;

    ApplySpringForce(_ecm);
  }

private:
  void ApplySpringForce(ignition::gazebo::EntityComponentManager &_ecm)
  {
    double h = 0.075; // height of the coil relative to end-effector
    ignition::math::Pose3d pose1 = ignition::gazebo::worldPose(linkEntity1, _ecm);
    ignition::math::Pose3d pose2 = ignition::gazebo::worldPose(linkEntity2, _ecm);
    ignition::math::Vector3d vel1 = Velocity(_ecm, linkEntity1);
    ignition::math::Vector3d vel2 = Velocity(_ecm, linkEntity2);

    ignition::math::Vector3d localOffset(0, 0, h);
    ignition::math::Vector3d pos1 = pose1.Pos() + pose1.Rot().RotateVector(localOffset);
    ignition::math::Vector3d pos2 = pose2.Pos();

    ignition::math::Vector3d r = pos2 - pos1;
    double distance = std::max(r.Length(), minDistance);
    ignition::math::Vector3d direction = r / distance;

    ignition::math::Matrix3d M(
        direction.X() * direction.X(), direction.X() * direction.Y(), direction.X() * direction.Z(),
        direction.Y() * direction.X(), direction.Y() * direction.Y(), direction.Y() * direction.Z(),
        direction.Z() * direction.X(), direction.Z() * direction. Y(), direction.Z() * direction.Z());

    ignition::math::Matrix3d I3(1, 0, 0, 0, 1, 0, 0, 0, 1);

    ignition::math::Vector3d phi(0, 0, 1); // magnetization axis
    ignition::math::Vector3d phi_world = pose1.Rot().RotateVector(phi);
    ignition::math::Vector3d phi_capsule = pose2.Rot().RotateVector(phi);

    // Br: remanent magnetization of permanent magnet (T)
    double Br = 1.3;
    ignition::math::Vector3d m1 = magnet_volume * Br * phi_world / u0;
    ignition::math::Vector3d m2 = capsule_volume * Br * phi_capsule / u0;

    ignition::math::Vector3d B = (u0 / (4.0 * M_PI * std::pow(distance, 3))) * ((3 * M - I3) * m1);

    // Force calculation
    double dot_m2_r = m2.Dot(r);
    double dot_m1_r = m1.Dot(r);
    double dot_m1_m2 = m1.Dot(m2);
    double distance2 = distance * distance;
    double distance5 = std::pow(distance, 5);

    ignition::math::Vector3d magnetic_force = (3 * u0 / (4.0 * M_PI * distance5)) *
        (dot_m2_r * m1 + dot_m1_r * m2 + dot_m1_m2 * r - 5 * (dot_m1_r * dot_m2_r / distance2) * r);

    ignition::math::Vector3d torque(
        m2.Y() * B.Z() - m2.Z() * B.Y(),
        m2.Z() * B.X() - m2.X() * B.Z(),
        m2.X() * B.Y() - m2.Y() * B.X());

    ignition::math::Vector3d dampingForce = -D * (vel2 - vel1);
    ignition::math::Vector3d dragForce = -waterDragCoeff * vel2.Length() * vel2;
    ignition::math::Vector3d buoyancyForce(0, 0, buoyancyForceValue);

    ignition::math::Vector3d totalForce = magnetic_force + dampingForce + dragForce + buoyancyForce;

    std::cout << "Total Force: " << totalForce << std::endl;

    ignition::msgs::Wrench wrenchMsg;
    wrenchMsg.mutable_force()->set_x(totalForce.X());
    wrenchMsg.mutable_force()->set_y(totalForce.Y());
    wrenchMsg.mutable_force()->set_z(totalForce.Z());
    wrenchMsg.mutable_torque()->set_x(torque.X());
    wrenchMsg.mutable_torque()->set_y(torque.Y());
    wrenchMsg.mutable_torque()->set_z(torque.Z());

    auto existing = _ecm.Component<ignition::gazebo::components::ExternalWorldWrenchCmd>(linkEntity2);
    if (existing)
      *existing = ignition::gazebo::components::ExternalWorldWrenchCmd(wrenchMsg);
    else
      _ecm.CreateComponent(linkEntity2, ignition::gazebo::components::ExternalWorldWrenchCmd(wrenchMsg));
  }

  ignition::math::Vector3d Velocity(
    ignition::gazebo::EntityComponentManager &_ecm,
    const ignition::gazebo::Entity &_linkEntity)
  {
    const auto *velComp = _ecm.Component<ignition::gazebo::components::LinearVelocity>(_linkEntity);
    return velComp ? velComp->Data() : ignition::math::Vector3d::Zero;
  }

  std::string linkName1;
  std::string linkName2;
  ignition::gazebo::Entity linkEntity1{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity linkEntity2{ignition::gazebo::kNullEntity};

  const double capsuleRadius = 0.0025;
  const double capsule_volume = 4 * M_PI * capsuleRadius * capsuleRadius * capsuleRadius / 3;
  const double magnet_volume = M_PI * 0.0175 * 0.0175 * 0.065 ; 

  const double u0 = 4 * M_PI * 1e-7;  // Vacuum permeability
  const double D = 6 * M_PI * 0.001 * capsuleRadius; // Viscous damping 
  const double area = M_PI * capsuleRadius * capsuleRadius;
  const double waterDragCoeff = 0.5 * 1000 * 0.001 * area;
  const double buoyancyForceValue = 1000 * capsule_volume * 9.81;
  const double minDistance = 1e-3; // Cutoff to prevent singularity
};
} 

IGNITION_ADD_PLUGIN(
  my_plugins::MyBoxIgnPlugin,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(my_plugins::MyBoxIgnPlugin, "my_plugins::MyBoxIgnPlugin")
