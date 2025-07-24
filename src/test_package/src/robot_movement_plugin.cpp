#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unordered_map>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace my_plugins
{

class RobotMovementPlugin
: public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:

  enum class State { Sync, Mirror, Adjust };

  RobotMovementPlugin() : currentState(State::Sync) {}

  void Configure(
    const ignition::gazebo::Entity &entity,
    const std::shared_ptr<const sdf::Element> & /*sdf*/,
    ignition::gazebo::EntityComponentManager &ecm,
    ignition::gazebo::EventManager &) override
  {
    setenv("ROS_DOMAIN_ID", "0", 1); 
    this->modelEntity = entity;

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    this->rosNode = std::make_shared<rclcpp::Node>("real_robot_sync_node");  // Création node ROS2

    this->jointStateSub = rosNode->create_subscription<sensor_msgs::msg::JointState>(
      "/lbr/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg)  
      {
        this->lastJointState = msg;
      });

    ecm.Each<ignition::gazebo::components::Joint,
             ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity &entity,
          const ignition::gazebo::components::Joint *,
          const ignition::gazebo::components::Name *name) -> bool
      {
        this->jointEntities[name->Data()] = entity;

        if (!ecm.Component<ignition::gazebo::components::JointVelocityCmd>(entity))
        {
          ecm.CreateComponent(entity,
            ignition::gazebo::components::JointVelocityCmd({0.0}));
        }

        if (!ecm.Component<ignition::gazebo::components::JointPosition>(entity))
        {
          ecm.CreateComponent(entity,
            ignition::gazebo::components::JointPosition({0.0}));
        }

        std::cout << "[Gazebo Plugin] Joint trouvé : " << name->Data() << std::endl;
        return true;
      });

    std::cout << "[RobotMovementPlugin] Initialisation complète." << std::endl;
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &ecm) override
  {
    rclcpp::spin_some(this->rosNode);

    if (!this->lastJointState)
      return;

    const double kP = 3.0;               // Gain proportionnel
    const double maxVel = 1.0;           // Vitesse max
    const double epsilon = 0.0005;       // Seuil erreur position
    const double zeroVelThreshold = 0.01; // Seuil vitesse nulle

    bool allSynced = true;
    bool allVelocitiesNearZero = true;

    for (size_t i = 0; i < this->lastJointState->name.size(); ++i)
    {
      const std::string &jointName = this->lastJointState->name[i];
      double realPosition = this->lastJointState->position[i];
      double realVelocity = this->lastJointState->velocity[i];

      if (this->jointEntities.find(jointName) == this->jointEntities.end())
        continue;

      auto jointEntity = this->jointEntities[jointName];

      auto *posComp = ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
      auto *velCmd = ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
      if (!velCmd)
        continue;

      double simPos = (posComp && !posComp->Data().empty()) ? posComp->Data()[0] : 0.0;

      double velocityToApply = 0.0;

      switch (currentState)
      {
        // case State::Sync:
        // {
        //   // double error = realPosition - simPos;
        //   // if (std::abs(error) > epsilon)
        //   //   allSynced = false;

        //   // velocityToApply = std::clamp(kP * error, -maxVel, maxVel);
        //   // break;
        // }

        case State::Mirror:
        {
          double error = realPosition - simPos;
          double velocityerror = std::clamp(kP * error, -maxVel, maxVel);
          velocityToApply = realVelocity + velocityerror;

          if (std::abs(realVelocity) > zeroVelThreshold)
            allVelocitiesNearZero = false;
          break;
        }

        // case State::Adjust:
        // {
        //   // double error = realPosition - simPos;
        //   // // Correction plus douce (demi-gain et vitesse max réduite)
        //   // velocityToApply = std::clamp(kP * error * 0.5, -maxVel * 0.3, maxVel * 0.3);
        //   // break;
        // }
      }

      velCmd->Data()[0] = velocityToApply;

      ecm.SetChanged(jointEntity,
                     ignition::gazebo::components::JointVelocityCmd::typeId,
                     ignition::gazebo::ComponentState::OneTimeChange);
    }

    // Gestion des transitions d’état
    currentState = State::Mirror;
    // if (currentState == State::Sync && allSynced)
    // {
    //   std::cout << "[Plugin] Joints initialisés. Passage en mode miroir." << std::endl;
    //   currentState = State::Mirror;
    // }
    // else if (currentState == State::Mirror && allVelocitiesNearZero)
    // {
    //   std::cout << "[Plugin] Vitesses nulles détectées. Passage en mode ajustement." << std::endl;
    //   currentState = State::Adjust;
    //   adjustStartTime = std::chrono::steady_clock::now();
    // }
    // else if (currentState == State::Adjust)
    // {
    //   // Quitte le mode ajustement après un délai (ex: 3 secondes)
    //   auto now = std::chrono::steady_clock::now();
    //   if (std::chrono::duration_cast<std::chrono::seconds>(now - adjustStartTime).count() > 3)
    //   {
    //     std::cout << "[Plugin] Fin du mode ajustement. Contrôle terminé." << std::endl;
    //     // Par exemple tu peux mettre currentState = State::Mirror; ou rester en Adjust
    //     // Ici on reste en Mirror pour continuer à copier vitesse réelle
    //     currentState = State::Mirror;
    //   }
    // }
  }

private:
  ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};
  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;
  sensor_msgs::msg::JointState::SharedPtr lastJointState;
  std::unordered_map<std::string, ignition::gazebo::Entity> jointEntities;

  State currentState;
  std::chrono::steady_clock::time_point adjustStartTime;
};

} // namespace my_plugins

IGNITION_ADD_PLUGIN(
  my_plugins::RobotMovementPlugin,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(my_plugins::RobotMovementPlugin, "my_plugins::RobotMovementPlugin")
