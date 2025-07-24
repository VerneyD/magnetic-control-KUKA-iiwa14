#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

namespace gazebo
{
  class MyBoxPlugin : public ModelPlugin
  {
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    // Paramètres
    double K = 0.1;      // Coefficient de raideur
    double z_target = 5.0; // Hauteur cible

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      std::cout << "Hello from MyBoxPlugin! (Attraction toward z=5)" << std::endl;

      this->model = _model;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MyBoxPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      physics::LinkPtr link = this->model->GetLink("base");
      if (link)
      {
        // Lire la position actuelle en z
        double current_z = link->WorldPose().Pos().Z();

        // Calcul de la force d'attraction vers z_target
        double force_z = -K * (current_z - z_target);

        // Appliquer la force
        ignition::math::Vector3d force(0, 0, force_z);
        link->AddForce(force);

        // Affichage pour debug
        std::cout << "Current z: " << current_z << ", Force_z: " << force_z << std::endl;
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(MyBoxPlugin)
}
