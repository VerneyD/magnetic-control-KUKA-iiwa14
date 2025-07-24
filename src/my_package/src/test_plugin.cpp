#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>

namespace gazebo
{
  class TestPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      std::cerr << "[TestPlugin] Chargé pour le modèle: " << _model->GetName() << std::endl;
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(TestPlugin)
}
