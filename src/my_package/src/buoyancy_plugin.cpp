#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>  // Ajouté pour que sdf::ElementPtr soit bien reconnu

#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class BuoyancyPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      gzdbg << ">>> BuoyancyPlugin Load() called for model: " << _model->GetName() << std::endl;

      this->model = _model;
      this->world = this->model->GetWorld();  // correction ici !

      // Parameters from SDF or defaults
      if (_sdf->HasElement("fluid_density"))
        this->fluidDensity = _sdf->Get<double>("fluid_density");
      else
        this->fluidDensity = 1000.0; // water density in kg/m^3

      if (_sdf->HasElement("volume"))
        this->objectVolume = _sdf->Get<double>("volume");
      else
        this->objectVolume = 0.001; // m^3

      if (_sdf->HasElement("center_of_buoyancy"))
        this->centerOfBuoyancy = _sdf->Get<ignition::math::Vector3d>("center_of_buoyancy");
      else
        this->centerOfBuoyancy = ignition::math::Vector3d(0, 0, 0);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&BuoyancyPlugin::OnUpdate, this));

      gzdbg << "BuoyancyPlugin loaded for model " << this->model->GetName()
            << " with fluid density: " << this->fluidDensity
            << ", volume: " << this->objectVolume
            << ", center of buoyancy: " << this->centerOfBuoyancy << std::endl;
    }

    void OnUpdate()
    {
      // Apply buoyant force
      // Buoyant force = fluid_density * volume * g
      double g = this->world->Gravity().Length();
      double buoyantForceMag = this->fluidDensity * this->objectVolume * g;

      ignition::math::Vector3d buoyantForce(0, 0, buoyantForceMag);

      // Apply force at center of buoyancy in world frame
      physics::LinkPtr link = this->model->GetLink();
      if (link)
      {
        ignition::math::Vector3d cobWorld = link->WorldPose().CoordPositionAdd(this->centerOfBuoyancy);
        link->AddForceAtWorldPosition(buoyantForce, cobWorld);
      }
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    double fluidDensity; // kg/m^3
    double objectVolume; // m^3
    ignition::math::Vector3d centerOfBuoyancy; // relative to link frame
  };

}  // namespace gazebo

// Très important : placer la macro EN DEHORS du namespace !
GZ_REGISTER_MODEL_PLUGIN(gazebo::BuoyancyPlugin)
