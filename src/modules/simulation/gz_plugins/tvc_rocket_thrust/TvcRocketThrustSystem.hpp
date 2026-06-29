/****************************************************************************
 *
 *   TVC real-platform rocket thrust (no spinning propellers).
 *
 ****************************************************************************/

#pragma once

#include <gz/msgs/actuators.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <mutex>
#include <string>

namespace custom
{

class TvcRocketThrustSystem :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) override;

private:
	void OnMotorCommand(const gz::msgs::Actuators &msg);

	gz::transport::Node _node;
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Link _direction_link{gz::sim::kNullEntity};
	gz::sim::Link _application_link{gz::sim::kNullEntity};

	std::string _command_topic;
	gz::math::Vector3d _thrust_axis{0., 0., 1.};
	gz::math::Vector3d _application_offset{0., 0., 0.};
	double _thrust_coefficient{3.0e-4};
	double _thrust_min{0.};
	double _thrust_max{250.};
	double _rotor_slowdown{1.0};
	bool _fixed_application_point{false};

	std::mutex _mutex;
	gz::msgs::Actuators _last_cmd;
	bool _have_cmd{false};
};

} // namespace custom
