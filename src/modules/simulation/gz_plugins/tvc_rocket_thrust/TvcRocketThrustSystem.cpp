/****************************************************************************
 *
 *   Apply thrust from PX4 motor commands.
 *
 *   Default: force on direction link origin, axis follows that link (gimbaled).
 *   Optional fixed application point: thrust direction from direction_link,
 *   force applied at application_offset on application_link (e.g. base_link).
 *
 ****************************************************************************/

#include "TvcRocketThrustSystem.hpp"

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	TvcRocketThrustSystem,
	gz::sim::System,
	TvcRocketThrustSystem::ISystemConfigure,
	TvcRocketThrustSystem::ISystemPreUpdate
)

void TvcRocketThrustSystem::Configure(const gz::sim::Entity &entity,
				      const std::shared_ptr<const sdf::Element> &sdf,
				      gz::sim::EntityComponentManager &ecm,
				      gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	const std::string link_name = sdf->Get<std::string>("link_name", "motor_housing").first;
	const std::string direction_link_name =
		sdf->Get<std::string>("direction_link_name", link_name).first;
	const std::string application_link_name =
		sdf->Get<std::string>("application_link_name", link_name).first;

	const auto direction_entity = _model.LinkByName(ecm, direction_link_name);
	const auto application_entity = _model.LinkByName(ecm, application_link_name);

	if (!direction_entity) {
		throw std::runtime_error(
			"TvcRocketThrustSystem: direction link \"" + direction_link_name + "\" not found.");
	}

	if (!application_entity) {
		throw std::runtime_error(
			"TvcRocketThrustSystem: application link \"" + application_link_name + "\" not found.");
	}

	_direction_link = gz::sim::Link(direction_entity);
	_application_link = gz::sim::Link(application_entity);
	_direction_link.EnableVelocityChecks(ecm, true);
	_application_link.EnableVelocityChecks(ecm, true);

	if (sdf->HasElement("thrust_axis")) {
		_thrust_axis = sdf->Get<gz::math::Vector3d>("thrust_axis");
	}

	if (sdf->HasElement("application_offset")) {
		_application_offset = sdf->Get<gz::math::Vector3d>("application_offset");
	}

	_fixed_application_point = (direction_entity != application_entity)
				   || (_application_offset.Length() > 1e-9);

	_thrust_coefficient = sdf->Get<double>("thrust_coefficient", 3.0e-4).first;
	_thrust_min = sdf->Get<double>("thrust_min", 0.).first;
	_thrust_max = sdf->Get<double>("thrust_max", 250.).first;
	_rotor_slowdown = sdf->Get<double>("rotor_velocity_slowdown", 1.0).first;

	const auto name_comp = ecm.Component<gz::sim::components::Name>(entity);
	const std::string model_name = name_comp ? name_comp->Data() : "model";
	_command_topic = sdf->Get<std::string>("command_topic", "command/motor_speed").first;

	std::string full_topic = _command_topic;

	if (_command_topic.front() != '/') {
		full_topic = "/" + model_name + "/" + _command_topic;
	}

	if (!_node.Subscribe(full_topic, &TvcRocketThrustSystem::OnMotorCommand, this)) {
		throw std::runtime_error("TvcRocketThrustSystem: failed to subscribe to " + full_topic);
	}

	gzmsg << "TvcRocketThrustSystem on [" << model_name << "] direction_link ["
	      << direction_link_name << "] application_link [" << application_link_name
	      << "] topic [" << full_topic << "] thrust_axis [" << _thrust_axis
	      << "] application_offset [" << _application_offset
	      << "] coeff=" << _thrust_coefficient << " clamp=[" << _thrust_min
	      << ", " << _thrust_max << "] N" << std::endl;
}

void TvcRocketThrustSystem::OnMotorCommand(const gz::msgs::Actuators &msg)
{
	std::lock_guard<std::mutex> lock(_mutex);
	_last_cmd = msg;
	_have_cmd = true;
}

void TvcRocketThrustSystem::PreUpdate(const gz::sim::UpdateInfo &,
				      gz::sim::EntityComponentManager &ecm)
{
	gz::msgs::Actuators cmd;

	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (!_have_cmd) {
			return;
		}

		cmd = _last_cmd;
	}

	double omega_sum = 0.;
	int count = 0;

	for (int i = 0; i < cmd.velocity_size(); ++i) {
		const double omega = std::abs(cmd.velocity(i)) / _rotor_slowdown;

		if (omega > 1e-6) {
			omega_sum += omega;
			++count;
		}
	}

	if (count == 0) {
		return;
	}

	const double omega_eff = omega_sum / static_cast<double>(count);
	double thrust = _thrust_coefficient * omega_eff * omega_eff;
	thrust = gz::math::clamp(thrust, _thrust_min, _thrust_max);

	const auto direction_pose = _direction_link.WorldPose(ecm);

	if (!direction_pose.has_value()) {
		return;
	}

	gz::math::Vector3d axis = _thrust_axis;

	if (axis.Length() < 1e-9) {
		axis = gz::math::Vector3d(0., 0., 1.);
	}

	axis.Normalize();
	const gz::math::Vector3d force_world = direction_pose->Rot().RotateVector(axis * thrust);
	const gz::math::Vector3d zero_torque = gz::math::Vector3d::Zero;

	if (_fixed_application_point) {
		_application_link.AddWorldWrench(ecm, force_world, zero_torque, _application_offset);
	} else {
		_application_link.AddWorldWrench(ecm, force_world, zero_torque);
	}
}
