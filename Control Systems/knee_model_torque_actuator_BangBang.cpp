#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

int main() {
	try {
		// Create an OpenSim model
		Model osimModel;

		// Define some properties for the model's bodies
		double bodyMass = 10;
		Vec3 bodyMassCenter(0, 0, 0);
		Inertia bodyInertia(1, 1, 1, 0, 0, 0);

		// Create the bodies
		OpenSim::Body* thigh = new OpenSim::Body("thigh", bodyMass, bodyMassCenter, bodyInertia);
		OpenSim::Body* shank = new OpenSim::Body("shank", bodyMass, bodyMassCenter, bodyInertia);

		// Add geometry to the bodies for visualization
		thigh->attachGeometry(new Mesh("C:/OpenSim 4.4/Geometry/femur_l.vtp"));
		shank->attachGeometry(new Mesh("C:/OpenSim 4.4/Geometry/fibula_l.vtp"));

		// Add the bodies to the model
		osimModel.addBody(thigh);
		osimModel.addBody(shank);

		// Create the knee joint
		OpenSim::PinJoint* knee = new OpenSim::PinJoint("knee",
			osimModel.getGround(), // Parent body (pelvis)
			Vec3(0, 1, 0),         // Location in parent
			Vec3(0),               // Orientation in parent
			*thigh,                // Child body (femur)
			Vec3(0, 0, 0),         // Location in child
			Vec3(0));              // Orientation in child

		// Set the properties of the knee joint's coordinate
		knee->updCoordinate().setName("knee_angle");
		knee->updCoordinate().setDefaultValue(0.0);
		knee->updCoordinate().setDefaultLocked(false);
		knee->updCoordinate().setDefaultClamped(false);

		// Add the knee joint to the model
		osimModel.addJoint(knee);

		// Create a torque actuator that applies a torque to the knee
		OpenSim::TorqueActuator* torqueActuator = new OpenSim::TorqueActuator();
		torqueActuator->setName("knee_torque_actuator");
		torqueActuator->setBodyA(*thigh);
		torqueActuator->setBodyB(*shank);
		torqueActuator->setAxis(Vec3(0, 0, 1));  // Assuming the knee joint rotates about the z-axis
		
		double lowerThreshold = ...; // Lower threshold for knee angle
		double upperThreshold = ...; // Upper threshold for knee angle
		double maxTorque = ...; // Maximum magnitude of torque

		// Define the control function for the Bang-Bang controller
		class BangBangControlFunction : public OpenSim::Function {
		public:
			BangBangControlFunction(Coordinate& coordinate, double lowerThreshold, double upperThreshold, double maxTorque)
				: m_coordinate(coordinate), m_lowerThreshold(lowerThreshold), m_upperThreshold(upperThreshold), m_maxTorque(maxTorque) {}

			double calcValue(const SimTK::Vector& x) const override {
				double angle = m_coordinate.getValue(m_coordinate.getModel().getWorkingState());
				if (angle < m_lowerThreshold) return m_maxTorque;
				else if (angle > m_upperThreshold) return -m_maxTorque;
				else return 0.0;
			}

		private:
			Coordinate& m_coordinate;
			double m_lowerThreshold;
			double m_upperThreshold;
			double m_maxTorque;
		};

		// Create the Bang-Bang controller and add it to the actuator
		BangBangControlFunction* bangBangControl = new BangBangControlFunction(knee->updCoordinate(), lowerThreshold, upperThreshold, maxTorque);
		torqueActuator->setControlFunction(*bangBangControl);


		osimModel.addForce(torqueActuator);

		// Initialize the system and state
		SimTK::State& state = osimModel.initSystem();

		// Write the model to a file
		osimModel.print("C:/Users/harry/Desktop/FYP/CustomStuff/simple_knee_model.osim");
	}
	catch (const OpenSim::Exception& ex) {
		std::cout << "Error: " << ex.getMessage() << std::endl;
		return 1;
	}
	catch (std::exception& e) {
		std::cout << "Error: " << e.what() << std::endl;
		return 1;
	}
	catch (...) {
		std::cout << "Unknown error occurred" << std::endl;
		return 1;
	}

	return 0;
}
