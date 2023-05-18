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


		// Here I create a ligament with the purpose of better simulating human knee-joint behaviour
		// Create a ligament
		Ligament* ligament = new Ligament("knee_ligament", *thigh, Vec3(0, 0, 0), *shank, Vec3(0, 0, 0));
		ligament->setRestingLength(0.2); // The length of the ligament when it is not stretched
		ligament->setStiffness(10.0); // The stiffness of the ligament
		ligament->setStrainAtOneNormForce(0.04); // The strain when the ligament force equals the body weight

		// Add the ligament to the model
		osimModel.addForce(ligament);


		// Create a torque actuator that applies a torque to the knee
		OpenSim::TorqueActuator* torqueActuator = new OpenSim::TorqueActuator();
		torqueActuator->setName("knee_torque_actuator");
		torqueActuator->setBodyA(*thigh);
		torqueActuator->setBodyB(*shank);
		torqueActuator->setAxis(Vec3(0, 0, 1));  // Assuming the knee joint rotates about the z-axis
		
		double desiredKneeAngle = ...; // Set this to the desired knee angle
		double Kp = ...; // Proportional gain
		double Kd = ...; // Derivative gain

		// Define the control function for the PD controller with variability and delay
		class PDControlFunction : public OpenSim::Function {
		public:
			PDControlFunction(Coordinate& coordinate, double desiredAngle, double Kp, double Kd, int delaySteps)
				: m_coordinate(coordinate), m_desiredAngle(desiredAngle), m_Kp(Kp), m_Kd(Kd), m_delaySteps(delaySteps) {}

			double calcValue(const SimTK::Vector& x) const override {
				double error = m_desiredAngle - m_coordinate.getValue(m_coordinate.getModel().getWorkingState());
				double errorRate = -m_coordinate.getSpeedValue(m_coordinate.getModel().getWorkingState());

				// Add variability to the desired angle and control gains
				std::random_device rd;
				std::mt19937 gen(rd());
				std::normal_distribution<> d(0, 0.01); // mean of 0 and std_dev of 0.01
				error += d(gen); // Add variability to the error

				double control = m_Kp * error + m_Kd * errorRate;

				// Add delay
				m_previousControls.push(control);
				if (m_previousControls.size() > m_delaySteps) {
					control = m_previousControls.front();
					m_previousControls.pop();
				}

				return control;
			}

		private:
			Coordinate& m_coordinate;
			double m_desiredAngle;
			double m_Kp;
			double m_Kd;
			int m_delaySteps;
			mutable std::queue<double> m_previousControls;
		};

		// Create the PD controller and add it to the actuator
		PDControlFunction* pdControl = new PDControlFunction(knee->updCoordinate(), desiredKneeAngle, Kp, Kd);
		torqueActuator->setControlFunction(*pdControl);

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
