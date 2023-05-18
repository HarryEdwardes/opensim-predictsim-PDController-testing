#include <OpenSim/OpenSim.h>
#include "PDController.h"

using namespace OpenSim;
using namespace SimTK;

int main() {
	try {
		// Create the OpenSim model
		Model model;
		model.setName("PDControllerTest");
		model.setGravity(Vec3(0, -9.81, 0));

		// Create a body
		double bodyMass = 1.0;
		Vec3 bodyInertia(1.0, 1.0, 1.0);
		Body* body = new Body("body", bodyMass, Vec3(0), bodyInertia);
		model.addBody(body);

		// Create a joint
		PinJoint* joint = new PinJoint("joint", model.getGround(), Vec3(0), Vec3(0), *body, Vec3(0), Vec3(0));
		model.addJoint(joint);

		// Create an actuator
		TorqueActuator* actuator = new TorqueActuator(joint->updCoordinate());
		actuator->setName("torque_actuator");
		model.addForce(actuator);

		// Create the PD controller
		PDController* pdController = new PDController();
		pdController->setKp(50.0);
		pdController->setKd(10.0);
		pdController->setReferenceValues(Vector(2, 0.0)); // Set reference position and velocity
		pdController->addActuator(*actuator);
		model.addController(pdController);

		// Initialize the system
		State& state = model.initSystem();
		model.equilibrateMuscles(state);

		// Set initial position and velocity
		joint->getCoordinate().setLocked(state, false);
		joint->getCoordinate().setValue(state, 0.0);
		joint->getCoordinate().setSpeedValue(state, 0.0);

		// Simulate the model
		Manager manager(model);
		manager.setIntegratorAccuracy(1e-6);
		manager.initialize(state);
		state = manager.integrate(2.0); // Simulate for 2 seconds

		// Print the final position and velocity
		std::cout << "Final position: " << joint->getCoordinate().getValue(state) << std::endl;
		std::cout << "Final velocity: " << joint->getCoordinate().getSpeedValue(state) << std::endl;

	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}