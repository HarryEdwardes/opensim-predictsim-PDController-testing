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
		Body* thigh = new Body("thigh", bodyMass, bodyMassCenter, bodyInertia);
		Body* shank = new Body("shank", bodyMass, bodyMassCenter, bodyInertia);

		// Add geometry to the bodies for visualization
		thigh->attachGeometry(new Mesh("PATH_TO_YOUR_GEOMETRY/thigh.vtp"));
		shank->attachGeometry(new Mesh("PATH_TO_YOUR_GEOMETRY/shank.vtp"));

		// Add the bodies to the model
		osimModel.addBody(thigh);
		osimModel.addBody(shank);

		// Define the knee joint coordinates and axes
		CoordinateSet kneeCoordSet;
		kneeCoordSet.setName("knee_coord_set");
		kneeCoordSet[0].setName("knee_angle");
		kneeCoordSet[0].setDefaultValue(0.0);
		kneeCoordSet[0].setDefaultLocked(false);
		kneeCoordSet[0].setDefaultClamped(false);

		// Create the knee joint
		PinJoint* knee = new PinJoint("knee",
			osimModel.getGround(), // Parent body (pelvis)
			Vec3(0, 1, 0),         // Location in parent
			Vec3(0),               // Orientation in parent
			*thigh,                // Child body (femur)
			Vec3(0, 0, 0),         // Location in child
			Vec3(0),               // Orientation in child
			kneeCoordSet);

		// Add the knee joint to the model
		osimModel.addJoint(knee);

		// Create a muscle that spans the joint
		Thelen2003Muscle* muscle = new Thelen2003Muscle("extensor", 200, 0.6, 0.55, 0);
		muscle->addNewPathPoint("muscle_origin", *thigh, Vec3(0, 0.8, 0));
		muscle->addNewPathPoint("muscle_insertion", *shank, Vec3(0, 0.7, 0));

		// Add the muscle to the model
		osimModel.addForce(muscle);
		
		// Create a prescribed controller that specifies the excitation of the muscle
		PrescribedController* controller = new PrescribedController();
		controller->addActuator(*muscle);
		controller->prescribeControlForActuator("extensor", new StepFunction(0.5, 3.0, 0.3, 1.0));

		// Add the controller to the model
		osimModel.addController(controller);
		
		// Create a ligament
		Ligament* ligament = new Ligament("knee_ligament", *thigh, Vec3(0, 0, 0), *shank, Vec3(0, 0, 0));
		ligament->setRestingLength(0.2); // The length of the ligament when it is not stretched
		ligament->setStiffness(10.0); // The stiffness of the ligament
		ligament->setStrainAtOneNormForce(0.04); // The strain when the ligament force equals the body weight

		// Add the ligament to the model
		osimModel.addForce(ligament);



		// Initialize the system and state
		SimTK::State& state = osimModel.initSystem();

		// Write the model to a file
		osimModel.print("simple_knee_model.osim");
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
