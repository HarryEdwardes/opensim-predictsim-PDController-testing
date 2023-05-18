#include "PDController.h"

using namespace OpenSim;
using namespace SimTK;

// Constructor
PDController::PDController() {
	constructProperties();
}

// Destructor
PDController::~PDController() {}

// Setters
void PDController::setKp(double kp) {
	_kpProp.setValue(kp);
}

void PDController::setKd(double kd) {
	_kdProp.setValue(kd);
}

void PDController::setReferenceValues(const Vector& ref_values) {
	_referenceValuesProp.setValue(ref_values);
}

// Getters
double PDController::getKp() const {
	return _kpProp.getValue();
}

double PDController::getKd() const {
	return _kdProp.getValue();
}

const Vector& PDController::getReferenceValues() const {
	return _referenceValuesProp.getValue();
}

// Compute control torque using PD control law
double PDController::computeControlTorque(const Vector& current_values) const {
	// Calculate position and velocity errors
	double posError = getReferenceValues()[0] - current_values[0];
	double velError = getReferenceValues()[1] - current_values[1];

	// Compute control torque using PD control law
	return getKp() * posError + getKd() * velError;
}

// Member function to implement required Controller interface
void PDController::computeControls(const State& state, Vector &controls) const {
	// Call computeControlTorque with current values for position and velocity
	// from the main code (not implemented here)
}

// Private member function for setting up properties
void PDController::constructProperties() {
	constructProperty_kp(0.0);
	constructProperty_kd(0.0);
	constructProperty_referenceValues(Vector(2, 0.0));
}