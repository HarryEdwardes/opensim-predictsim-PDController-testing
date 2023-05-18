#ifndef OPENSIM_PD_CONTROLLER_H_
#define OPENSIM_PD_CONTROLLER_H_

#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Common/Property.h>

namespace OpenSim {

	class PDController : public Controller {
		OpenSim_DECLARE_CONCRETE_OBJECT(PDController, Controller);

	public:
		// Constructor and Destructor
		PDController();
		virtual ~PDController();

		// Setters
		void setKp(double kp);
		void setKd(double kd);
		void setReferenceValues(const SimTK::Vector& ref_values);

		// Getters
		double getKp() const;
		double getKd() const;
		const SimTK::Vector& getReferenceValues() const;

		// Compute control torque using PD control law
		double computeControlTorque(const SimTK::Vector& current_values) const;

	protected:
		// Member function to implement required Controller interface
		void computeControls(const SimTK::State& state, SimTK::Vector &controls) const override;

	private:
		// Member variables
		Property<double> _kpProp;
		Property<double> _kdProp;
		Property<SimTK::Vector> _referenceValuesProp;

		// Declare private member function for setting up properties
		void constructProperties();

	}; // end of class PDController

} // end of namespace OpenSim

#endif // OPENSIM_PD_CONTROLLER_H_

