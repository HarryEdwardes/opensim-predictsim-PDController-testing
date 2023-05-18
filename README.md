# opensim-predictsim-PDController-testing
## Overview

This repository contains code used in my final year project 'Optimal Trajectory Generation for a Human Robot System Compensating for Impaired Gait'.

**Knee Model Tests** contains various basic knee models for use in OpenSim with variable complexity. They should be used for actuator testing. knee_model.cpp is the most basic, followed by knee_model_v2.cpp which integrates muscle functionality to the model, followed by knee_model_v3.cpp which integrates ligament functionality.

**Control Systems** contains four different control strategies for the torque actuator and the torque actuator initialisation itself. The control strategies are: bang-bang, sinusoidal feed-forward, proportional-derivative, and proportional-derivative with added variability/error to add realism.

**PD Controller** contains a header and a corresponding body file for defining a PD Controller for use in external functions. It is to be applied when rebuilding the OpenSim source code. There is also a test case for testing correct initialisation.

**External Functions** contains various external functions for use in Antoine Falisse's 3dPredictSim. They lay the foundation for development of PD Controlled torque actuator functionality.
