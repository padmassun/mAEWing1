## Instructions
* If "AircraftData/ConfigurationFiles_Geri/Geri_strucmodel.mat" does not exist, rename the ".tam" file to ".mat".
*  Run "AircraftData/MakeAircraft.m" to ouput the aircraft data in a .mat file.
* The resulting .mat needs to be copied to the "/Nonlinear\_Model/Simulation_6dof/Configuration" folder
* The nonlinear simulation model run by executing "Simulation\_6dof/NL\_Sim/setup\_NL.m"

## Code structure
##### Aircraft Data Generation
* Folder "AircraftData" contains the code to generate the Steady or Unsteady aerodynamic data (using FEM data).
* It calls the Unsteady aerodynamic code in the folder "DLMcode", if required
* Aircraft specific structural and gridding files for the aircraft "Geri" are stored in "ConfigurationFiles_\Geri"

##### Nonlinear simulation
* Folder "Simulation\_6dof" contains the SIMULINK based nonlinear simulation model of the aircraft.