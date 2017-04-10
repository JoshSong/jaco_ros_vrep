#include <iostream>
#include <dlfcn.h> //Ubuntu
#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <unistd.h>
//Note that under windows, you may/will have to perform other #include
using namespace std;
int main()
{
        int result;
        int programResult = 0;
        //Handle for the library's command layer.
        void * commandLayer_handle;
        //Function pointers to the functions we need
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyGetAngularCommand)(AngularPosition &);
        int(*MyGetAngularPosition)(AngularPosition &);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        int(*MyGetActuatorAcceleration)(AngularAcceleration &Response);
        int(*MyGetAngularVelocity)(AngularPosition &Response);
        int(*MyMoveHome)();
        int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
        int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
        int(*MySetTorqueSafetyFactor)(float factor);
        int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
        int(*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);
        int(*MySetGravityVector)(float Command[3]);
        int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
        int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
        int(*MySetGravityType)(GRAVITY_TYPE Type);
        int(*MySetTorqueVibrationController)(float value);
        int(*MyGetAngularForceGravityFree)(AngularPosition &);
        int(*MyGetCartesianForce)(CartesianPosition &);
        int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
        //We load the library (Under Windows, use the function LoadLibrary)
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);
        MyInitAPI = (int(*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyGetAngularCommand = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MyGetAngularPosition = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularPosition");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        MyGetActuatorAcceleration = (int(*)(AngularAcceleration &)) dlsym(commandLayer_handle, "GetActuatorAcceleration");
        MyGetAngularVelocity = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularVelocity");
        MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        MySendCartesianForceCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendCartesianForceCommand");
        MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
        MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
        MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
        MyGetCartesianForce = (int(*)(CartesianPosition &)) dlsym(commandLayer_handle, "GetCartesianForce");
        MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
        MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
        MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) || (MyGetAngularPosition == NULL)
                || (MySetActiveDevice == NULL) || (MyGetDevices == NULL) || (MyGetAngularVelocity == NULL))
        {
                cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
                programResult = 0;
        }
        else
        {
                cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
                result = (*MyInitAPI)();
                int resultComm;
                AngularPosition DataCommand;
                // Get the angular command to test the communication with the robot
                resultComm = MyGetAngularCommand(DataCommand);
                cout << "Initialization's result :" << result << endl;
                cout << "Communication result :" << resultComm << endl;
                // If the API is initialized and the communication with the robot is working
                if (result == 1 && resultComm == 1)
                {
                        cout << "API initialization worked" << endl;
                        cout << "The robot will swich to torque control mode and move. Be cautious." << endl;
                        // Set to position mode
                        MySwitchTrajectoryTorque(POSITION);
                        // Move to home position
                        //MyMoveHome();

                        AngularPosition TorqueFree;
                        MyGetAngularForceGravityFree(TorqueFree);
                        CartesianPosition CartForce;
                        MyGetCartesianForce(CartForce);

                        // Set the Optimal parameters obtained from the identification sequence
                        float OptimalParam[OPTIMAL_Z_PARAM_SIZE] = {
                                1.2117,
                                0.0110096,
                                -0.016218,
                                -1.18155,
                                0.00158937,
                                0.589306,
                                0.006256,
                                0.194219,
                                -0.00426873,
                                -0.0128789,
                                0.138244,
                                -0.353219,
                                0.0161938,
                                0.0333042,
                                0.0387776,
                                0.0133606,
                        };

                        MySetGravityOptimalZParam(OptimalParam);
                        // Set gravity type to optimal
                        MySetGravityType(OPTIMAL);
                        usleep(20000);
                        // Get and print the Torques and Forces (with gravity removed)
                        MyGetAngularForceGravityFree(TorqueFree);
                        MyGetCartesianForce(CartForce);
                        cout << "*********************************" << endl;
                        cout << "*** Optimal mode gravity estimation" << endl;
                        cout << "Actuator 1   Torque : " << TorqueFree.Actuators.Actuator1 << "°" << "     Force X : " << CartForce.Coordinates.X << endl;
                        cout << "Actuator 2   Torque : " << TorqueFree.Actuators.Actuator2 << "°" << "     Force Y : " << CartForce.Coordinates.Y << endl;
                        cout << "Actuator 3   Torque : " << TorqueFree.Actuators.Actuator3 << "°" << "     Force Z : " << CartForce.Coordinates.Z << endl;
                        cout << "Actuator 4   Torque : " << TorqueFree.Actuators.Actuator4 << "°" << "     Torque X : " << CartForce.Coordinates.ThetaX << endl;
                        cout << "Actuator 5   Torque : " << TorqueFree.Actuators.Actuator5 << "°" << "     Torque Y : " << CartForce.Coordinates.ThetaY << endl;
                        cout << "Actuator 6   Torque : " << TorqueFree.Actuators.Actuator6 << "°" << "     Torque Z : " << CartForce.Coordinates.ThetaZ << endl << endl;

                        // Set the torque control type to Direct Torque Control
                        MySetTorqueControlType(DIRECTTORQUE);
                        // Set the safety factor to 0.6
                        MySetTorqueSafetyFactor(0.6);
                        // Set the vibration controller to 0.5
                        MySetTorqueVibrationController(0.5);
                        // Switch to torque control
                        // (Here we switch before sending torques. The switch is possible because the gravity torques are already taken into account.)
                        MySwitchTrajectoryTorque(TORQUE);
                        // Initialize the torque commands
                        float TorqueCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                TorqueCommand[i] = 0;
                        }
                        // Send the torque commands for 2 seconds
                        /*
                        for (int i = 0; i < 200; i++)
                        {
                                // Torque of 0.5Nm on joint 6
                                TorqueCommand[5] = 0.7;
                                // Send the torques
                                MySendAngularTorqueCommand(TorqueCommand);
                                // Sleep 10 ms
                                usleep(10000);
                        }
                        */
                        // Wait
                        usleep(1000000);
                        cout << "START" << endl;
                        // Initialize the Cartesian force commands
                        float CartForceCommand[COMMAND_SIZE];
                        for (int i = 0; i < COMMAND_SIZE; i++)
                        {
                                CartForceCommand[i] = 0;
                        }
                        // Send the force commands for x seconds
                        for (int i = 0; i < 3000; i++)
                        {
                                CartForceCommand[2] = -2;
                                // Send the force command
                                MySendCartesianForceCommand(CartForceCommand);
                                // Sleep of 10 ms
                                usleep(10000);
                        }
                        cout << "STOP" << endl;

                        //Wait
                        usleep(2000000);
                        // Switch back to position
                        MySwitchTrajectoryTorque(POSITION);
                }
                else
                {
                        cout << "API initialization failed" << endl;
                }
                cout << endl << "C L O S I N G   A P I" << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }
        dlclose(commandLayer_handle);
        return programResult;
}
