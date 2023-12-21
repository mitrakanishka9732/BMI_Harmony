/**
 * @file bmi_exercise.cpp
 * @brief receive a UDP input from EEG cap and perform an exercise!
 * @version 0.2
 */

/******************************************************************************************
 * INCLUDES
 *****************************************************************************************/
#include "research_interface.h"
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <ctime>
#include <sstream>

// UDP include
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#define PI 3.141592653
#define DEG_2_RAD PI / 180
#define RAD_2_DEG 180 / PI

// UDP define
#define PORT 8080
#define MAX_BUFFER_SIZE 1024

#define s400Stiffness_Nm_p_rad 15.0 // desired joint stiffness for series 400 [shoulder] motors(max is 50)
#define s600Stiffness_Nm_p_rad 15.0 // desired joint stiffness for series 600 [elbow] motors (max is 30)
#define s700Stiffness_Nm_p_rad 1.5 // desired joint stiffness for series 700 [wrist] motors(max is 3)

#ifdef WRIST_MODS
#define WD_SCALING 1.0
#define WF_SCALING 0.33
#endif

#ifdef TORSO_MODS
#define nCols 2 * harmony::armJointCount + harmony::torsoJointCount + 1 // number of columns in dataset
#else
#define nCols 2 * harmony::armJointCount + 1 // number of columns in dataset
#endif

// Buffer Times 
int ImpedenceBufferTime_s = 4;
int startPosBufferTime_s = 5;
int beginExBufferTime_s = 6; 
int waitBufferTime_s = 3;
int back2StartBufferTime_s = 2;
int waitBufferTime2_s = waitBufferTime_s;
 
/******************************************************************************************
 * Structs
 *****************************************************************************************/
struct AllArmsOverrides {
    harmony::ArmJointsOverride leftOverrides;
    harmony::ArmJointsOverride rightOverrides;
#ifdef TORSO_MODS
    harmony::TorsoJointsOverride torsoOverrides;
#endif
};

/**
 * @brief Prints the start script before writing to harmony
 *
 * @param fs sample frequency from logfile
 */
void printStart(double fs) {
    // std::cout << "stiffness : " << stiffness_Nm_p_rad << " Nm per rad\n";
    std::cout << "fs : " << fs << " Hz\n";
    std::cout << "Starting in";
    std::cout.flush();

    for (int i = 3; i >= 1; i--) {
        std::cout << "..." << i;
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "\n--> LETS EXERCISE <--\n";
}

std::string getCurrentDateTime() {
    std::time_t now = std::time(nullptr);
    std::tm* ptm = std::localtime(&now);

    std::stringstream ss;
    // ss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S");
    ss << std::put_time(ptm, "%Y_%m_%d");
    return ss.str();
}

/**
 * @brief Writes the header for the log file
 * The first line of the log file is the sampling frequency,
 * the second line is the column names
 * the nth line is the nth recorded joint position vector with
 * respect to time
 * @param logFile the ofstream object for the file to be written
 * @param fs the sampling frequency in Hz
 */
void printLogHeader(std::ofstream* logFile, double fs) {
    *logFile << "TIME\tITERATION\tMOV\tTRIGGER";
    for (int i=0; i<harmony::armJointCount; i++){ *logFile << "\tleft_j" << i;  }
    for (int i=0; i<harmony::armJointCount; i++){ *logFile << "\tright_j" << i; }
    *logFile << "\tl_end_pos_x";
    *logFile << "\tl_end_pos_y";
    *logFile << "\tl_end_pos_z";
    *logFile << "\tr_end_pos_x";
    *logFile << "\tr_end_pos_y";
    *logFile << "\tr_end_pos_z";
    *logFile << "\n";

#ifdef TORSO_MODS
    for (int i=0; i<harmony::torsoJointCount; i++){ *logFile << "torso_j" << i << ", "; }
#endif
    *logFile << std::endl;
}


/**
 * @brief Convert a given file prefix to a log filename, including path.
 *  Makes all files end in a '-log.txt'
 * @param filePrefix the desired file prefix
 * @return std::string logfilename
 */
std::string filepath(std::string filePrefix) {
    return "./log/" + filePrefix + "_log.txt";
}

// /**
//  * @brief Log the data to a log file
//  *
//  * @param logfile is the ofstream object for logging
//  * @param states is the read joint states of the object
//  */
// void logData(std::ofstream* logfile, std::array<harmony::JointState, harmony::armJointCount> states) {
//     std::time_t now = std::time(nullptr);
//     std::tm* ptm = std::localtime(&now);

//     std::stringstream ss;
//     ss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");

//     *logfile << ss.str() << " ";

//     for (int i = 0; i < harmony::armJointCount; i++) {
//         *logfile << states[i].position_rad * RAD_2_DEG << ", ";
//     }
// }

// Function to save data in the log file
void saveDataInLogFile(std::ofstream* logFile, harmony::ResearchInterface* info, int iteration, char movement, std::string trigger_type) {
    // Continuously write data to the log file
    // while (true) {
        // Get the current timestamp

        std::array<harmony::JointState, harmony::armJointCount> states_left =  info->joints().leftArm.getOrderedStates();
        std::array<harmony::JointState, harmony::armJointCount> states_right =  info->joints().rightArm.getOrderedStates();
        harmony::Pose pose_left = info->poses().leftEndEffector;
        harmony::Pose pose_right = info->poses().rightEndEffector; 

        using namespace std::chrono;

        // auto now = std::chrono::system_clock::now();
        // date::format("%T", std::chrono::floor<std::chrono::milliseconds>(now));
        auto now = std::chrono::system_clock::now();
        auto timePoint = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::tm* timeinfo = std::localtime(&timePoint);

        *logFile << std::put_time(timeinfo, "%H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count() << "\t" << iteration <<"\t" << movement << "\t" << trigger_type;

        // Generate some sample data to save in the log file
        for (int i = 0; i < harmony::armJointCount; i++) { *logFile << "\t" << states_left[i].position_rad * RAD_2_DEG;}
        for (int i = 0; i < harmony::armJointCount; i++) { *logFile << "\t" << states_right[i].position_rad * RAD_2_DEG;}
        *logFile << "\t" << pose_left.position_mm.x;
        *logFile << "\t" << pose_left.position_mm.y;
        *logFile << "\t" << pose_left.position_mm.z;
        
        *logFile << "\t" << pose_right.position_mm.x;
        *logFile << "\t" << pose_right.position_mm.y;
        *logFile << "\t" << pose_right.position_mm.z;
        *logFile << "\n";

        *logFile << std::endl;

        // std::string data = "Sample data " + states[i].position_rad * RAD_2_DEG;

        // Save the data in the log file
        // logFile << timestampStr << " - " << data << std::endl;

        // Sleep for some time (e.g., 1 second)
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
}



// /**
//  * @brief Locks the user in place to start recording
//  *
//  * @return true if the user hits [return]
//  * @return false if the user hits [q]
//  */
// bool waitToStart() {
//     std::string usr_input;

//     // lambda function to check if valid response
//     auto valid_response = [&]() -> bool { return usr_input.length() == 0 || usr_input == "q"; };

//     do {
//         std::cout << "Enter [return] to start exercise or [q] to quit. ";
//         std::getline(std::cin, usr_input);
//     } while (!valid_response());

//     bool startRecording = usr_input.length() == 0;
//     if (!startRecording) { std::cout << "Exiting..." << std::endl; }

//     return startRecording;
// }

/**
 * @brief Choice of harmony operating side
 *
 * @return true if the user hits [0]
 * @return false if the user hits [1]
 */
bool setSideChoice() {

    char usr_input[6];

    std::cout << "Choose the side you want to exercise with!\n";
    std::cout << "r : RIGHT arm, l : LEFT arm!\n";

    std::cin >> usr_input;

    bool startRecording;

    if (usr_input[0] == 'r') {
        std::cout << "RIGHT side chosen\n";
        startRecording = true;
    } else {
        std::cout << "LEFT side chosen\n";
        startRecording = false;
    }

    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    return startRecording;
}



// /**
//  * @brief stops the exercise when user input matches with []
//  *
//  * @return true if the user hits [return]
//  * @return false if the user hits [q]
//  */
// bool stopExercise() {
//     std::string usr_input;

//     // lambda function to check if valid response
//     auto valid_response = [&]() -> bool { return usr_input == "s"; };

//     do {
//         std::cout << "Enter [s] to stop the exercise. ";
//         std::getline(std::cin, usr_input);
//     } while (!valid_response());

//     bool stopRecording;
//     if (usr_input == "s")
//     {
//         stopRecording = true;
//     }
//     else
//     {
//         stopRecording = false;
//     }

//     if (!stopRecording) { std::cout << "Exiting..." << std::endl; }

//     return stopRecording;
// }

/**
 * @brief returns the desired joint position for home mode
 */

std::array<double, nCols> setHomePosition(harmony::ResearchInterface* info) {
    std::array<double, nCols> data;

    // // std::array<double, nCols> homepositionRight_array = {0, 0, 0, 30, 45, -90, 40};
    // // std::array<double, nCols> homepositionLeft_array = {0, 0, 0, -30, -45, 90, -40};

    std::array<double, nCols> homepositionRight_array = {6.3, -2.2, -8, -6, -30, -114, 10};
    std::array<double, nCols> homepositionLeft_array = {-6.3, 2.2, 8, 6, 30, 114, -10};

    data[0] = 0.0;

    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + 1] = DEG_2_RAD * homepositionLeft_array[i];
    }
    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + harmony::armJointCount + 1] = DEG_2_RAD * homepositionRight_array[i];
    }

    return data;
}

/**
 * @brief returns the desired joint position for home mode
 */

std::array<double, nCols> setSideArmActive(harmony::ResearchInterface* info, bool side) {
    std::array<double, nCols> data;

    std::array<double, nCols> homepositionRight_array;
    std::array<double, nCols> homepositionLeft_array;

    if (side == true) {
        // RIGHT side
        homepositionRight_array = {6.3, -2.2, -8, -6, -30, -114, 10}; // home position
        homepositionLeft_array = {-6.3, 2.2, 2, -80, -88, 114, -10}; // transfer position
    } else {
        // LEFT side
        homepositionRight_array = {6.3, -2.2, -2, 80, 88, -114, 10}; // transfer position
        homepositionLeft_array = {-6.3, 2.2, 8, 6, 30, 114, -10}; // home position
    }

    data[0] = 0.0;

    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + 1] = DEG_2_RAD * homepositionLeft_array[i];
    }
    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + harmony::armJointCount + 1] = DEG_2_RAD * homepositionRight_array[i];
    }

    return data;
}

/**
 * @brief returns the desired joint position for execise mode
 */

std::array<double, nCols> setEndPoint2(harmony::ResearchInterface* info, bool side, char movement) {
    std::array<double, nCols> data;

    std::array<double, nCols> endpointRight_array;
    std::array<double, nCols> endpointLeft_array;

    if (side == true) {
        // RIGHT side
        // if     (movement == 'x') {endpointRight_array = {20, -8.6, -6.3, -21, 69, -31, 0.6};} //og
        // else if(movement == 'y') {endpointRight_array = {5.2, -4.85, -2.3, -40.6, 17.2, -77.4, -36.8};} //intra
        // else if(movement == 'z') {endpointRight_array = {7, -2.8, -27, 47.6, 40, -56.1, -19.8};} //fully extended

        // if     (movement == 'x') {endpointRight_array = {0.23, -0.096, -0.46, 0.20, 0.87, -1.09, -0.014};} //og
        // else if(movement == 'y') {endpointRight_array = {0.25, -0.11, -0.25, -0.26, 0.97, -1.04, -0.29};} //intra
        // else if(movement == 'z') {endpointRight_array = {0.43, -0.11, -0.41, 0.21, 1.29, -1.03, -0.26};} //fully extended

        if     (movement == 'x') {endpointRight_array = {0.200571, -0.0687415, -0.5702, 0.390104, 0.95567, -0.736053, 0.166412};} //og
        else if(movement == 'y') {endpointRight_array = {0.242212, -0.122677, -0.35063, -0.430419, 1.17505, -0.510246, 0.0386217};} //intra
        else if(movement == 'z') {endpointRight_array = {0.38315, -0.141666, -0.132891, 0.259636, 1.60026, -0.769595, -0.0552481};} //fully extended

        for(int i = 0; i < harmony::armJointCount; i++){endpointRight_array[i] = endpointRight_array[i] * RAD_2_DEG;}

        endpointLeft_array = {-6.3, 2.2, 2, -80, -88, 114, -10}; // keeps transfer position
    } else {
        // LEFT side
        endpointRight_array = {6.3, -2.2, -2, 80, 88, -114, 10}; // keeps transfer position
        // if     (movement == 'x') {endpointLeft_array = {-20, 8.6, 6.3, 21, -69, 31, -0.6};} //og
        // else if(movement == 'y') {endpointLeft_array = {-5.2, 4.85, 2.3, 40.6, -17.2, 77.4, 36.8};} //intra
        // else if(movement == 'z') {endpointLeft_array = {-7, 2.8, 27, -47.6, -40, 56.1, 19.8};}   //fully extended 

        if     (movement == 'x') {endpointLeft_array  = {-0.200571, 0.0687415, 0.5702, -0.390104, -0.95567, 0.736053, -0.166412};} //og
        else if(movement == 'y') {endpointLeft_array  = {-0.242212, 0.122677, 0.35063, 0.430419, -1.17505, 0.510246, -0.0386217};} //intra
        else if(movement == 'z') {endpointLeft_array  = {-0.38315, 0.141666, 0.132891, -0.259636, -1.60026, 0.769595, 0.0552481};} //fully extended

        for(int i = 0; i < harmony::armJointCount; i++){endpointLeft_array[i] = endpointLeft_array[i] * RAD_2_DEG;} 
    }

    data[0] = 0.0;

    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + 1] = DEG_2_RAD * endpointLeft_array[i];
        // data[i + 1] = endpointLeft_array[i];
    }
    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + harmony::armJointCount + 1] = DEG_2_RAD * endpointRight_array[i];
        // data[i + harmony::armJointCount + 1] = endpointRight_array[i];
    }

    return data;
}


/**
 * @brief returns the desired joint stiffness based on the actuator type
 * Joints <5 are all series 400, joint 5 is the elbow series 500, and
 * joint 6 is the wrist series 700 joint.
 * @param joint_idx current index for joint
 * @return double the desired joint stiffness
 */
double jointStiffness(int joint_idx) {
    double stiffness;
    harmony::ArmJoint joint = harmony::ArmJoint(joint_idx);

    switch (joint) {
        case harmony::ArmJoint::elbowFlexion:
            stiffness = s600Stiffness_Nm_p_rad;
            break;
        case harmony::ArmJoint::wristPronation:
            stiffness = s700Stiffness_Nm_p_rad;
            break;
#ifdef WRIST_MODS
        case harmony::ArmJoint::wristAbduction:
            stiffness = WD_SCALING * s700Stiffness_Nm_p_rad;
            break;
        case harmony::ArmJoint::wristFlexion:
            stiffness = WF_SCALING * s700Stiffness_Nm_p_rad;
            break;
#endif
        default:
            stiffness = s400Stiffness_Nm_p_rad;
            break;
    }

    return stiffness;
}

/**
 * @brief returns the desired joint stiffness based on the actuator type
 * Joints <5 are all series 400, joint 5 is the elbow series 500, and
 * joint 6 is the wrist series 700 joint.
 * @param joint_idx current index for joint
 * @param scaling is used to indirectly control the stiffnesses
 * @return double the desired joint stiffness
 */
double jointStiffness(int joint_idx, double scaling) {
    double stiffness;
    harmony::ArmJoint joint = harmony::ArmJoint(joint_idx);

    switch (joint) {
        case harmony::ArmJoint::elbowFlexion:
            stiffness = s600Stiffness_Nm_p_rad * scaling;
            break;
        case harmony::ArmJoint::wristPronation:
            stiffness = s700Stiffness_Nm_p_rad * scaling;
            break;
#ifdef WRIST_MODS
        case harmony::ArmJoint::wristAbduction:
            stiffness = WD_SCALING * s700Stiffness_Nm_p_rad * scaling;
            break;
        case harmony::ArmJoint::wristFlexion:
            stiffness = WF_SCALING * s700Stiffness_Nm_p_rad * scaling;
            break;
#endif
        default:
            stiffness = s400Stiffness_Nm_p_rad;
            break;
    }

    return stiffness;
}

/**
 * @brief Takes a data array and converts it to override format
 *
 * @param data read data line from log file, parsed to array
 * @return AllArmsOverrides studt containing left and right overrides
 */
AllArmsOverrides data2override(std::array<double, nCols> data) {
    std::array<harmony::JointOverride, harmony::armJointCount> leftOverrides;
    std::array<harmony::JointOverride, harmony::armJointCount> rightOverrides;
    for (int i = 0; i < harmony::armJointCount; i++) {
        leftOverrides[i] = {data[i + 1], jointStiffness(i)};
        rightOverrides[i] = {data[i + harmony::armJointCount + 1], jointStiffness(i)};
    }

#ifdef TORSO_MODS
    std::array<harmony::JointOverride, harmony::torsoJointCount> torsoOverrides;
    for (int i = 0; i < harmony::torsoJointCount; i++) {
        torsoOverrides[i] = {data[i + harmony::armJointCount * 2 + 1], jointStiffness(i)};
    }
#endif

    return {harmony::ArmJointsOverride(leftOverrides),
        harmony::ArmJointsOverride(rightOverrides)
#ifdef TORSO_MODS
            ,
        harmony::TorsoJointsOverride(torsoOverrides)
#endif
    };
}

/**
 * @brief Takes a data array and converts it to override format
 *
 * @param data read data line from log file, parsed to array
 * @param scaling indirectly controlls impedence values
 * @return AllArmsOverrides studt containing left and right overrides
 */
AllArmsOverrides data2override(std::array<double, nCols> data, double scaling) {
    std::array<harmony::JointOverride, harmony::armJointCount> leftOverrides;
    std::array<harmony::JointOverride, harmony::armJointCount> rightOverrides;
    for (int i = 0; i < harmony::armJointCount; i++) {
        leftOverrides[i] = {data[i + 1], jointStiffness(i, scaling)};
        rightOverrides[i] = {data[i + harmony::armJointCount + 1], jointStiffness(i, scaling)};
    }

#ifdef TORSO_MODS
    std::array<harmony::JointOverride, harmony::torsoJointCount> torsoOverrides;
    for (int i = 0; i < harmony::torsoJointCount; i++) {
        torsoOverrides[i] = {data[i + harmony::armJointCount * 2 + 1], jointStiffness(i, scaling)};
    }
#endif

    return {harmony::ArmJointsOverride(leftOverrides),
        harmony::ArmJointsOverride(rightOverrides)
#ifdef TORSO_MODS
            ,
        harmony::TorsoJointsOverride(torsoOverrides)
#endif
    };
}

void printStates(std::array<harmony::JointState, harmony::armJointCount> states) {
    for (int i = 0; i < harmony::armJointCount; i++) {
        std::cout << "joint " << i << " position (rad): " << states[i].position_rad
                  << " torque (Nm): " << states[i].torque_Nm << std::endl;
    }
    std::cout << std::endl;
}

/**
 * @brief Read info for current arm positions and return the posisitons as Joint Overrides
 *
 * @param info pointer to research interface
 * @return AllArmsOverrides object holding left, right, and torso overrides
 */
std::array<double, nCols> getCurrentArmPositionsAsDataLine(harmony::ResearchInterface* info) {
    auto leftStates = info->joints().leftArm.getOrderedStates();
    auto rightStates = info->joints().rightArm.getOrderedStates();

    std::array<double, nCols> data;

    data[0] = 0.0;

    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + 1] = leftStates[i].position_rad;
    }
    for (int i = 0; i < harmony::armJointCount; i++) {
        data[i + harmony::armJointCount + 1] = rightStates[i].position_rad;
    }

#ifdef TORSO_MODS
    auto torsoStates = info->joints().torso.getOrderedStates();
    for (int i = 0; i < harmony::torsoJointCount; i++) {
        data[i + 2 * harmony::armJointCount + 1] = torsoStates[i].position_rad;
    }
#endif

    return data;
}

/**
 * @brief Given an initial and target position, interpolate between the two
 * Moves each joint (linearly) towards the traget posiiton with the iter representing the
 * current step taken out of nSteps. This function returns AllArmsOverrides to be pushed to
 * Harmony
 * @param initialOverride Initial position as AllArmsOverrides
 * @param targetOverride Target Position as AllArmsOverrides
 * @param iter iteration of interpolation
 * @param nSteps number of steps to take between initial and target positions
 * @return AllArmsOverrides the override command per the _iter_ step
 */
std::array<double, nCols> step2targetPosition(std::array<double, nCols> start,
    std::array<double, nCols> finish,
    int iter,
    int nSteps) {

    std::array<double, nCols> step;
    step[0] = 0.;

    for (int i = 1; i < nCols; i++) {
        step[i] = start[i] + (finish[i] - start[i]) * iter / nSteps;
    }
    return step;
}

/*******UDP LOOP*********/
void UDPloop(char* buffer, int buffer_len, struct sockaddr_in cliaddr, int sockfd) {

    while (true) {
        // Receiving data from the client
        buffer_len = sizeof(cliaddr);
        int n = recvfrom(
            sockfd, (char*)buffer, MAX_BUFFER_SIZE, MSG_WAITALL, (struct sockaddr*)&cliaddr, (socklen_t*)&buffer_len);
        buffer[n] = '\0';
        std::cout << "Client: " << buffer << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// /*******SHUTDOWN LOOP*********/
// void exitLoop(harmony::ResearchInterface* info, int sockfd){

//     char usr_input[6];

//     std::cin >> usr_input;

//     // int stopProgram;

//     auto left = info->makeLeftArmController(); // left arm controller
//     auto right = info->makeRightArmController(); // right arm controller

//     if(usr_input[0] == 'q')
//     {
//         std::cout << usr_input[0] <<std::endl;
//         std::cout << "Exiting...\n";

//         left->removeOverride();
//         right->removeOverride();
// #ifdef TORSO_MODS
//         torso->removeOverride();
// #endif
//         close(sockfd);

//         // stopProgram = 0;

//         exit(0);
//     }
//     else
//     {
//         // stopProgram = 1;
//     }

//     // return stopProgram;

// }

int main() {

    double fs = 200; // recording frequency
    uint T_ms = uint(1000 / fs);
    ; // recording time step

    //     /*--------- Init Research Interface --------*/
    harmony::ResearchInterface info;
    if (!info.init()) {
        std::cerr << "Failed to initialize Research Interface" << std::endl;
        return -1;
    } else {
        std::cout << "Research interface initialized!" << std::endl;
    }

#ifdef TORSO_MODS
    auto torso = info.makeTorsoController();
    if (!torso->init()) {
        std::cerr << "Failed to initialize Torso Controller" << std::endl;
        return -1;
    } else {
        std::cout << "Torso Controller initialized!" << std::endl;
    }
#endif

    auto left = info.makeLeftArmController(); // left arm controller
    auto right = info.makeRightArmController(); // right arm controller
    if (!left->init() || !right->init()) {
        std::cerr << "Failed to initialize Arm Controllers" << std::endl;
        return -1;
    }

    // UDP socket initialization

    int sockfd, buffer_len;
    char buffer[MAX_BUFFER_SIZE];
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "socket creation failed" << std::endl;
        return 1;
    } else {
        std::cout << "Socket succesflly created!" << std::endl;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "bind failed" << std::endl;
        return 1;
    } else {
        std::cout << "Binded succesflly!" << std::endl;
    }

    std::cout << "DONE\n";

    // Calling UDP Thread !!
    std::thread udpBackground(UDPloop, buffer, buffer_len, cliaddr, sockfd);
    char* UDPinput = buffer;
    udpBackground.detach();

    // //Calling SHUTDOWN Thread !!
    // std::thread exitBackground(exitLoop, &info, sockfd);
    // exitBackground.detach();

    /***************Main LooP *************/

    /*------------------Choice of the hand ------------*/
    int subjectNumber; 
    int runNumber;
    int sessionNumber;
    bool online;

    std::cout << "Enter subject number:\n";
    std::cin >> subjectNumber;

    std::cout << "Offline (0), Online (1):\n"; 
    std::cin >> online; 

    std::cout << "Enter session number:\n";
    std::cin >> sessionNumber; 

    std::cout << "Enter run number:\n";
    std::cin >> runNumber; 

    bool side = setSideChoice();
    std::string dateString = getCurrentDateTime(); 

    

    std::string filePrefix; 
    // open logFile and set header
    if (online) 
    {
        filePrefix = dateString +"_sub" + std::to_string(subjectNumber) + "_on"+std::to_string(sessionNumber)+ "_r" + std::to_string(runNumber); // file prefix specified by user
    } else 
    {
        filePrefix = dateString +"_sub" + std::to_string(subjectNumber) + "_off"+std::to_string(sessionNumber)+ "_r" + std::to_string(runNumber); // file prefix specified by user
    }
    
    std::ofstream logFile(filepath(filePrefix), std::ios::out); // ofstream object for writing
    printLogHeader(&logFile, fs);    

 /*--------- Scale Up Impedence Control --------*/
    // std::cout << "Scaling Up Impedence Control Values [" << ImpedenceBufferTime_s << "s]" << std::endl;
    std::cout << "Scaling Up Impedence Control Values" << std::endl;
    std::cout.flush();

    int nSteps = ImpedenceBufferTime_s * fs;
    auto robotStartPosition = getCurrentArmPositionsAsDataLine(&info);

    for (int i = 0; i < harmony::armJointCount; i++) {
        robotStartPosition = getCurrentArmPositionsAsDataLine(&info);
        auto overrides = data2override(robotStartPosition, i / nSteps);

        left->setJointsOverride(overrides.leftOverrides);
        right->setJointsOverride(overrides.rightOverrides);
#ifdef TORSO_MODS
        torso->setJointsOverride(overrides.torsoOverrides);
#endif
        if (i * T_ms % 1000 == 0) {
            std::cout << ".";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
    }

    /*--------- Move Harmony to start position --------*/
    std::cout << "Moving Harmony to starting position [" << startPosBufferTime_s << "s]";
    std::cout.flush();
    nSteps = startPosBufferTime_s * fs;

    std::array<double, nCols> data;
    std::array<double, nCols> prevData = robotStartPosition;
    std::array<double, nCols> exerciseStartPos = setSideArmActive(&info, side); // setHomePosition(&info)

    for (int i = 0; i <= nSteps; i++) {
        data = step2targetPosition(robotStartPosition, exerciseStartPos, i, nSteps);

        auto overrides = data2override(data);

        left->setJointsOverride(overrides.leftOverrides);
        right->setJointsOverride(overrides.rightOverrides);
#ifdef TORSO_MODS
        torso->setJointsOverride(overrides.torsoOverrides);
#endif

        if (i * T_ms % 1000 == 0) {
            std::cout << ".";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
        prevData = data;
    }

    std::cout << "DONE\n";

    if (*UDPinput == 'e') {
        *UDPinput = 0;
        std::cout << "Exit detected";
        left->removeOverride();
        right->removeOverride();

        return -1;
    }
    int iterations = 0;
    /************START OF TRIAL LOOP**********/
    for (int i = 0; i < 20; i++){
        iterations++;
        std::cout << "Waiting for exercise selection..."<<std::endl;


        while (true) {
            

            if (*UDPinput == 'x' || *UDPinput == 'y' || *UDPinput == 'z' || *UDPinput == 'e') {
                if (*UDPinput == 'e') {

                    *UDPinput = 0;
                    std::cout << "Exit detected";
                    left->removeOverride();
                    right->removeOverride();

                    return -1;

                    // Exit the loop when the desired input is received
                }

                break; 
            }
        }                

        char movement = *UDPinput;
        *UDPinput = 0;       

        std::cout << "Waiting to start exercise\n";
        // WAIT FOR UDP INPUT (g to START)
        while (*UDPinput != 'g');

        std::string trigger_type = "START";     
        saveDataInLogFile(&logFile, &info, iterations, movement, trigger_type); 
        *UDPinput = 0;

        /*--------- Begin exercise [60s] --------*/
        nSteps = beginExBufferTime_s * fs;

        robotStartPosition = prevData; // getCurrentArmPositionsAsDataLine(&info);
        exerciseStartPos = setEndPoint2(&info, side, movement);

        trigger_type = "MOVING";
        int counter = 0;

        for (int i = 0; i < nSteps; i++) {
            counter++;

            if(counter % 2 == 0){
                saveDataInLogFile(&logFile, &info, iterations, movement, trigger_type); 
            }
            

            // WAIT FOR UDP INPUT (s = stop)
            if (*UDPinput == 's') {
                std::cout << "Stop Requested\n";

                trigger_type = "STOP";
                saveDataInLogFile(&logFile, &info, iterations, movement, trigger_type); 

                *UDPinput = 0;
                prevData = data;
                break;
            }

            data = step2targetPosition(robotStartPosition, exerciseStartPos, i, nSteps);

            auto overrides = data2override(data);

            left->setJointsOverride(overrides.leftOverrides);
            right->setJointsOverride(overrides.rightOverrides);

            if (i * T_ms % 1000 == 0) {
                std::cout << ".";
                std::cout.flush();

                if (*UDPinput == 'e') {
                    *UDPinput = 0;
                    std::cout << "Exit detected";
                    left->removeOverride();
                    right->removeOverride();

                    return -1;
                }
            }

            prevData = data;

            std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
        }
        std::cout << "EXERCISE DONE\n";

        //     /*--------- Wait 5s before new command --------*/

        std::cout << "Waiting time [" << waitBufferTime_s << "s]";
        std::cout.flush();

        nSteps = waitBufferTime_s * fs;

        for (int i = 0; i <= nSteps; i++) {

            if (i * T_ms % 1000 == 0) {
                std::cout << ".";
                std::cout.flush();

                if (*UDPinput == 'e') {
                    *UDPinput = 0;
                    std::cout << "Exit detected";
                    left->removeOverride();
                    right->removeOverride();

                    return -1;
                }
            }

            // prevData = data;
            std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
        }

        std::cout << "WAIT DONE -- 1\n";

        /*--------- Move Harmony back to start position --------*/
        std::cout << "Moving Harmony back to starting position [" << back2StartBufferTime_s << "s]";
        std::cout.flush();

        nSteps = back2StartBufferTime_s * fs;

        // std::array<double, nCols> data;
        robotStartPosition = prevData; // getCurrentArmPositionsAsDataLine(&info);
        exerciseStartPos = setSideArmActive(&info, side); // setHomePosition(&info);

        for (int i = 0; i <= nSteps; i++) {
            data = step2targetPosition(robotStartPosition, exerciseStartPos, i, nSteps);

            auto overrides = data2override(data);

            left->setJointsOverride(overrides.leftOverrides);
            right->setJointsOverride(overrides.rightOverrides);
#ifdef TORSO_MODS
            torso->setJointsOverride(overrides.torsoOverrides);
#endif

            if (i * T_ms % 1000 == 0) {
                std::cout << ".";
                std::cout.flush();

                if (*UDPinput == 'e') {
                    *UDPinput = 0;
                    std::cout << "Exit detected";
                    left->removeOverride();
                    right->removeOverride();

                    return -1;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
            prevData = data;
        }

        std::cout << "WE DID IT!" << std::endl;

        /*--------- Wait 5s before new command --------*/

        std::cout << "Waiting time at home -- 2[" << waitBufferTime2_s << "s]";
        std::cout.flush();

        nSteps = waitBufferTime2_s * fs;

        for (int i = 0; i <= nSteps; i++) {

            if (i * T_ms % 1000 == 0) {
                std::cout << ".";
                std::cout.flush();

                if (*UDPinput == 'e') {
                    *UDPinput = 0;
                    std::cout << "Exit detected";
                    left->removeOverride();
                    right->removeOverride();

                    return -1;
                }
            }

            // prevData = data;
            std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
        }
        std::cout << "WAIT DONE -- 2\n";
    } // end of trial loop!    

    /*--------- Close out --------*/

    left->removeOverride();
    right->removeOverride();
#ifdef TORSO_MODS
    torso->removeOverride();
#endif
    close(sockfd);

    return 0;
}
