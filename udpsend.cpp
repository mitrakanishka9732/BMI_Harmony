#include "overrides.h"
#include "research_interface.h"
#include <iostream>
#include <sstream>
#include <vector>

std::string exeName = "";

void printUsage(std::string errorMessage) {
    if (!errorMessage.empty()) { std::cout << "Error: " << errorMessage << std::endl << std::endl; }

    std::cout << "Usage: " << exeName << " "
              << "side mode [data]" << std::endl;
    std::cout << "side values: left, right" << std::endl;
    std::cout << "mode values: harmony, joints" << std::endl;
    std::cout << "if joints is specified, data is a string of 14 whitespace delimited numbers: position stiffness (for "
                 "each of 7 joints)"
              << std::endl;
}

bool isLeftParse(std::string side) {
    if (side == "left") {
        return true;
    } else if (side == "right") {
        return false;
    } else {
        printUsage("side value must be one of: left, right");
        exit(-1);
    }
}

harmony::ArmController::Mode modeParse(std::string mode) {
    if (mode == "harmony") {
        return harmony::ArmController::Mode::harmony;
    } else if (mode == "joints") {
        return harmony::ArmController::Mode::jointsOverride;
    } else {
        printUsage("mode value must be one of harmony, joints");
        exit(-1);
    }
}

std::vector<double> splitStringToDoubles(const std::string& str) {
    std::vector<double> result;
    std::istringstream iss(str);
    double d;
    while (iss >> d) {
        result.push_back(d);
    }

    return result;
}

harmony::ArmJointsOverride parseJointsOverride(std::string data) {
    auto d = splitStringToDoubles(data);
    if (d.size() != 14) {
        printUsage("joints data must have 14 values");
        exit(-1);
    }
    return harmony::ArmJointsOverride({harmony::JointOverride{d[0], d[1]},
        harmony::JointOverride{d[2], d[3]},
        harmony::JointOverride{d[4], d[5]},
        harmony::JointOverride{d[6], d[7]},
        harmony::JointOverride{d[8], d[9]},
        harmony::JointOverride{d[10], d[11]},
        harmony::JointOverride{d[12], d[13]}});
}

int main(int argc, char** argv) {
    exeName = argv[0];

    if (argc < 3) {
        std::string errorMsg = argc == 1 ? "" : "not enough arguments";
        printUsage(errorMsg);
        exit(-1);
    }
    bool isLeft = isLeftParse(argv[1]);
    auto mode = modeParse(argv[2]);

    if (mode == harmony::ArmController::Mode::jointsOverride && argc < 4) {
        printUsage("data argument is required for joints mode");
        exit(-1);
    }

    harmony::ResearchInterface research;
    if (!research.init()) {
        std::cerr << "Failed to initialize research interface" << std::endl;
        return -1;
    }

    auto controller = isLeft ? research.makeLeftArmController() : research.makeRightArmController();
    if (!controller->init()) {
        std::cerr << "Failed to initialize arm controller" << std::endl;
        return -1;
    }

    if (mode == harmony::ArmController::Mode::jointsOverride) {
        auto override = parseJointsOverride(argv[3]);
        controller->setJointsOverride(override);
    } else { // harmony mode
        controller->removeOverride();
    }
}