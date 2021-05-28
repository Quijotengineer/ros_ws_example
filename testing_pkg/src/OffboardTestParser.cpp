#include "OffboardTestParser.hpp"

// for convenience
using json = nlohmann::json;

OffboardTestParser::OffboardTestParser()
{
    std::cout << "Constructed OffboardTestParser instance" << "\n";
};

OffboardTestParser::~OffboardTestParser()
{
    std::cout << "Destroyed OffboardTestParser instance" << "\n";
};

std::vector<float> OffboardTestParser::getTestSetpoints()
{
    // Read test_setpoints.json file
    std::ifstream test_setpoints_json("./test_setpoints.json");
    auto parsed_test_setpoints = json::parse(test_setpoints_json);

    // Populate vector of setpoint coordinates
    uint num_setpoints = parsed_test_setpoints.count("Setpoints");
    for (uint sp = 0; sp < num_setpoints; sp++)
    {
        std::string sp_string = std::to_string(sp);
        // test_setpoints_vec.push_back(parsed_test_setpoints.value("Setpoints").value(sp_string).value("x","0"));
        // test_setpoints_vec.push_back(parsed_test_setpoints.value("Setpoints").value(sp_string).value("y","0"));
        // test_setpoints_vec.push_back(parsed_test_setpoints.value("Setpoints").value(sp_string).value("z","0"));

        OffboardTestParser::test_setpoints_vec.push_back(std::stof(parsed_test_setpoints["Setpoints"][sp_string].value("x","0")));
        OffboardTestParser::test_setpoints_vec.push_back(std::stof(parsed_test_setpoints["Setpoints"][sp_string].value("y","0")));
        OffboardTestParser::test_setpoints_vec.push_back(std::stof(parsed_test_setpoints["Setpoints"][sp_string].value("z","0")));
    }

    return OffboardTestParser::test_setpoints_vec;
}; 