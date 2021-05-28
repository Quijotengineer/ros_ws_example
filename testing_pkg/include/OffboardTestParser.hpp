#pragma once
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

class OffboardTestParser
{
    public:
    OffboardTestParser();
    ~OffboardTestParser();

    std::vector<float> getTestSetpoints();
    std::vector<float> test_setpoints_vec;

    protected:
    private:
    
};