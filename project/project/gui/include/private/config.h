#pragma once

#include <yaml-cpp/yaml.h>

class Configurable {
public:
    Configurable(
        YAML::Node const & node
    ):
    mNode(node)
    {

    }

    YAML::Node mNode;
};

