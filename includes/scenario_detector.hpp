#pragma once

enum class Scenario
{
    None,
    S2
};

class ScenarioDetector
{
public:
    Scenario check();
};