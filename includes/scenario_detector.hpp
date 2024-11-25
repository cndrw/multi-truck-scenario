#pragma once

enum class Scenario
{
    None,
    S1,
    S2
};

class ScenarioDetector
{
public:
    Scenario check();
};