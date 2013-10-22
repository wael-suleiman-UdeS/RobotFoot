#include "../../Utilities/logger.h"
#include "../../Control/STM32F4.h"
#include "../../Control/MotorControl_2.h"

#include <string>
#include <vector>

class StaticWalk
{
public:

    StaticWalk(STM32F4* mc);
    ~StaticWalk();

    void init(const std::string filename, const bool isUsingAlgorithm, const bool isMotorActivated);

    void initPosition(const int msInitializationTime);
    void run(const double uDt);

private:
    bool bIsUsingAlgorithm;
    bool bIsMotorActivated;

    std::vector<std::vector<double>> vPosition;
    std::vector<std::vector<double>>::iterator itrPos;
    std::vector<std::vector<double>>::iterator itrEnd;
	
    MotorControl *motion;
    ThreadManager *_threadManager;
};
