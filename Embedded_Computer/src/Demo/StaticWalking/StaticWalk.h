#include "../../Utilities/logger.h"
#include "../../Control/STM32F4.h"
#include "../../Control/MotorControl_2.h"
#include "Utilities/ThreadManager.h"

#include <string>
#include <vector>
#include <memory> // shared_ptr

class StaticWalk
{
public:

    StaticWalk(std::shared_ptr<ThreadManager> threadManager_ptr, std::shared_ptr<MotorControl> mc_ptr);
    ~StaticWalk();

    void init(const std::string filename, const bool isUsingAlgorithm, const bool isMotorActivated, const bool isStandAlone);

    void initPosition(const int msInitializationTime);
    void run(double uDt);

private:
    bool bIsUsingAlgorithm;
    bool bIsMotorActivated;
    bool bIsStandAlone;

    std::vector<std::vector<double>> vPosition;
    std::vector<std::vector<double>>::iterator itrPos;
    std::vector<std::vector<double>>::iterator itrEnd;
	
    std::shared_ptr<ThreadManager> _threadManager;
    std::shared_ptr<MotorControl> _motion;
};
