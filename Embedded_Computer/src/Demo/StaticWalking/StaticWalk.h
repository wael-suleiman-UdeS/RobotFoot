/**
******************************************************************************^M
* @file    StaticWalk.h
* @author  Mathieu Drapeau
* @date    2013-09-20
* @brief   Class to test mouvement on robot
******************************************************************************^M
*/
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
    
    // Initialize configuration of mouvement
    void init(const std::string filename, const bool isUsingAlgorithm, const bool isMotorActivated, const bool isStandAlone);

    // Move robot to initial position
    void initPosition(const int msInitializationTime);
    // Start the mouvement
    void run(double uDt);

private:
    // Use Algorithm if true and text file input if false to generate mouvement
    bool bIsUsingAlgorithm;
    // Activated motor if true
    bool bIsMotorActivated;
    // Activated stand alone mode without multithreading
    bool bIsStandAlone;

    // Contains motor position from the text file
    std::vector<std::vector<double>> vPosition;
    std::vector<std::vector<double>>::iterator itrPos;
    std::vector<std::vector<double>>::iterator itrEnd;
	
    std::shared_ptr<ThreadManager> _threadManager;
    std::shared_ptr<MotorControl> _motion;
};
