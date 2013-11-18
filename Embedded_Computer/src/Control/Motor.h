#ifndef MOTOR_H
#define MOTOR_H

#include <memory> // shared_ptr

class Motor
{
    public:  
        Motor(std::share_ptr<STM32F4> stm32f4, std::string name, int id, int offset, int min, int max, int playTime);
        ~Motor();
        void setPos(double pos);
        const double getPos();
        const double getMinAngle();
        const double getMaxAngle();

    private:
        int Angle2Value(const double angle);
        double Value2Angle(const int value);

        std::share_ptr<STM32F4> _stm32f4; 
        std::string _name;
        int _id;
        int _offset;
        int _min;
        int _max;
        int _speed;
        int _playTime;
        double _angle;
};
#endif  //MOTOR_H

