#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H


class EmergencyStop
{
    public:
        EmergencyStop();
        virtual ~EmergencyStop();

        void StartEmergencyButton();
    protected:
    private:
        void initBtnPollingTimer();
};

#endif // EMERGENCYSTOP_H
