#include "vex.h"

namespace doon_pid {
    class PID{
        private:
            double kp, ki, kd, derv, intg, e_margin, i_margin, c_domain;
            double target, output, prevoutput;
            double currentPos, error, preverror, totalerror;
        public:
            bool inputIsDrivetrain;


            PID(double target, double currentPos, double errorMargin, double integralMargin);
        
            double tick();
            void reset();
            void setConsts(double sP, double sI, double sD, double error_margin, double integral_margin, double clamp_domain);
    };
}

//using namespace DOON_PID