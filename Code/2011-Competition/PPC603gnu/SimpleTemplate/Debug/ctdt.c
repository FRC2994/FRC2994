/* ctors and dtors arrays -- to be used by runtime system */
/*   to call static constructors and destructors          */
/*                                                        */
/* NOTE: Use a C compiler to compile this file. If you    */
/*       are using GNU C++, be sure to use compile this   */
/*       file using a GNU compiler with the               */
/*       -fdollars-in-identifiers flag.                   */


void _GLOBAL__I__Z20FRC_userClassFactoryv();

void _GLOBAL__I__ZN19DashboardDataSenderC2Ev();

void _GLOBAL__I__ZN9ErrorBase16globalErrorMutexE();

void _GLOBAL__I__ZN9RobotBase10m_instanceE();

void _GLOBAL__I__ZN14SmartDashboard11BUFFER_SIZEE();

void _GLOBAL__I__ZN10Ultrasonic9kPingTimeE();

void _GLOBAL__I_AxisCamera_debugFlag();

extern void (*_ctors[])();
void (*_ctors[])() =
    {
    _GLOBAL__I__Z20FRC_userClassFactoryv,
    _GLOBAL__I__ZN19DashboardDataSenderC2Ev,
    _GLOBAL__I__ZN9ErrorBase16globalErrorMutexE,
    _GLOBAL__I__ZN9RobotBase10m_instanceE,
    _GLOBAL__I__ZN14SmartDashboard11BUFFER_SIZEE,
    _GLOBAL__I__ZN10Ultrasonic9kPingTimeE,
    _GLOBAL__I_AxisCamera_debugFlag,
    0
    };

void _GLOBAL__D__Z20FRC_userClassFactoryv();

void _GLOBAL__D__ZN19DashboardDataSenderC2Ev();

void _GLOBAL__D__ZN9ErrorBase16globalErrorMutexE();

void _GLOBAL__D__ZN9RobotBase10m_instanceE();

void _GLOBAL__D__ZN10Ultrasonic9kPingTimeE();

void _GLOBAL__D_AxisCamera_debugFlag();

extern void (*_dtors[])();
void (*_dtors[])() =
    {
    _GLOBAL__D__Z20FRC_userClassFactoryv,
    _GLOBAL__D__ZN19DashboardDataSenderC2Ev,
    _GLOBAL__D__ZN9ErrorBase16globalErrorMutexE,
    _GLOBAL__D__ZN9RobotBase10m_instanceE,
    _GLOBAL__D__ZN10Ultrasonic9kPingTimeE,
    _GLOBAL__D_AxisCamera_debugFlag,
    0
    };
