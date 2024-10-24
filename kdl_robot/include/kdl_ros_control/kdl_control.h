#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);                                                                  //Costruttore con argomenti   

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,                                                        //InverseDynamicsController (nello spazio dei giunti).
                           KDL::JntArray &_dqd,                                                       //2 definizioni del controller. Questa è quella fatta a "Lezione 10" con il prof
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

////////////////////////////////////////////////////////////////////////////////
//                              HOMEWORK2                                     //
////////////////////////////////////////////////////////////////////////////////

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,                                                       //InverseDynamicsController nello spazio operativo ("Robotics" di Siciliano -> cap 8 -> 8.6.3)
                           KDL::Twist &_desVel,                                                       
                           KDL::Twist &_desAcc, 
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);


Eigen::VectorXd idCntr(KDL::Frame &_desPos,                                                          //InverseDynamicsController nello spazio operativo DELLA SOLA PARTE POSIZIONALE
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp,
                                      double _Kdp);

////////////////////////////////////////////////////////////////////////////////                                   

private:

    KDLRobot* robot_;

};

#endif
