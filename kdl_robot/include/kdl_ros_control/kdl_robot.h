#ifndef KDLROBOT
#define KDLROBOT

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>

#include "utils.h"
#include <stdio.h>
#include <iostream>
#include <sstream>

class KDLRobot
{

public:
    double epsilon_t;
    // robot
    KDLRobot();                                                                   //costruttore senza argomenti
    KDLRobot(KDL::Tree &robot_tree);                                              //costruttore con argomenti
    void update(std::vector<double> _jnt_values,std::vector<double> _jnt_vel);
    unsigned int getNrJnts();
    unsigned int getNrSgmts();
    void addEE(const KDL::Frame &_f_tip);                                         //metodo che aggiunge una trasformazione "flange frame -> e.e. frame" al nostro flange frame, in modo da ottenere posa/frame, velocità/twist ecc. relative non più al flange (parte grigia che collega il braccio robotico all'eventuale end effector che verrà aggiunto), ma all'e.e.

    // joints
    Eigen::MatrixXd getJntLimits();                                                //metodi definiti in "kdl_robot.cpp"
    Eigen::MatrixXd getJsim();                                                     //JointSpaceInertiaMAtrix: B(q)
    Eigen::MatrixXd getCoriolisMatrix();
    Eigen::VectorXd getCoriolis();
    Eigen::VectorXd getGravity();
    Eigen::VectorXd getJntValues();
    Eigen::VectorXd getJntVelocities();
    KDL::ChainIdSolver_RNE* idSolver_;
    Eigen::VectorXd getID(const KDL::JntArray &q,
                          const KDL::JntArray &q_dot,
                          const KDL::JntArray &q_dotdot,
                          const KDL::Wrenches &f_ext);

    KDL::ChainIkSolverPos_NR_JL* ikSol_;                        //Per usare metodi per ricavare InverseKinematics (cinematica inversa) basato sul metodo di iterazione di Newton-Raphson, e calcolare la position transformation from Cartesian to joint space 
    KDL::JntArray getInvKin(const KDL::JntArray &q,
                            const KDL::Frame &eeFrame);
    // end-effector
    KDL::Frame getEEFrame();                                    //ritorna un oggetto di classe KDL::Frame (vettore posa), relativo all'end effector
    KDL::Frame getFlangeEE();                                   //ritorna un oggetto di classe KDL::Frame (vettore posa), relativo alla flangia (=polso, parte che collega braccio ed end effector)
    KDL::Twist getEEVelocity();                                 //ritorna un oggetto di classe KDL::Twist (vettore twist = velocità posizionale + traslazionale), relativo all'end effector
    KDL::Twist getEEBodyVelocity();
    KDL::Jacobian getEEJacobian();                              //ritorna un oggetto di classe KDL::Jacobian (Jacobiano GEOMETRICO), relativo alla posa dell'end effector
    KDL::Jacobian getEEBodyJacobian();
    Eigen::MatrixXd getEEJacDot();                              //ritorna un oggetto di classe Eigen::MatrixXd (matrice), che rappresenta il Jacobiano_dot GEOMETRICO. //Questa funzione era "getEEJacDotqDot", ma era sbagliata. Era sbagliato anche la classe ritornata, che era un "KDL::VectorXd"

    ///////////////////////////////////HOMEWORK2/////////////////////////////
    Eigen::VectorXd getEEJacDotqDot();                          //ritorna un oggetto di classe Eigen::VectorXd (vettore), che rappresenta il prodotto J_dot*q_dot (viene usato Jacobiano GEOMETRICO)
    Eigen::VectorXd getEEJacDotqDotPos();                       //ritorna un oggetto di classe Eigen::VectorXd (vettore), che rappresenta il prodotto J_dot*q_dot PER LA SOLA PARTE POSIZIONALE (viene usato Jacobiano GEOMETRICO)
    /////////////////////////////////////////////////////////////////////////


    void getInverseKinematics(KDL::Frame &f,
                              KDL::Twist &twist,
                              KDL::Twist &acc,
                              KDL::JntArray &q,
                              KDL::JntArray &dq,
                              KDL::JntArray &ddq);

private:                                            //ATTRIBUTI

    // chain
    unsigned int n_;
    void createChain(KDL::Tree &robot_tree);
    KDL::Chain chain_;                              //Per usare metodi di "KDL::Chain" per sfruttare la catena cinematica
    KDL::ChainDynParam* dynParam_;                  //Per usare metodi come "KDL::ChainDynParam::JntToCoriolis()" ricavare matrici di inerzia B, di Coriolis C, e di gravità G 
    KDL::ChainJntToJacSolver* jacSol_;              //Per usare costruttore "KDL::ChainJntToJacSolver::ChainJntToJacSolver(chain)" per costruire il Jacobiano J(q) a partire dalla catena  
    KDL::ChainFkSolverPos_recursive* fkSol_;        //Per usare metodi per ricavare ForwardKinematics (cinematica diretta), e calcolare la posizione
    KDL::ChainFkSolverVel_recursive* fkVelSol_;     //Per usare metodi per ricavare ForwardKinematics (cinematica diretta), e calcolare la posizione e la velocità
    KDL::ChainJntToJacDotSolver* jntJacDotSol_;     //Per usare metodi per ricavare the Jacobian time derivative (Jdot) e Jdot*qdot.
    // KDL::ChainIkSolverPos_NR_JL* ikSol_;         //L'ha scritto alla riga 51 come public
    KDL::ChainIkSolverVel_wdls* ikVelSol_;

    // joints
    void updateJnts(std::vector<double> _jnt_values, std::vector<double> _jnt_vel);
    KDL::JntSpaceInertiaMatrix jsim_;                                                                //matrice di inerzia B(q)
    KDL::JntArray jntArray_;                                                                         //Costruttore classe "KDL::JntArray": una classe per array di joint values
    KDL::JntArray jntVel_;
    KDL::JntArray coriol_;
    KDL::JntArray grav_;
    KDL::JntArray q_min_;
    KDL::JntArray q_max_;

    // end-effector
    KDL::Frame f_F_ee_;             // end-effector frame in flange frame           f=flange, s=spatial, b=body          //KDL::Frame:    classe per oggetti che rappresentano una trasformazione (traslazione + rotazione) cartesiana. "f_F_ee.M" contiene la rotazione/orientazione del Frame, "f_F_ee.p" contiene la posizione/traslazione rispetto al frame base
    KDL::Frame s_F_ee_;             // end-effector frame in spatial frame
    KDL::Twist s_V_ee_;             // end-effector twist in spatial frame                                               //KDL::Twist:    classe per oggetti che rappresentano un twist (velocità traslazionale + rotazionale) in un punto
    KDL::Jacobian s_J_ee_;          // end-effector Jacobian in spatial frame                                            //KDL::Jacobian: classe per oggetti che rappreserntano uno Jacobiano (6 x ??)
    KDL::Jacobian b_J_ee_;          // end-effector Jacobian in body frame
    KDL::Jacobian s_J_dot_ee_;      // end-effector Jacobian dot in spatial frame
    KDL::Jacobian b_J_dot_ee_;      // end-effector Jacobian dot in body frame
    KDL::Twist s_J_dot_q_dot_ee_;   // end-effector Jdot*qdot in spatial frame

};

#endif
