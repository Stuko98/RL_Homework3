#include "kdl_ros_control/kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)                                                                         //Costruttore con argomenti
{
    robot_ = &_robot;
}


Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,                                                              //InverseDynamicsController nello spazio dei giunti ("Robotics" di Siciliano -> cap 8 -> 8.5.2). Usato a lezione 10. Ritorna un oggetto di classe "Eigen::VectorXd", cioè un vettore di dimensione nota solo a run-time di elementi "double"
                                      KDL::JntArray &_dqd,         
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();                                                                        //current q, joint position
    Eigen::VectorXd dq = robot_->getJntVelocities();                                                                   //current dq, joint velocity

    // calculate errors 
    Eigen::VectorXd e = _qd.data - q;                                                                                  //q_desired - q
    Eigen::VectorXd de = _dqd.data - dq;                                                                               //q_dot_desired - q_dot
    Eigen::VectorXd ddqd = _ddqd.data;                                                                                 //q_double_dot desired

    // control law
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)                                                                 //ritorno la legge di controllo "u", cioè le torques da applicare ai giunti: B(q) * y + n(q,q_dot) = B(q) * y + C(q,q_dot) * q_dot + F * q_dot + g(q).
            + robot_->getCoriolis() + robot_->getGravity()/*friction compensation?*/;                                  //Non usiamo la componente "F*q_dot"
}


////////////////////////////////////////////////////////////////////////////////
//                              HOMEWORK2                                     //
////////////////////////////////////////////////////////////////////////////////


Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,                                                             //InverseDynamicsController nello spazio operativo ("Robotics" di Siciliano -> cap 8 -> 8.6.3)
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // calculate gain matrices (matrici diagonali)
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();                                                              //Eigen::DenseBase::block: metodo che permette di trattare una matrice tramite blocchi. Parametri: 1° riga da cui partire per considerare il blocco, 1° colonna, # righe del blocco, # colonne del blocco. In questo caso quindi stiamo considerando come blocco le prime tre righe e colonne (parte posizionale di Kp) e lo stiamo riempiendo con la matrice identità
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();                                                              //parte rotazionale di Kp
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();                                                              //parte posizionale di Kd
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();                                                              //parte rotazionale di Kp


    // read current cartesian state 
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;                                                        //Jacobiano geometrico J(q) all'attuale configurazione q. "getEEJacobian()" ritorna un elemento di tipo "KDL::Jacobian". "getEEJacobian().data" restituisce la componente matriciale (di tipo "Eigen::Matrix") di tale elemento. La componente matriciale è fondamentale per svolgere le operazioni matematiche che servono per elaborare la legge di controllo   //NON ESISTE NESSUNA FUNZIONE "getJacobian" -> compilatore mi ha suggerito "getEEJacobian". Errore conversione "KDL::Jacobian -> Eigen::Matrix" -> il campo "data" di KDL::Jacobian è un elemento di tipo "Eigen::Matrix"
    Eigen::Matrix<double,7,7> M = robot_->getJsim();                                                                   //Inertia Matrix "B(q)" all'attuale configurazione q. Lui la chiama M.
    //Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);                                                    //Jacobian Pseudo-inverse, ma calcolata non con il metodo classico (Moore-Penrose), ma un altro. Definita in "utils.h"
    Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);                                                                //Jacobian Pseudo-inverse (classico, quello di Moore-Penrose). Definita in "utils.h"


    // position (desired + current)
    Eigen::Vector3d p_d(_desPos.p.data);                                                                               //desired position
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data);                                                                  //end effector current position  //NON ESISTE NESSUNA FUNZIONE "getCartesianPose()" -> L'unico metodo che mi restituisce un Frame è "getEEFrame()" in "kdl_robot.h"    
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);                                                     //desired rotation
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);                                        //end effector rotation
    R_d = matrixOrthonormalization(R_d);                                                                               //Ortonormalizzazioni
    R_e = matrixOrthonormalization(R_e);

    // velocity (desired + current)
    Eigen::Vector3d dot_p_d(_desVel.vel.data);                                 
    Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);                                                         //NON ESISTE NESSUNA FUNZIONE "getVelocityPose()" -> L'unico metodo che mi restituisce un Twist è "getEEVelocity()" in "kdl_robot.h"    
    Eigen::Vector3d omega_d(_desVel.rot.data);
    Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);

    // acceleration (desired)
    Eigen::Matrix<double,6,1> dot_dot_x_d;
    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

    // compute linear errors (position + velocity)
    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);                                                       //"utils.h"
    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);                                           //"utils.h"

    // compute orientation errors (position + velocity)                                        
    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);                                                  //"utils.h"
    Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,                                       //"utils.h"
                                                                        omega_e,
                                                                        R_d,
                                                                        R_e);

    // calculate errors                                                                   
    Eigen::Matrix<double,6,1> x_tilde;                                                 
    Eigen::Matrix<double,6,1> dot_x_tilde;
    x_tilde << e_p, e_o;
    dot_x_tilde << dot_e_p, dot_e_o;                                                                                   //sbagliato: "-omega_e" -> "dot_e_o"
    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;
   
    //Define J_dot
    Eigen::Matrix<double,6,7> Jdot = robot_->getEEJacDot(); 
   
    //Compute Analitycal Jacobian Ja=Ta^(-1)*J                                         
    Eigen::Matrix<double,3,1> euler = computeEulerAngles(R_e);                                                         //"utils.h"                       
    Eigen::Matrix<double,6,7> JA = AnalitycalJacobian(J,euler);                                                        //"utils.h"
    Eigen::Matrix<double,7,6> JApinv = pseudoinverse(JA);                                                              //"utils.h"

    // Compute TA_inv
    //Eigen::Matrix<double,6,> TA_inv = JA * J.inverse();
    Eigen::Matrix<double,6,6> TA;                               
    TA.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    TA.block(3,3,3,3) = T_matrix(euler);                                                                               //"utils.h"
    Eigen::Matrix<double,6,6> TA_inv = TA.inverse();

    // Compute TA_dot
    Eigen::Matrix<double,6,6> TA_dot;
    TA_dot.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    TA_dot.block(3,3,3,3) = Tdot_matrix(euler);                                                                        //"utils.h"

    // Compute JA_dot 
    Eigen::Matrix<double,6,7> JAdot = TA_inv*(Jdot - TA_dot*JA);                                                       //J=Ta*Ja  -->  J_dot= Ta_dot*Ja + Ta*Ja_dot  -->  Ja_dot= Ta^(-1)*(J_dot-Ta_dot*Ja)

    // control law
    Eigen::Matrix<double,7,1> y;

    //y << JApinv*(dot_dot_x_d - JAdot*robot_->getJntVelocities() + Kd*dot_x_tilde + Kp*x_tilde);
      y << Jpinv * (dot_dot_x_d - Jdot*robot_->getJntVelocities() + Kd*dot_x_tilde + Kp*x_tilde);

    return M * y 
            + robot_->getGravity() + robot_->getCoriolis();


//  // null space control
//  double cost;
//  Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);                    //gradientJointLimits: metodo in "utils.h" --- getJntValues, getJntLimits: metodo creato in "kdl_robot.h" e "".cpp" 

          
//  // control law
//  return M * (Jpinv*y + (I-Jpinv*J)*(/*- 10*grad */- 1*robot_->getJntVelocities()))
//          + robot_->getGravity() + robot_->getCoriolis();


}





//CONTROLLORE SOLO POSIZIONALE
Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp,
                                      double _Kdp)
{
    // calculate gain matrices
    Eigen::Matrix<double,3,3> Kp, Kd;  
    Kp = _Kpp*Eigen::Matrix3d::Identity(); 
    Kd = _Kdp*Eigen::Matrix3d::Identity();
    

    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;                                                        //parte posizionale Jacobiano Analitico == parte posizionale Jacobiano geometrico 
    Eigen::Matrix<double,3,7> J_red = J.topRows(3);                                                                    //J_reduced = parte posizionale di J(q)
    Eigen::Matrix<double,7,7> M = robot_->getJsim();
    Eigen::Matrix<double,7,3> Jpinv = pseudoinverse(J_red);

    // position
    Eigen::Vector3d p_d(_desPos.p.data);
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data);

    // velocity
    Eigen::Vector3d dot_p_d(_desVel.vel.data);
    Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);


    // acceleration
    Eigen::Matrix<double,3,1> dot_dot_x_d;
    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);

    // compute linear errors
    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

    // calculate errors                                                                       
    Eigen::Matrix<double,3,1> x_tilde;
    Eigen::Matrix<double,3,1> dot_x_tilde;
    x_tilde << e_p;
    dot_x_tilde << dot_e_p;
    dot_dot_x_d << dot_dot_p_d;

    // control law
    Eigen::Matrix<double,7,1> y;

    y << Jpinv * (dot_dot_x_d - robot_->getEEJacDotqDotPos() + Kd*dot_x_tilde + Kp*x_tilde);      

    return M * y
            + robot_->getGravity() + robot_->getCoriolis();  
}


////////////////////////////////////////////////////////////////////