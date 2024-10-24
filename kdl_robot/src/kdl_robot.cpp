#include "kdl_ros_control/kdl_robot.h"

KDLRobot::KDLRobot()                                                                                          //costruttore senza argomenti
{

}

KDLRobot::KDLRobot(KDL::Tree &robot_tree)                                                                     //costruttore con argomenti. Costruisce a partire da un elemento di tipo KDL::Tree. Dopo essere stato invocato, definisce e inizializza tutte queste cose
{
    createChain(robot_tree);                                                                                  //metodo della classe KDLRobot (da noi creato) che mi permette di creare la "chain variable" (vedi riga 79). 
    n_ = chain_.getNrOfJoints();                                                                              //metodo della classe KDL::Chain: riempio una variabile "n_" con il # di joints della chain.
    grav_ = KDL::JntArray(n_);                                                                                //dichiaro "grav_" (gravity) tramite il costruttore con argomenti JntArray, che costruisce array che contengono joint values. A che serve? Per usare metodi che di KDL::ChainDynParam che permettono di sintetizzare il vettore gravità g(q) per il robot                                    
    s_J_ee_ = KDL::Jacobian(n_);                                                                              //dichiaro matrici tramite il costruttore della classe KDL::Jacobian   
    b_J_ee_ = KDL::Jacobian(n_);
    s_J_dot_ee_ = KDL::Jacobian(n_);
    b_J_dot_ee_ = KDL::Jacobian(n_);
    s_J_ee_.data.setZero();                                                                                   //setZero(): metodo della classe Eigen::Matrix che inizializza a zero l'attributo ".data" (Eigen::Matrix) di "s_J_ee" (KDL::Jacobian) 
    b_J_ee_.data.setZero();
    s_J_dot_ee_.data.setZero();
    b_J_dot_ee_.data.setZero();
    jntArray_ = KDL::JntArray(n_);
    jntVel_ = KDL::JntArray(n_);
    coriol_ = KDL::JntArray(n_);
    dynParam_ = new KDL::ChainDynParam(chain_,KDL::Vector(0,0,-9.81));                                        //costruttore con argomenti che, a partire da oggetto KDL::Chain e un vettore di gravità KDL::Vector, costruisce un oggetto di classe KDL::ChainDynParam per costruire vettori gravità, coriolis e la matrice di inerzia B
    jacSol_ = new KDL::ChainJntToJacSolver(chain_);                                                           //Jacobian solver (vedi documentazione)
    jntJacDotSol_ = new KDL::ChainJntToJacDotSolver(chain_);                                                  //Jacobian_dot solver 
    fkSol_ = new KDL::ChainFkSolverPos_recursive(chain_);                                                     //Forward Kinematic solver (posizione)
    fkVelSol_ = new KDL::ChainFkSolverVel_recursive(chain_);                                                  //Forward Kinematic solver (velocità)
    idSolver_ = new KDL::ChainIdSolver_RNE(chain_,KDL::Vector(0,0,-9.81));
    jsim_.resize(n_);
    grav_.resize(n_);
    q_min_.data.resize(n_);
    q_max_.data.resize(n_);
    q_min_.data << -2.96,-2.09,-2.96,-2.09,-2.96, -2.09,-2.96;//
    q_max_.data <<  2.96,2.09,2.96,2.09,2.96, 2.09, 2.96;
    ikVelSol_ = new KDL::ChainIkSolverVel_wdls(chain_);
    ikSol_ = new KDL::ChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fkSol_, *ikVelSol_);                    //inverse kinematic solver
    // jntArray_out_ = KDL::JntArray(n_);
}

void KDLRobot::update(std::vector<double> _jnt_values, std::vector<double> _jnt_vel)
{
    KDL::Twist s_T_f;
    KDL::Frame s_F_f;
    KDL::Jacobian s_J_f(7);
    KDL::Jacobian s_J_dot_f(7);
    KDL::FrameVel s_Fv_f;
    KDL::JntArrayVel jntVel(jntArray_,jntVel_);
    KDL::Twist s_J_dot_q_dot_f;

    // joints space
    updateJnts(_jnt_values, _jnt_vel);                                   
    dynParam_->JntToMass(jntArray_, jsim_);                                                                   //dai joint values, definisce la mass matrix del robot
    dynParam_->JntToCoriolis(jntArray_, jntVel_, coriol_);                                                    //dai joint values e velocities, definisce la terna di coriolis del modello
    dynParam_->JntToGravity(jntArray_, grav_);                                                                //dai joint values, definisce il gravity vector del robot

    // robot flange                                                                                           //flangia = parte grigia del kuka iiwa, che collega l'eventuale end effector che sarà aggiunto e il braccio del robot) -> Otteniamo Posa/Frame e Twist/Velocità della flangia rispetto allo Spatial/Inertial/Base frame, e lo Jacobiano classico/Spatial Jacobian (che mappa le configurazioni dei giunti nel twist della flangia rispetto allo Spatial/Inertial/Base frame)
    fkVelSol_->JntToCart(jntVel, s_Fv_f);                                                                     //KDL::ChainFkSolverVel_recursive::JointToCartesian: Calcola cinematica diretta relativa alla POSA/FRAME e VELOCITA'/TWIST da un vettore di classe "KDLJntArrayVel". Tale metodo riempie s_Fv_f (Frame&Velocity della FLANGIA (e non e.e.) rispetto lo Spatial/Inertial/Base Frame), un vettore di classe "KDL::FrameVel": An FrameVel is a Frame and its first derivative, a Twist vector.
    s_T_f = s_Fv_f.GetTwist();                                                                                //s_T_f conterrà il Twist (rot + trasl) di s_Fv_f. Cioè conterrà il Twist della flangia rispetto allo Spatial/inertial/base frame.
    s_F_f = s_Fv_f.GetFrame();                                                                                //s_T_f conterrà il Frame (rot + trasl) di s_Fv_f. Cioè conterra la Posa della flangia rispetto allo Spatial/inertial/base frame
    int err = fkSol_->JntToCart(jntArray_,s_F_f);                                                             //KDL::ChainFkSolverPos_recursive::JointToCartesian: Calcola cinematica diretta relativa alla POSA/FRAME da un vettore di classe "KDLJntArray". s_F_f è riempito con la posa della flangia. "err" conterrà un valore <0 se c'è un errore
    err = jacSol_->JntToJac(jntArray_, s_J_f);                                                                //Calcola lo Jacobiano geometrico a partire dai joints. In uscita restituisce l'oggetto di classe "Jacobian" s_J_f, cioè lo Jacobiano geometrico in quella configurazione relativo alla FLANGIA rispetto allo Spatial/inertial/base frame(vedi documentazione)
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_q_dot_f);                                                //KDL::ChainJntToJacDotSolver::JntToJacDot: quello che fa dipende dal secondo parametro in input. In questo caso, essendo un elemento di tipo "KDL::Twist" (velocità di traslazione + rotazione): a partire da vettore "KDL::JntArrayVel", calcola "J(q)_dot*q_dot" (relativo alla FLANGIA e rispetto allo Spatial/inertial/base frame) e riempie s_J_dot_q_dot_f
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_f);                                                      //KDL::ChainJntToJacDotSolver::JntToJacDot: quello che fa dipende dal secondo parametro in input. In questo caso, essendo un elemento di tipo "KDL::Jacobian" (matrice Jacobiana): a partire da vettore "KDL::JntArrayVel", calcola "J(q)_dot" (relativo alla FLANGIA e rispetto allo Spatial/inertial/base frame) e riempie s_J_dot_f

    // robot end-effector                                                                                     //finora tutto e' stato fatto rispetto alla FLANGIA (e allo Spatial/inertial/base frame) -> aggiungerò un end effector -> servono delle trasformazioni per fare tutto rispetto all'e.e.)
    s_F_ee_ = s_F_f*f_F_ee_;                                                                                  //il prodotto tra 2 oggetti di classe KDL::Frame rappresenta la TRASFORMAZIONE (non solo rotazione) tra due Frame (vedi documentazione). Otteniamo quindi il frame dell'e.e. rispetto allo Spatial/inertial/base frame, unendo il frame della flangia rispetto allo Spatial/inertial/base frame e il frame dell'e.e. rispetto al Flange frame
    KDL::Vector s_p_f_ee = s_F_ee_.p - s_F_f.p;                                                               //ottengo la distanza tra flange frame ed e.e. frame (infatti tra flangia ed e.e. non può esserci una rotazione relativa, ma al massimo solo una leggera distanza relativa )
    KDL::changeRefPoint(s_J_f, s_p_f_ee, s_J_ee_);                                                            //s_J_f -> s_J_ee. a partire dallo Jacobiano rispetto alla flangia (e allo Spatial/inertial/base frame) ricavo quello rispetto all'e.e. (e allo Spatial/inertial/base frame)   
    KDL::changeRefPoint(s_J_dot_f, s_p_f_ee, s_J_dot_ee_);                                                    //stessa cosa, ma con J(q)_dot
    KDL::changeBase(s_J_ee_, s_F_ee_.M.Inverse(), b_J_ee_);                                                   //s_J_ee -> b_J_ee. Completo la trasformazione dello Jacobiano cambiandogli la base. Ottengo il Body Jacobian rispetto al'e.e. frame e al Body/e.e. Mappa le configurazioni dei giunti nel twist dell'e.e. rispetto all'E.E. FRAME (e non rispetto allo Spatial /inertial/base frame come lo Jacobiano classico/Spatial Jacobian))
    KDL::changeBase(s_J_dot_ee_, s_F_ee_.M.Inverse(), b_J_dot_ee_);                                           //stessa cosa, ma con J(q)_dot
    s_V_ee_ = s_T_f.RefPoint(s_p_f_ee);                                                                       //Ottengo il Twist dell'e.e. rispetto allo Spatial/inertial/base frame 

}


////////////////////////////////////////////////////////////////////////////////
//                                 CHAIN                                      //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::createChain(KDL::Tree &robot_tree)                                                             //creiamo/otteniamo la chain di questo robot
{
    //if(!robot_tree.getChain(robot_tree.getRootSegment()->first, "lbr_iiwa_link_7",chain_))
    if(!robot_tree.getChain(robot_tree.getRootSegment()->first,                                               //getChain() -> metodo di "KDL::Tree". La catena ottenuta riempie "chain_", oggetto di classe KDL::Chain. Ritorna un bool per successo o fallimento
        std::prev(std::prev(robot_tree.getSegments().end()))->first, chain_))
    {
        std::cout << "Failed to create KDL robot" << std::endl;
        return;
    }
    std::cout << "KDL robot model created" << std::endl;
    std::cout << "with " << chain_.getNrOfJoints() << " joints" << std::endl;                                 //metodo di KDL::Tree che ritorna il numero di joints della catena
    std::cout << "and " << chain_.getNrOfSegments() << " segments" << std::endl;                              //metodo di KDL::Tree che ritorna il numero di links/segments della catena. Sono sempre compresi il base frame (ground) e l'end effector frame (2 in più)
}

unsigned int KDLRobot::getNrJnts()                                                                            //A che serve aver creato questo metodo quando c'è già un metodo "getNrOfJoints"? Perché quest'ultimo appartiene alla classe "KDL::Chain", quindi avrei potuto usarlo solo con oggetti di questa classe. Definendo questo metodo per la classe da noi implementata "KDLRobot", possiamo usarlo direttamente sul nostro robot "robot", che è un oggetto di questa classe. E' per rendere quindi tutto più intuitivo in "kdl_robot_test.cpp"
{
    return n_;                                                                                                //usa l'"n_" riempito a riga 11 
}

unsigned int KDLRobot::getNrSgmts()
{
    return chain_.getNrOfSegments();
}

////////////////////////////////////////////////////////////////////////////////
//                                 JOINTS                                     //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::updateJnts(std::vector<double> _jnt_pos, std::vector<double> _jnt_vel)                         //metodo di classe KDLRobot (creato da noi) per aggiornare lo stato (posizione + velocità) dei giunti      
{
    for (unsigned int i = 0; i < n_; i++)
    {
        //std::cout << _jnt_pos[i] << std::endl;
        jntArray_(i) = _jnt_pos[i];
        jntVel_(i) = _jnt_vel[i];
    }
}
Eigen::VectorXd KDLRobot::getJntValues()                                                                      //prelevo rotazione/posizione dei giunti, che è un oggetto di classe Eigen::VectorXd   
{
    return jntArray_.data;                                                                                    //"jntArray" è oggetto di classe KDL::JntArray. Il suo attributo ".data", che contiene i suoi dati, è di classe Eigen::VectorXd
}

Eigen::VectorXd KDLRobot::getJntVelocities()                                                                  //prelevo velocità rotazionale dei giunti
{
    return jntVel_.data;
}

Eigen::MatrixXd KDLRobot::getJntLimits()
{
    Eigen::MatrixXd jntLim;
    jntLim.resize(n_,2);

    jntLim.col(0) = q_min_.data;
    jntLim.col(1) = q_max_.data;

    return jntLim;
}

Eigen::MatrixXd KDLRobot::getJsim()                                                                           //prelevo matrice di inerzia B (la ritorno come oggetto di classe Eigen::MAtrixXd)  
{
    return jsim_.data;
}

Eigen::VectorXd KDLRobot::getCoriolis()                                                                       //prelevo vettore di coriolis
{
    return coriol_.data;
}

Eigen::VectorXd KDLRobot::getGravity()                                                                        //prelevo vettore di gravità
{
    return grav_.data;
}

Eigen::VectorXd KDLRobot::getID(const KDL::JntArray &q,
                                const KDL::JntArray &q_dot,
                                const KDL::JntArray &q_dotdot,
                                const KDL::Wrenches &f_ext)
{
    Eigen::VectorXd t;
    t.resize(chain_.getNrOfJoints());
    KDL::JntArray torques(chain_.getNrOfJoints());
    int r = idSolver_->CartToJnt(q,q_dot,q_dotdot,f_ext,torques);
    std::cout << "idSolver result: " << idSolver_->strError(r) << std::endl;
    // std::cout << "torques: " << torques.data.transpose() << std::endl;
    t = torques.data;
    return t;
}

KDL::JntArray KDLRobot::getInvKin(const KDL::JntArray &q,                                                       //metodo della classe KDLRobot (creato da noi) che attua algoritmo di inversione cinematica e ricava la configurazione "jnt_Array_out" (classe KDL::JntArray) corrispondente alla posa "eeFrame" (classe KDL::Frame)
                        const KDL::Frame &eeFrame)
{
    KDL::JntArray jntArray_out_;
    jntArray_out_.resize(chain_.getNrOfJoints());
    int err = ikSol_->CartToJnt(q, eeFrame, jntArray_out_);
    if (err != 0)                                                                                               //durante ogni step del ciclo, se l'errore di calcolo è != 0 ... 
    {
        printf("inverse kinematics failed with error: %d \n", err);                                             //... stampa questo messaggio a a terminale.: Se è andato tutto bene te ne accorgi: 1) da Gazebo ad occhio; 2) da terminale non sarà stampato alcun messaggio a parte "time: ...". Se qualcosa è andato storto: 1)  te ne accorgi da Gazebo ad occhio; 2) a terminale, ad ogni ciclo è stampato questo messaggio
    }
    return jntArray_out_;
}
////////////////////////////////////////////////////////////////////////////////
//                              END-EFFECTOR                                  //
////////////////////////////////////////////////////////////////////////////////

KDL::Frame KDLRobot::getEEFrame()                                                                               //prelevo la TRASFORMAZIONE "spatial/base/inertial frame -> e.e. frame" (di classe KDL::Frame). La classe KDL::Frame, nonostante sembra possa definire un frame, in realtà rappresenta una TRASFORMAZIONE TRA FRAMES    
{
    return s_F_ee_;
}


KDL::Frame KDLRobot::getFlangeEE()                                                                              //prelevo la TRASFORMAZIONE "spatial/base/inertial frame -> flange frame"
{
    return f_F_ee_;
}


KDL::Twist KDLRobot::getEEVelocity()                                                                            //prelevo il Twist (velocità traslazionale + rotazionale) di un punto (nel nostro caso dell'e.e) (classe KDL::Twist)
{
    return s_V_ee_;
}

KDL::Twist KDLRobot::getEEBodyVelocity()
{
    return s_V_ee_;
}

KDL::Jacobian KDLRobot::getEEJacobian()                                                                         //prelevo lo SPATIAL Jacobian (spatial frame -> e.e. frame) geometrico (classe KDL::Jacobian)
{
    return s_J_ee_;                                                                                                                                           
}

KDL::Jacobian KDLRobot::getEEBodyJacobian()                                                                     //ritorna il BODY Jacobian, di classe KDL::Jacobian
{
    return b_J_ee_;
}

Eigen::MatrixXd KDLRobot::getEEJacDot()                                                                         //era "getEEJacDotqDot", ma l'ho rinominata perché in realtà non ritorna "J(q)dot*qdot", ma solo un Jacobiano "J(q)dot", di classe Eigen::MAtrixXd
{
    return s_J_dot_ee_.data;                                                                                    //elemento di tipo Eigen::MatrixXd  (perché c'è il ".data")
}

void KDLRobot::addEE(const KDL::Frame &_f_F_ee)                                                                 //metodo della classe KDLRobot (creato da noi) per aggiungere una trasformazione all'attuale frame considerato dell'e.e, ed AGGIORNARE quindi l'e.e. frame. Serve nel caso debba aggiungere un frame da considerare il vero e.e. per tutte le successive operazioni. Ad esempio per quando aggiungerò una camera: sarà la camera la nostra nuova appendice! 
{
    f_F_ee_ = _f_F_ee;                                                                                          //trasformazione "flange frame -> e.e. frame"
    this->update(toStdVector(this->jntArray_.data), toStdVector(this->jntVel_.data));                           //aggiungo questa trasformazione all'e.e attuale (che è in realtà il flange frame), così da avere adesso un nuovo ed effettivo e.e.
}


////////////////////////////////////////////////////////////////////////////////
//                              HOMEWORK2                                     //
////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd KDLRobot::getEEJacDotqDot()                                                                     //ritorna oggetto "J(q)dot*qdot" di classe Eigen::VectorXd
{
    return s_J_dot_ee_.data*jntVel_.data;                                                                       
}

Eigen::VectorXd KDLRobot::getEEJacDotqDotPos()
{
    return s_J_dot_ee_.data.topRows(3)*jntVel_.data;
}

/////////////////////////////////////////////////////////////////////////////////