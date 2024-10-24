#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>
#include "eigen_conversions/eigen_kdl.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "geometry_msgs/PoseStamped.h"


// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0);
bool robot_state_available = false;

std::vector<double> init_jnt_pos(7,0.0), aruco_pose(7,0.0);                                                                          //HOMEWORK3       
bool aruco_pose_available = false;                                                                                                   //HOMEWORK3

int selection=3;                                                                                                                     //HOMEWORK2   "kdl_planner.cpp" -> in base al valore di "selection", verrà inseguita una determinata traiettoria

// Functions
KDLRobot createRobot(std::string robot_string)                                                                                       //metodo per creare un robot. Ma non l'avevo già fatto con il file .urdf? il software OROCOS e la sua libreria KDL non comprendono i file .urdf, quindi è necessario effettuare il parsing (conversione di una struttura dati da un certo formato ad un altro formato, magari compresnibile e manipolabile) del robot dal formato .urdf al formato delle classi KDL::Tree o KDL::Chain
{
    KDL::Tree robot_tree;                                                                                                            //costruisco oggetto della classe KDL::Tree. Non è un costruttore senza argomenti, perché KDL::Tree non ce l'ha. Piuttosto è un costruttore che ha come argomento una stringa vuota "".
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))                                                                                            //metodo della classe urdf::Model per fare un parsing intermedio "urdf -> file"
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))                                                                        //metodo in "kdl_parser.cpp" che applica il parsing "urdf -> tree", necessario per lavorare con il nostro robot in OROCOS e con la libreria KDL
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);                                                                                                      //costruisco l'oggetto di classe KDLRobot (da noi creata) a partire dal tree richiamando il costruttore della classe KDLRobot
    return robot;                     
}

void jointStateCallback(const sensor_msgs::JointState & msg)                                                                         //callback function che viene chiamata  appena viene popolato il topic "/iiwa/joint_states" con lo stato dei giunti non fissi del robot(tempo, nome, posizione, velocità, effort di ognuno).
{
    robot_state_available = true;                                                                                                    //(riga 17): false -> true. Capirai l'uso alla riga 111
    jnt_pos.clear();                                                                                                                 //std::clear() : metodo per svuotare i vettori prima di ri-riempirne gli elementi
    jnt_vel.clear();                                                                           
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);                                                                                          //std::push_back() : metodo per aggiornare i valori [i] nell'array. Riempie il primo elemento vuoto che trova in jnt_pos
        jnt_vel.push_back(msg.velocity[i]);                                         
    }
}

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)                                                                       //HOMEWORK3
{                                                                                                                                    //HOMEWORK3
    aruco_pose_available = true;                                                                                                     //HOMEWORK3
    aruco_pose.clear();                                                                                                              //HOMEWORK3
    aruco_pose.push_back(msg.pose.position.x);                                                                                       //HOMEWORK3                                                                                   
    aruco_pose.push_back(msg.pose.position.y);                                                                                       //HOMEWORK3
    aruco_pose.push_back(msg.pose.position.z);                                                                                       //HOMEWORK3
    aruco_pose.push_back(msg.pose.orientation.x);                                                                                    //HOMEWORK3
    aruco_pose.push_back(msg.pose.orientation.y);                                                                                    //HOMEWORK3
    aruco_pose.push_back(msg.pose.orientation.z);                                                                                    //HOMEWORK3
    aruco_pose.push_back(msg.pose.orientation.w);                                                                                    //HOMEWORK3
}

// Main
int main(int argc, char **argv)
{
    if (argc < 2) 
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");                                                                                   //inizializzo nodo
    ros::NodeHandle n;                                                                                                               //oggetto di classe ros::NodeHandle per gestire il nodo e usare tutte le funzionalità previste su di esso                                   

    // Rate
    ros::Rate loop_rate(500);                                                                                                        //oggetto di classe ros::Rate per gestire la frequenza desiderata per il nodo

    // Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);                                      //oggetto di classe ros::Subscriber per iscriversi al topic all'interno del quale vanno gli stati dei joints. Ogni volta che arriva un messaggio a questo topic, parte la callback function. Chi è il publisher di questo topic, cioè quello che ci informa dello stato? GAZEBO. Quando inizia a mandare questi messaggi su questo topic? Appena premo "play", rendendo disponibile lo stato dei giunti del robot
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);                                        //HOMEWORK3. mi sottoscrivo al topic che riceve dalla camera la stima della posa dell'aruco marker inquadrato. AATENZIONE: la posa dell'aruco marker, essendo stimata dalla camera, è rispetto al CAMERA FRAME. Più avanti quindi dovremo sicuramente riportare tutto rispetto allo Spatial/inertial/base frame


    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_1_effort_controller/command", 1);            //oggetto di classe ros::Publisher per scrivere messaggi al topic dal quale il controller spawner legge quali comandi di forza (tau) mandare ai giunti. Con cosa riempiamo questi messaggi? Con i "tau" che ci vengono inviati dal controller in "kdl_control.cpp"        
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_7_effort_controller/command", 1);    

    
    ros::Publisher err_pos_norm_pub = n.advertise<std_msgs::Float64>("/iiwa/error_norm/pos", 1);                                     //HOMEWORK3   
    ros::Publisher err_or_norm_pub = n.advertise<std_msgs::Float64>("/iiwa/error_norm/or", 1);                                       //HOMEWORK3




    // Services: richiedi un servizio, e aspetti di essere servito
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration"); //oggetto di classe ros::ServiceClient (cioè un client di servizio) per inviare una richiesta al servizio "/gazebo/set_model_configuration" (servizio che consente di modificare la configurazione di un modello nel simulatore Gazebo) di tipo "SetModelConfiguration" (per imoostare la configurazione INIZIALE)
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");                                      //oggetto di classe ros::ServiceClient (cioè un client di servizio) per inviare una richiesta al servizio "/gazebo/pause_physics" (servizio che mette in pausa la simulazione fisica in Gazebo. Posso riprende la simulazione facendo clic su "Play" nella Gazebo GUI in basso a sx) di tipo "std_srvs::Empty" (vuoto). IMPORTANTE: GAZEBO PARTE GIA' IN MODALITA' PAUSE grazie a "iiwa_gazebo_effort.launch" che carica il world "iiwa_world.world" (arg="paused" value="true")

    // Set robot state:                                                                                                              
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);//rads
    robot_init_config.request.joint_positions.push_back(1.57); //pi/2
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.2);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-0.37);
    if(robot_set_state_srv.call(robot_init_config))                                                                                  //Effettuo la chiamata al servizio. Se c'è la risposta alla chiamata...
        ROS_INFO("Robot state set.");
    else                                                                                                                             //altrimenti...
        ROS_INFO("Failed to set robot state.");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;                                          //dichiarazione dei messaggi che saranno poi mandati al topic su cui pubblicano gli effort_controller (vedi riga 69-75). Sono coppie/torques ai giunti "tau" (controllore in coppia)
    std_msgs::Float64 err_pos_norm_msg, err_or_norm_msg;                                                                             //HOMEWORK3
    std_srvs::Empty pauseSrv;                                                                                                        //oggetto che mi serve per effettuare la chiamata al servizio di pausa in Gazebo, MA NON ALL'INIZIO, MA ALLA FINE A RIGA 316!!!! LA PAUSA ALL'INIZIO E' OPERA DI "iiwa_gazebo_effort.launch" CHE LANCIA IL WORLD "iiwa_world.world" IN MODALITA' PAUSE (arg="paused" value="true")

    // Wait for robot and object state:                                                                                              //IMPORTANTE. Carico la simulazione del robot (file ".launch") + Eseguo questo nodo di controllo ($ rosrun .... ): appena eseguo, la simulazione viene messa in "pause mode" in una configurazione iniziale prestabilita (visto prima). Appena premerò "play", parte la callback function -> viene settata true la variabile a riga 17 -> Gazebo partirà -> Gazebo fornirà lo stato del robot al topic -> in base allo stato che viene letto dal nodo subscriber e alla posa desiderata, viene elaborata la legge di controllo -> mando al topic riguardante i comandi "tau", che fungono da attuatori -> controllo robot nella posa desiderata. Problema: gli attuatori inizieranno a funzionare solo nel momento in cui premo "play". Finché non premo "play" come faccio a mantenere il robot nella configurazione iniziale prestabilita, dal momento che solo degli attuatori potrebbero manternerlo in quella posa? Lo faccio bloccandolo nella configurazione iniziale settata intrappolandoci prima di un un CICLO INFINITO (ciclo while), che viene abbandonato solo se viene settata true la variabile a riga 17. Ma a che mi serve fare tutto ciò? Perché altrimenti il robot cadrebbe, e risulterebbe instabile da controllare.
    while (!(robot_state_available))                                                                                                 //finche' lo stato del robot non è disponibile, mi blocco in questo loop dove
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        if (!(robot_set_state_srv.call(robot_init_config)))     
            ROS_INFO("Failed to set robot state.");            
        
        ros::spinOnce();
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    robot.update(jnt_pos, jnt_vel);
    int nrJnts = robot.getNrJnts();

    // Specify an end-effector 
    robot.addEE(KDL::Frame::Identity());                                                                                             //aggiungendo come trasformazione una matrice identità, vuol dire che flange frame = end effector frame. Se non avessi fatto nulla, sarebbe stato la stessa cosa, ma ai fini didattici andava fatta vedere 

    // Specify an end-effector: camera in flange transform                                                                           //HOMEWORK3
    KDL::Frame ee_T_cam;                                                                                                             //HOMEWORK3 trasformazione "camera frame - end-effector(che coincide con la flange in questo caso)-frame". Ci serve perché dovremo controllare posizione + orientamento (posa) della CAMERA, non della flange/e.e. Per quanto riguarda il controllo di posizione, non è un gran problema, perché il frame della camera e quello della flange/e.e. si muovono con la stessa velocità (sono fissati tra loro). Il problema è la rotazione: quando ruoti attorno al cmmera frame, la camera si traslerà + ruoterà in giro, ma se ruoti l'oggtto solo attorno al punto centrale della camera, essa non traslerà, ma ruoterà solamente. Tutto ciò è fatto internamente alla libreria KDLRobot
    ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57);                                                               //HOMEWORK3 rotazione relativa camera - end effector. Camera frame ruotato di un quarto di giro (90°) prima attorno all'asse y, e poi attorno all'asse z (terna dinamica, postmoltiplicazioni) -> asse x alla destra della camera, asse y sotto la camera, asse z in direzione della camera
    ee_T_cam.p = KDL::Vector(0,0,0.025);                                                                                             //HOMEWORK3 traslazione relativa camera - end-effector
    robot.addEE(ee_T_cam);                                                                                                           //HOMEWORK3 aggiorno end effector. Da ora, se mai attuerò metodi come "getEEJacobian()", mi verrà fornito lo Jacobiano geometrico relativo alla CAMERA rispetto allo Spatial/inertial/base frame

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());                                              //definisco desired_joint_values (qd), desired_joint_velocities (dot_q_desired) e desired_joint_accelerations (double_dot_q_desired) (costruttori con argomento)
    dqd.data.setZero();                                                                                                              //Per ora inizializzo solo a zero le velocità ...
    ddqd.data.setZero();                                                                                                             //...e le accelerazioni. Le posizioni angolari vengono inizializzate alla riga 219

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);                                                                                                  //metodo della classe KDLRobot (crato da noi) in "kdl_robot.h" e ."cpp"                                                                                        

    // Init controller
    KDLController controller_(robot);                                                                                                //KDLController e' una classe definita in "kdl_control.h" e "kdl_control.cpp". Questo è il suo costruttore con argomenti

    // EE's trajectory initial position
    KDL::Frame init_cart_pose = robot.getEEFrame();                                                                                  //initial_cartesian_pose, di classe "KDL::Frame". La posizione iniziale della traiettoria è la posizione attuale del robot (configurazione iniziale di cui sopra)
    Eigen::Vector3d init_position(init_cart_pose.p.data);                                                                            //Obiettivo: modificare la posa dell'end effector del robot modificando di essa SOLO LA POSIZIONE, e non l'orientamento. Quindi, del frame "init_cart_pose", che individuerà la posa iniziale del robot, prelevo solo la parte della posizione, cioè il vettore. Dichiaro quindi un oggetto di classe "Vector3d" con i dati provenienti dalla parte posizionale del frame attuale dell'e.e, cioè "init_cart_pose.p.data"

    // EE trajectory end position
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();                                               //il punto finale della traiettoria sarà un punto con stessa componente x, stessa z, ma y opposta

    // Plan trajectory
    double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, init_time_slot = 1.0, radius=0.2;                                       //definisco variabili che caratterizzeranno la traiettoria. //HOMEWORK2 radius deve essere non superiore a 0.2 m (try and error)
    KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);                                                    //currently using trapezoidal velocity profile. KDLPlanner: classe definita in "kdl_planner.h" e "kdl_planner.cpp". Questo è il suo costruttore con argomenti
    //KDLPlanner planner(traj_duration, init_position, radius);                                                                      //HOMEWORK2

    // Retrieve the first trajectory point
    //trajectory_point p = planner.compute_trajectory(t);                                                                            //"trajectory_point": attributo della classe "KDLPlanner" per definire la parte traslazionale di una traiettoria, formato da 3 campi: sono i vettori posizione, velocità e accelerazione. "compute trajectory": metodo della classe "KDLPlanner" che computa la traiettoria con profilo trapezoidale di velocità, e assegnando i valori a "trajectory_point". Trovi tutto in "kdl_planner.h" e "kdl_planner.cpp"
    trajectory_point p = planner.compute_trajectory(t,selection);                                                                    //HOMEWORK2

    // Gains
    //double Kp = 50, Kd = sqrt(Kp);  //idCntr in joint space     (riga 224)
    double Kp = 50, Ko = 50;      //idCntr in operative space (riga 228)                                                         //HOMEWORK2
                                                                                                                       
    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity(); KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
    des_pose.M = robot.getEEFrame().M;                                                                                               //l'orientazione desiderata è quella iniziale. La posizione desiderata invece dipenderà dalla traiettoria scelta

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)                                                      //Control Loop. Voglio che la simulazione duri almeno 2 volte la durata della traiettoria + offset temporale che ho stabilito tra il momento in cui premo "play" e l'effettivo inizio della traiettoria
    {
        if (robot_state_available)                                                                                                   //Se ho premuto "Play", ad ogni ciclo...
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);                                                                                          

            // Update time
            t = (ros::Time::now()-begin).toSec();
            std::cout << "time: " << t << std::endl;

            // Extract desired pose
            des_cart_vel = KDL::Twist::Zero();
            des_cart_acc = KDL::Twist::Zero();
            if (t <= init_time_slot) // wait a second.                                                                               //aspetto l'offset temporale prima di far partire il tracking della traiettoria
            {
                //p = planner.compute_trajectory(0.0);                                                                               //la traiettoria rimane al punto iniziale 
                p = planner.compute_trajectory(0.0,selection);                                                                       //HOMEWORK2 
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)                                                       //ad ogni ciclo elaboro il successivo punto della traiettoria
            {
                //p = planner.compute_trajectory(t-init_time_slot);
                p = planner.compute_trajectory(t-init_time_slot,selection);                                                          //HOMEWORK2            
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());                            //KDL::Twist: classe di oggetti che contiene il Twist: velocità lineare (p) + angolare (M). A noi interessa in questo caso solo la parte lineare (vogliamo realizzare solo una traiettoria che modifichi la parte lineare), quindi setto a zero quella angolare 
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());              
            }
            else                                                                                                                     //se supero il tempo previsto per la durata della traiettoria... 
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }

            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);                                                                    //la posizione desiderata (p=parte traslazionale della posa) sarà quella definita dalla traiettoria "p" ad ogni ciclo. L'orientazione desiderata abbiamo già detto che è quella iniziale
            
            // compute current jacobians
            KDL::Jacobian J_cam = robot.getEEJacobian();                                                                             //HOMEWORK3 Alla riga 180 abbiamo detto che tutte le funzioni che prima restituivano caratteristiche relative all'end effector, ora sono in realtà relative alla camera. Anche lo Jacobiano geometrico classico/spatial. Cosa comporta usare lo Spatial Jacobian? Che, dal momento che gli errori di orientazione della camera saranno computati rispetto al CAMERA frame (vedrai tra poco), bisognerà riportarli rispetto allo SPATIAL frame 
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2])); //HOMEWORK3 transformation "object frame - camera frame". Ottengo la posa dell'oggetto (aruco marker) con l'updated aruco pose quantities che ricevo attraverso la callback function "arucoPoseCallback". Restituisce la posa (posizione + orientamento) dell'aruco marker rispetto al CAMERA FRAME (infatti è la camera che inquadra l'aruco marker)
                       
            // look at point: compute rotation error from angle/axis                                                                 //HOMEWORK3
            Eigen::Matrix<double,3,1> aruco_pos_n = toEigen(cam_T_object.p);//(aruco_pose[0],aruco_pose[1],aruco_pose[2]);           //HOMEWORK3 cPo. ottengo il VETTORE posizione (Matrix<double,3,1>) dell'aruco marker rispetto al camera frame: è il vettore che collega l'origine dell'aruco marker frame al camera frame. Lo converto a un elemento di classe Eigen::Matrix perché solo così posso usarlo nelle elaborazioni matematiche 
            aruco_pos_n.normalize();                                                                                                 //HOMEWORK3 versore s. Il versore che vogliamo eguagliare durante la rotazione. Ottenuto per normalizzazione di cPo == aruco_pos_n. ATTENZIONE: questa cosa normalizza il VETTORE ORIGINALE aruco_pos_n, che quindi ora è == s
            Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;                                                          //HOMEWORK3 vettore r_o = cross product/prodotto vettoriale tra versore zc (asse z della camera) e  versore s. Nell'esempio "r_o" = "a" è un versore ortogonale al piano identificato da zc ed s.
            double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));                                                 //HOMEWORK3 angolo tra i versori zc e s, computato dall'arcoseno del dot product/prodotto scalare tra i due.
            KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);                                 //HOMEWORK3 matrice di rotazione "RotationError" (usando la rappresentazione "Asse Angolo":: r_o = asse, aruco_angle = angolo) tra la rotazione attuale della camera attorno all'asse r_o e la rotazione desiderata attorno allo stesso asse per allinearsi con s. E' quindi un errore rispetto al CAMERA frame (se vuoi la desired matrix rispetto al base frame, devi moltiplicare la desired matrix per la trasformazione tra camera frame e base frame fornita da "getEEFrame()"           
            des_pose.M = robot.getEEFrame().M*Re;                                                                                    //HOMEWORK3           
            
            // std::cout << "jacobian: " << std::endl << robot.getEEJacobian().data << std::endl;
            // std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            // std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            // std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            // std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            // std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            // std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            // std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            // std::cout << "current_pose: " << std::endl << robot.getEEFrame() << std::endl;

            // inverse kinematics
            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];                           //qd = q desired. viene riempita con le joint positions CORRENTI (quelle della callback)
            //qd = robot.getInvKin(qd, des_pose);                                                                                      //a partire da joint position corrente (mi serve per lo jacobiano) + posa desiderata, usiamo un algoritmo di cinematica inversa basato su Newton-Raphson ("kdl_robot.h", "kdl_robot.cpp") per calcolare la configurazione desiderata (in realtà quella subito successiva) per continuare a seguire la traiettoria. Calcoliamo i joint perche' le torques verranno applicate direttamente su di loro
            qd = robot.getInvKin(qd, des_pose*ee_T_cam.Inverse());                                                                   //HOMEWORK3 al momento il tutto è valutato rispetto a spatial/base/inertial frame e CAMERA frame. Dal momento che tale funzione da noi implementata contiene il metodo KDL::ChainIkSolverPos_NR_JL::CartToJnt(), che attua la cinematica inversa a partire però dall'END EFFECTOR frame. Quindi dobbiamo "tornare indietro" tramite la trasformazione "camera frame -> e.e. frame"             

            // joint space inverse dynamics control
            //tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);                                                                       //sintetizziamo le torques grazie al mio InverseDynamicsController nello spazio dei giunti ("Robotics" di Siciliano -> cap 8 -> 8.5.2). NOTARE: "dqd" e "ddqd" non vengono mai aggiornati: sono sempre 0
            

            // cartesian space inverse dynamics control
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc,Kp, Ko, 2*sqrt(Kp), 2*sqrt(Ko));                           //HOMEWORK2
            //tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc,Kp, Ko);                                                 //HOMEWORK2 (solo posizionale, non usarlo)


            // cartesian error norm along the trajectory
            Eigen::Matrix<double,3,1> err_pos = computeLinearError(toEigen(des_pose.p), toEigen(robot.getEEFrame().p));              //HOMEWORK3
            Eigen::Matrix<double,3,1> err_or = computeOrientationError(toEigen(des_pose.M), toEigen(robot.getEEFrame().M));          //HOMEWORK3
            double err_pos_norm = err_pos.norm();                                                                                    //HOMEWORK3
            double err_or_norm = err_or.norm();                                                                                      //HOMEWORK3
            


            // Set torques
            tau1_msg.data = tau[0];                                                                                                  //le torques che elaboro con il controller saranno quelle che, riempiranno i messaggi
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);                                                                                     //i messaggi andranno all'interno del topic dal quale il controller spawner legge. Il controller spawner, con i valori letti, agisce poi sul robot.
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);

            // Set errors norms                                                                                                      //HOMEWORK3  
            err_pos_norm_msg.data = err_pos_norm;                                                                                    //HOMEWORK3
            err_or_norm_msg.data = err_or_norm;                                                                                      //HOMEWORK3

            // Publish                                                                                                               //HOMEWORK3
            err_pos_norm_pub.publish(err_pos_norm_msg);                                                                              //HOMEWORK3
            err_or_norm_pub.publish(err_or_norm_msg);                                                                                //HOMEWORK3


            ros::spinOnce();
            loop_rate.sleep();
        }
    }                                                                                                                                //una volta finito il tempo che avevo prefissato...
    if(pauseGazebo.call(pauseSrv))                                                                                                   //...metto in pausa facendo la chiamata al servizio                               
        ROS_INFO("Simulation paused.");                                                                
    else
        ROS_INFO("Failed to pause simulation.");

    return 0;
}