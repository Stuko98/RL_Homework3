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
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"


// Global variables
std::vector<double> jnt_pos(7,0.0), init_jnt_pos(7,0.0), jnt_vel(7,0.0), aruco_pose(7,0.0);
bool robot_state_available = false, aruco_pose_available = false;
double lambda = 10*0.2;
double KP = 15;

// Functions
KDLRobot createRobot(std::string robot_string)
{
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    // Update joints
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);                                                                                      //l'elemento specificato riempie l'ultimo elemento vuoto dell'array
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
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
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);                                        //mi sottoscrivo al topic che riceve dalla camera la stima della posa dell'aruco marker inquadrato. AATENZIONE: la posa dell'aruco marker, essendo stimata dalla camera, è rispetto al CAMERA FRAME. Più avanti quindi dovremo sicuramente riportare tutto rispetto allo Spatial/inertial/base frame
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);                                      //mi sottoscrivo al topic che riceve da Gazebo gli stati dei joint del robot
    

    // Publishers
    ros::Publisher joint1_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J1_controller/command", 1);
    ros::Publisher joint2_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J2_controller/command", 1);
    ros::Publisher joint3_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J3_controller/command", 1);
    ros::Publisher joint4_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J4_controller/command", 1);
    ros::Publisher joint5_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J5_controller/command", 1);
    ros::Publisher joint6_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J6_controller/command", 1);
    ros::Publisher joint7_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J7_controller/command", 1);

////////////////////////////////////////////////////////////////////////
//                               HOMEWORK3                           //
////////////////////////////////////////////////////////////////////////
    ros::Publisher s_x_pub = n.advertise<std_msgs::Float64>("s/x_component",1);                                                      //creo i topic              
    ros::Publisher s_y_pub = n.advertise<std_msgs::Float64>("s/y_component",1);                                    
    ros::Publisher s_z_pub = n.advertise<std_msgs::Float64>("s/z_component",1);                                    
///////////////////////////////////////////////////////////////////////

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Initial desired robot state
    init_jnt_pos[0] = 0.0;
    init_jnt_pos[1] = 1.57;
    init_jnt_pos[2] = -1.57;
    init_jnt_pos[3] = -1.57;
    init_jnt_pos[4] = 1.57;
    init_jnt_pos[5] = -1.57;
    init_jnt_pos[6] = +1.57;
    Eigen::VectorXd qdi = toEigen(init_jnt_pos);

    // Create robot
    KDLRobot robot = createRobot(argv[1]);

    // Messages
    std_msgs::Float64 dq1_msg, dq2_msg, dq3_msg, dq4_msg, dq5_msg, dq6_msg, dq7_msg;
    std_msgs::Float64 s_x_msg, s_y_msg, s_z_msg;                                                                                     //HOMEWORK3

    std_srvs::Empty pauseSrv;

    // Joints
    KDL::JntArray qd(robot.getNrJnts()), dqd(robot.getNrJnts()), ddqd(robot.getNrJnts());
    qd.data.setZero();
    dqd.data.setZero();
    ddqd.data.setZero();

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // Bring robot in a desired initial configuration with velocity control
    ROS_INFO("Robot going into initial configuration....");                                                                          //Appena premi "Play" nella Gazebo GUI, il robot viene portato alla configurazione iniziale desiderata e viene controllato in questa posizione grazie a un controller proporzionale (P)
    double jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos),toEigen(jnt_pos));                                  //Loop che mantiene il robot nella configurazione desiderata: basato su un controller P basato sulla differenza tra initial joint position che vuoi raggiungere, e quella corrente. Ciò crea la velocità che porta joint position ->initial joint position.
    while (jnt_position_error_norm > 0.01)                                                                                           //Le velocities vengono mandate al controller finché  position error > 0.01 m                                      
    {
        dqd.data = KP*(toEigen(init_jnt_pos) - toEigen(jnt_pos));                                                                    //proportional (P) controller

        // Set joints
        dq1_msg.data = dqd.data[0];
        dq2_msg.data = dqd.data[1];
        dq3_msg.data = dqd.data[2];
        dq4_msg.data = dqd.data[3];
        dq5_msg.data = dqd.data[4];
        dq6_msg.data = dqd.data[5];
        dq7_msg.data = dqd.data[6];

        // Publish
        joint1_dq_pub.publish(dq1_msg);                                                                  
        joint2_dq_pub.publish(dq2_msg);
        joint3_dq_pub.publish(dq3_msg);
        joint4_dq_pub.publish(dq4_msg);
        joint5_dq_pub.publish(dq5_msg);
        joint6_dq_pub.publish(dq6_msg);
        joint7_dq_pub.publish(dq7_msg);

        jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos),toEigen(jnt_pos));
        std::cout << "jnt_position_error_norm: " << jnt_position_error_norm << "\n" << std::endl;               
        ros::spinOnce();
        loop_rate.sleep();

    }

    // Specify an end-effector: camera in flange transform
    KDL::Frame ee_T_cam;                                                                                                             //trasformazione "camera frame - end-effector(che coincide con la flange in questo caso)-frame". Ci serve perché dovremo controllare posizione + orientamento (posa) della CAMERA, non della flange/e.e. Per quanto riguarda il controllo di posizione, non è un gran problema, perché il frame della camera e quello della flange/e.e. si muovono con la stessa velocità (sono fissati tra loro). Il problema è la rotazione: quando ruoti attorno al cmmera frame, la camera si traslerà + ruoterà in giro, ma se ruoti l'oggtto solo attorno al punto centrale della camera, essa non traslerà, ma ruoterà solamente. Tutto ciò è fatto internamente alla libreria KDLRobot
    ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57);                                                               //rotazione relativa camera - end effector. Camera frame ruotato di un quarto di giro (90°) prima attorno all'asse y, e poi attorno all'asse z (terna dinamica, postmoltiplicazioni) -> asse x alla destra della camera, asse y sotto la camera, asse z in direzione della camera
    ee_T_cam.p = KDL::Vector(0,0,0.025);                                                                                             //traslazione relativa camera - end-effector
    robot.addEE(ee_T_cam);                                                                                                           //aggiorno end effector. Da ora, se mai attuerò metodi come "getEEJacobian()", mi verrà fornito lo Jacobiano geometrico relativo alla CAMERA rispetto allo Spatial/inertial/base frame

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Retrieve initial ee pose
    KDL::Frame Fi = robot.getEEFrame();     //FrameInitial
    Eigen::Vector3d pdi = toEigen(Fi.p);    //PositionDesiredInitial

    while (true)
    {
        if (robot_state_available && aruco_pose_available)
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);
        
            // Update time
            double t = (ros::Time::now()-begin).toSec();
            std::cout << "time: " << t << std::endl;

            // compute current jacobians
            KDL::Jacobian J_cam = robot.getEEJacobian();                                                                             //Alla riga 180 abbiamo detto che tutte le funzioni che prima restituivano caratteristiche relative all'end effector, ora sono in realtà relative alla camera. Anche lo Jacobiano geometrico classico/spatial. Cosa comporta usare lo Spatial Jacobian? Che, dal momento che gli errori di orientazione della camera saranno computati rispetto al CAMERA frame (vedrai tra poco), bisognerà riportarli rispetto allo SPATIAL frame 
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2])); //transformation "object frame - camera frame". Ottengo la posa dell'oggetto (aruco marker) con l'updated aruco pose quantities che ricevo attraverso la callback function "arucoPoseCallback". Restituisce la posa (posizione + orientamento) dell'aruco marker rispetto al CAMERA FRAME (infatti è la camera che inquadra l'aruco marker)
                       
            // look at point: compute rotation error from angle/axis
            Eigen::Matrix<double,3,1> aruco_pos_n = toEigen(cam_T_object.p);//(aruco_pose[0],aruco_pose[1],aruco_pose[2]);           //cPo. ottengo il VETTORE posizione (Matrix<double,3,1>) dell'aruco marker rispetto al camera frame: è il vettore che collega l'origine dell'aruco marker frame al camera frame. Lo converto a un elemento di classe Eigen::Matrix perché solo così posso usarlo nelle elaborazioni matematiche 
            aruco_pos_n.normalize();                                                                                                 //versore s. Il versore che vogliamo eguagliare durante la rotazione. Ottenuto per normalizzazione di cPo == aruco_pos_n. ATTENZIONE: questa cosa normalizza il VETTORE ORIGINALE aruco_pos_n, che quindi ora è == s
            Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;                                                          //vettore r_o = cross product/prodotto vettoriale tra versore zc (asse z della camera) e  versore s. Nell'esempio "r_o" = "a" è un versore ortogonale al piano identificato da zc ed s.
            double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));                                                 //angolo tra i versori zc e s, computato dall'arcoseno del dot product/prodotto scalare tra i due.
            KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);                                 //matrice di rotazione "RotationError" (usando la rappresentazione "Asse Angolo":: r_o = asse, aruco_angle = angolo) tra la rotazione attuale della camera attorno all'asse r_o e la rotazione desiderata attorno allo stesso asse per allinearsi con s. E' quindi un errore rispetto al CAMERA frame (se vuoi la desired matrix rispetto al base frame, devi moltiplicare la desired matrix per la trasformazione tra camera frame e base frame fornita da "getEEFrame()"


////////////////////////////////////////////////////////////////////////
//                               HOMEWORK3                           //
////////////////////////////////////////////////////////////////////////

//          KDL::Frame offset(KDL::Rotation::Identity(), KDL::Vector(0.2, 0, 0));
//          Eigen::Matrix<double,3,1> aruco_pos_n_with_offset = toEigen(cam_T_object.p + offset.p);
//          aruco_pos_n_with_offset.normalize();                                                                                                   
//          r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n_with_offset;                                                           
//          aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n_with_offset));                                 
//          Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle); 

////////////////////////////////////////////////////////////////////////


            // compute errors
            Eigen::Matrix<double,3,1> e_o = computeOrientationError(toEigen(robot.getEEFrame().M*Re), toEigen(robot.getEEFrame().M));  //errore di orientazione 1: tra matrice di rotazione totale "camera + camera->aruco" e matrice di rotazione "camera". Il tutto è computato rispetto al SPATIAL/INERTIAL/BASE frame (trasformazione che mi porta nel base frame è possibile grazie al prodotto con "getEEFrame()"). Se lavorassi con un BODY JACOBIAN, non avrei la necessità di attuare questa trasformazione al base frame
            Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot.getEEFrame().M));                   //errore di orientazione 2: tra la rotazione  iniziale della camera e quella attuale. Quindi vuoi che la camera insegua l'aruco marker senza ruotare rispetto alla configurazione iniziale, ma scoprirai tra un paio di righe SOLO RIMANENDO FERMO RELATIVAMENTE A ROTAZIONI ATTORNO ALL'ASSE x
            Eigen::Matrix<double,3,1> e_p = computeLinearError(pdi,toEigen(robot.getEEFrame().p));                                     //errore di posizione tra la posizione iniziale della camera e quella attuale. Quindi vuoi che la camera insegua l'aruco marker ruotando, ma non spostandosi
            Eigen::Matrix<double,6,1> x_tilde; x_tilde << e_p,  e_o_w[0], e_o[1], e_o[2];                                              //errore totale. Viene considerato e_o_w solo relativamente a rotazioni attorno all'asse x (ricordo che ora l'asse x è quello che attraversa la camera lateralmente, e quindi definisce il movimento "si con la testa" della camera), ed e_o solo relativamente a rotazioni attorno agli assi y e z. In questo modo, se ad esempio spostassi verso l'alto/basso l'aruco marker, non ruoterebbe solo la camera, MA TUTTO IL ROBOT

            // resolved velocity control law  (ti ricordo che stai controllando velocità ai giunti (del tipo qdot), non coppie)        //"Robotics" di Siciliano -> cap 10 -> 10.7 -> 10.7.2
            Eigen::MatrixXd J_pinv = J_cam.data.completeOrthogonalDecomposition().pseudoInverse();                                     //metodo di Eigen::completeOrthogonalDecomposition per ricavare la pseudo-inversa del Jacobiano Geometrico del Camera frame rispetto allo Spatial/inertial/base frame
            dqd.data = lambda*J_pinv*x_tilde + 10*(Eigen::Matrix<double,7,7>::Identity() - J_pinv*J_cam.data)*(qdi - toEigen(jnt_pos));//"Robotics" di Siciliano -> cap 10 -> 10.7.2 -> (10.76)    //lambda= gain che computa le velocita' nel joint space.  ---  "10*(Eigen::Matrix<double,7,7>::Identity() - J_pinv*J_cam.data)": modo classico per lavorare nel null space.  ---   "(qdi - toEigen(jnt_pos)": usato per mantenere l'elbow e i joints del robot più vicino possibile configurazione iniziale dei joints 



////////////////////////////////////////////////////////////////////////
//                               HOMEWORK3                           //
////////////////////////////////////////////////////////////////////////    

            Eigen::Matrix3d I3 =  Eigen::Matrix3d::Identity();
            Eigen::MatrixXd L  = Eigen::Matrix<double,3,6>::Zero();
            Eigen::Matrix<double,3,1> aruco_pos_n2 = toEigen(cam_T_object.p);                                                          //mi serve definire un nuovo "aruco_pos_n == cPo", perché quello di prima è stato già normalizzato 
            L.block(0,0,3,3) = (-1/aruco_pos_n2.norm())*(I3 - aruco_pos_n*aruco_pos_n.transpose());
            L.block(0,3,3,3) = skew(aruco_pos_n);                                                                                      //aruco_pos_n è il vettore GIA' NORMALIZZATO "s" (vedi più su)
            Eigen::Matrix3d Rc = toEigen(robot.getEEFrame().M);
            Eigen::MatrixXd R = Eigen::Matrix<double,6,6>::Zero();
            R.block(0,0,3,3) = Rc;
            R.block(3,3,3,3) = Rc;
            L=L*R.transpose();                                                                                                         //se la formula mi dice "R", perché devo moltiplicare per "R.tranpose()"? Perché c'è scritto "mappa velocità lineari/angolari della CAMERA", quindi è nel reference frame della camera. Ma noi vogliamo comandare velocità nel base/spatial/inertial frame. Quindi la formula è sbagliata: è "R^T"!

            //resolved veocity control law
            Eigen::Vector3d sd(0, 0, 1);
            Eigen::MatrixXd I7 =  Eigen::MatrixXd::Identity(7,7);
            Eigen::MatrixXd LJ = L*robot.getEEJacobian().data;
            Eigen::MatrixXd LJpinv = LJ.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd N = (I7 - (LJpinv*LJ));
            double k = 10;

            dqd.data = k*LJpinv*sd + N*(qdi - toEigen(jnt_pos));

            
            //Set s components
            s_x_msg.data = aruco_pos_n[0];
            s_x_msg.data = aruco_pos_n[1];
            s_x_msg.data = aruco_pos_n[2];

            //Publish
            s_x_pub.publish(s_x_msg);
            s_y_pub.publish(s_y_msg);
            s_z_pub.publish(s_z_msg);

//////////////////////////////////////////////////////////////////////////////      


            // debug
            // std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
            // std::cout << "Rd: " << std::endl << toEigen(robot.getEEFrame().M*Re) << std::endl;
            // std::cout << "aruco_pos_n: " << std::endl << aruco_pos_n << std::endl;
            // std::cout << "aruco_pos_n.norm(): " << std::endl << aruco_pos_n.norm() << std::endl;
            // std::cout << "Re: " << std::endl << Re << std::endl;
            // std::cout << "jacobian: " << std::endl << robot.getEEJacobian().data << std::endl;
            // std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            // std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            // std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            // std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            // std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            // std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            // std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            // std::cout << "current_pose: " << std::endl << robot.getEEFrame() << std::endl;
        }
        else{
            dqd.data = KP*(toEigen(init_jnt_pos) - toEigen(jnt_pos));                                                                  //se gli stati del robot o dell'aruco marker non sono disponibili, rimani nella configurazione iniziale
        }
        // Set joints
        dq1_msg.data = dqd.data[0];
        dq2_msg.data = dqd.data[1];
        dq3_msg.data = dqd.data[2];
        dq4_msg.data = dqd.data[3];
        dq5_msg.data = dqd.data[4];
        dq6_msg.data = dqd.data[5];
        dq7_msg.data = dqd.data[6];

        // Publish
        joint1_dq_pub.publish(dq1_msg);
        joint2_dq_pub.publish(dq2_msg);
        joint3_dq_pub.publish(dq3_msg);
        joint4_dq_pub.publish(dq4_msg);
        joint5_dq_pub.publish(dq5_msg);
        joint6_dq_pub.publish(dq6_msg);
        joint7_dq_pub.publish(dq7_msg);





        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}