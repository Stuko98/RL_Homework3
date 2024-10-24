#include "kdl_ros_control/kdl_robot.h"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

// Functions
KDLRobot createRobot(std::string robot_string) //definisco una funzione che crea un robot. Accetta in ingresso una stringa, che sarà il path all'urdf del robot (vedi riga 34)
{
    //conversione urdf -> urdf model -> kdl tree
    KDL::Tree robot_tree; //dichiaro un kdl tree
    urdf::Model my_model; //dichiaro un urdf model
    if (!my_model.initFile(robot_string)) //urdf -> urdf model
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree)) //urdf model -> kdl tree ad opera del parser
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree); //KDLRobot e' una classe dichiarata nel file "kdl_robot.h". Il suo costruttore con argomenti e' definito in "kdl_robot.cpp"
    return robot;
}

// Main: Creeremo oggetto "robot" di classe "KDLRobot" + Assegnata una configurazione q al nostro robot, calcoleremo con la cinematica diretta la corrispondente posa dell'end effector + Calcoleremo lo Jacobiano geometrico in questa posa (sfizio) + Assegneremo una posa desiderata per l'end effector, e tramite un algoritmo di cinematica inversa (sicuramente necessita anche dello Jacobiano, però nella posa attuale) calcoleremo la corrispondente configurazione desiderata qd
int main(int argc, char **argv) 
{
    if (argc < 2)//se gli argomenti immessi da terminale sono meno di 2... (0 = il ".cpp", 1 = il path per l'urdf)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]); //a partire dal secondo (1) argomento, che e' l'urdf, crea il robot (vedi funzione alla riga 6)
    int nrJnts = robot.getNrJnts(); //metodo di KDLRobot

    // Set jnt states
    std::vector<double> q(nrJnts, 1.0), qd(nrJnts, 0.0); //definisco due vettori di double: q (joint values) di lunghezza "nrJnts" e contenente solo "1.0", e qd (q_dot, joint velocities) con soli "0.0". Vedi meglio su "std::vector cppreference" su internet
    robot.update(q, qd); //inserisco questi valori nella rappresentazione del robot
    KDL::JntArray q_kdl(7); //definisco alla maniera del namespace "KDL" un vettore di joint values di dimensione fissa 7 (costruttore con argomenti)... 
    q_kdl.data << q[0], q[1], q[2], q[3], q[4], q[5], q[6]; //... e lo inizializzo  con i valori di q

    for(unsigned int i = 0; i < q_kdl.data.size(); i++)
        std::cout << "Joint " << i << " position value = " << q_kdl.data[i] << std::endl; 
    
    // Specify an end-effector: invece di avere solo la flangia del robot (il polso, parte che collega il robot all'e.e. eventuale) ha aggiunto un e.e. solo per far vedere come si fa. Funzione definita in "kdl_robot.cpp"
    robot.addEE(KDL::Frame::Identity()); //ha supposto per semplicita' che l'e.e. coincide con la flangia del robot (funzione "Identity()")

    // Direct kinematics: joint values -> e.e frame
    KDL::Frame F = robot.getEEFrame(); //getEndEffectorFrame. Metodo definito in "kdl_control.h" e "kdl_robot.cpp". Restituisce le variabili che contengono la posa dell'e.e (o equivalentemente della sua terna/frame), e riempie l'oggetto di classe "Frame"
    std::cout << "The robot end-effector frame is currently at: \n" << F << std::endl; //matrice di rotazione/orientamento + vettore posizione/traslazione della terna (frame) e.e. rispetto alla terna base

    KDL::Jacobian J = robot.getEEJacobian();//getEndEffectorJacobian. Metodo definito in "kdl_control.h" e "kdl_robot.cpp". Restituisce lo Jacobiano del nostro robot nella configurazione in cui esso si trova)
    std::cout << "The robot Jacobian is: \n" << J.data << std::endl; //matrice Jacobiana

    // Inverse kinematics
    KDL::Frame Fd = KDL::Frame(robot.getEEFrame().M, robot.getEEFrame().p - KDL::Vector(0,0,0.1)); //definisco un desired frame Fd : e' costruito da una rotazione/orientamento (robot.getEEFrame():M) e da una posizione/traslazione (.p-ecc). Alla fine sto dicendo che il mio Desired Frame è spostato di un vettore (0, 0, 0.1) rispetto a quello attuale: quindi avrà la stessa rotazione, ma posizione spostata sull'asse z di 0.1 metri in basso
    q_kdl = robot.getInvKin(q_kdl, Fd); ///getInverseKinematics. Metodo definito in "kdl_control.h" e "kdl_robot.cpp". Effettua la cinematica inversa tramite algoritmo di inversione cinematica (CLIK) basato sulle iterazioni di Newton-Raphson a partire dallo Jacobiano nella configurazione attuale "q_kdl" e la posa desiderata "Fd", e aggiorna "q_kdl"
    std::cout << "The desired robot end-effector frame is: \n" << Fd << std::endl;

    std::cout << "Inverse kinematics returned the following values..." << std::endl;
    for(unsigned int i = 0; i < q_kdl.data.size(); i++)
        std::cout << "Joint " << i << " position value = " << q_kdl.data[i] << std::endl;

    return 0;
}
