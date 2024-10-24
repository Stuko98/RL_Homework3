#include "kdl_ros_control/kdl_planner.h"


KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)                                                                          //costruttore con argomenti
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}


KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)          //costruttore con argomenti
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}


void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,                                                   //creo delle pose desiderate (frames), eseguo questa funzione, che computa la traiettoria passante per questi frames
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());                    //KDL::Path_RoundedComposite: metodo per la definizione di traiettorie con segmenti composti

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}


void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}


KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}


trajectory_point KDLPlanner::compute_trajectory(double time)                                                                    //computo la traiettoria nello spazio operativo, assegnandole un trapezoidal velocity profile
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);                //"Robotics" di Siciliano -> capitolo 4 -> eq (4.5) OCCHIO: quella di Siciliano è nello spazio dei giunti, questa è nello spazio operativo

  if(time <= accDuration_)                                                  //RISING PHASE (PARABULA)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);                // s0 + 0.5 * s_c_dot_dot*t^2
    traj.vel = ddot_traj_c*time;                                            // s_c_dot_dot * t
    traj.acc = ddot_traj_c;                                                 // s_c_dot_dot (accelerazione positiva)
  }
  else if(time <= trajDuration_-accDuration_)                               //CONSTANT PHASE (LINEAR)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);  // s0 + s_c_dot_dot * (t - t_c/2)
    traj.vel = ddot_traj_c*accDuration_;                                    // s_c_dot_dot * t_c
    traj.acc = Eigen::Vector3d::Zero();                                     // (zona lineare -> accelerazione nulla)
  }
  else                                                                      //FALLING PHASE (PARABULA)
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);   // s_f - 0.5*s_c_dot_dot * (t_f - t)^2
    traj.vel = ddot_traj_c*(trajDuration_-time);                            // s_c_dot_dot * (t_f - t)
    traj.acc = -ddot_traj_c;                                                // -(s_c_dot_dot) (accelerazione negativa)
  }

  return traj;

}



////////////////////////////////////////////////////////////////////////////////
//                              HOMEWORK2                                     //
////////////////////////////////////////////////////////////////////////////////

void KDLPlanner::trapezoidal_vel(double time, double accDuration_, double &s, double &sd, double &sdd)                          //trapezoidal velocity profile per l'ascissa curvilinea s
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  //Come da traccia:
  //si=0
  //sf=1

  double ddot_s_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)/* *(1-0) */;                                     //"Robotics" di Siciliano -> capitolo 4 -> eq (4.5)

  if(time <= accDuration_)                    //RISING PHASE (PARABULA)                                                         //Perché non hai più "(trajEnd_-trajInit_)"? Perché: --- 1) banalmente ora s appartiene [0,1], quindi hai "(1-0)" --- 2) Perché gli elementi nella parentesi sono oggetti di classe "Eigen::Vector3d" (vedi kdl_planner.h riga 73), e quindi avresti delle conversioni di tipo impossibili Eigen::Vector3d -> double                               
  {
    s = 0.5*ddot_s_c*std::pow(time,2);                              
    sd = ddot_s_c*time;                                             
    sdd = ddot_s_c;                                                 
  }
  else if(time <= trajDuration_-accDuration_) //CONSTANT PHASE (LINEAR)                               
  {
    s = 0.5*ddot_s_c*(time-accDuration_/2);  
    sd = ddot_s_c*accDuration_;                                   
    sdd = 0;                                                         
  }
  else                                        //FALLING PHASE (PARABULA)                            
  {
    s = 1-0.5*ddot_s_c*std::pow(trajDuration_-time,2);   
    sd = ddot_s_c*(trajDuration_-time);                           
    sdd = -ddot_s_c;                                               
  }
}

void KDLPlanner::cubic_polinomial(double time, double &s, double &sd, double &sdd)                                              //cubic polynomial velocity profile per l'ascissa curvilinea s
{
                                                                                                                     
  double a0=0, a1=0, a2=3/std::pow(trajDuration_,2), a3=-2/std::pow(trajDuration_,3);                                           //"Robotics" di Siciliano -> capitolo 4 -> eq (4.1). Calcolati a mano offline + verifica con Geogebra

  s=a3*std::pow(time,3)+a2*std::pow(time,2)+a1*time+a0;
  sd=3*a3*std::pow(time,2)+2*a2*time+a1;
  sdd=6*a3*time+2*a2;

}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)                                     //costruttore con 3 argomenti, tra cui il raggio della traiettoria
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;

}


trajectory_point KDLPlanner::compute_trajectory_circ_cp(double time)                                                            //computo la traiettoria circolare a partire dalla cubic polynomial curvilinear abscissa s
{                                                                                                                               //SUCCESSIVI CALCOLI: CALCOLATORE DERIVATE ONLINE + "Robotics" di Siciliano -> capitolo 4 -> 4.3.1 + 4.3.2

  double s, sd, sdd;
  trajectory_point traj;
  cubic_polinomial(time,s,sd,sdd);


  traj.pos[0]=trajInit_[0];
  traj.pos[1]=trajInit_[1]-trajRadius_*cos(2*M_PI*s);
  traj.pos[2]=trajInit_[2]-trajRadius_*sin(2*M_PI*s);

  traj.vel[0]=0;
  traj.vel[1]=2*M_PI*sd*trajRadius_*sin(2*M_PI*s);
  traj.vel[2]=-2*M_PI*sd*trajRadius_*cos(2*M_PI*s);

  traj.acc[0]=0;
  traj.acc[1]=2*M_PI*trajRadius_*(sdd*sin(2*M_PI*s)+2*M_PI*std::pow(sd,2)*cos(2*M_PI*s));
  traj.acc[2]=2*M_PI*trajRadius_*(2*M_PI*std::pow(sd,2)*sin(2*M_PI*s)-sdd*cos(2*M_PI*s));


  return traj;

}

trajectory_point KDLPlanner::compute_trajectory_circ_tv(double time)                                                            //computo la traiettoria circolare a partire dalla trapezoidal velocity curvilinear abscissa s
{

  double s, sd, sdd;
  trajectory_point traj;
  trapezoidal_vel(time,accDuration_,s,sd,sdd);

  traj.pos[0]=trajInit_[0];
  traj.pos[1]=trajInit_[1]-trajRadius_*cos(2*M_PI*s);
  traj.pos[2]=trajInit_[2]-trajRadius_*sin(2*M_PI*s);

  traj.vel[0]=0;
  traj.vel[1]=2*M_PI*sd*trajRadius_*sin(2*M_PI*s);
  traj.vel[2]=-2*M_PI*sd*trajRadius_*cos(2*M_PI*s);

  traj.acc[0]=0;
  traj.acc[1]=2*M_PI*trajRadius_*(sdd*sin(2*M_PI*s)+2*M_PI*std::pow(sd,2)*cos(2*M_PI*s));
  traj.acc[2]=2*M_PI*trajRadius_*(2*M_PI*std::pow(sd,2)*sin(2*M_PI*s)-sdd*cos(2*M_PI*s));


  return traj;

}

trajectory_point KDLPlanner::compute_trajectory_lin_cp(double time)                                                             //computo la traiettoria lineare a partire dalla trapezoidal velocity curvilinear abscissa 
{                                                                    


  double s, sd, sdd;
  trajectory_point traj;
  cubic_polinomial(time,s,sd,sdd);

 
  traj.pos=trajInit_+s*(trajEnd_-trajInit_);
  traj.vel=sd*(trajEnd_-trajInit_);
  traj.acc=sdd*(trajEnd_-trajInit_);


  return traj;

}

trajectory_point KDLPlanner::compute_trajectory_lin_tv(double time)                                                             //computo la traiettoria lineare a partire dalla trapezoidal velocity curvilinear abscissa 
{

  double s, sd, sdd;
  trajectory_point traj;
  trapezoidal_vel(time,accDuration_,s,sd,sdd);

  traj.pos=trajInit_+s*(trajEnd_-trajInit_);
  traj.vel=sd*(trajEnd_-trajInit_);
  traj.acc=sdd*(trajEnd_-trajInit_);


  return traj;

}


trajectory_point KDLPlanner::compute_trajectory(double time, int sel)                                                           //in base a "sel", scelgo quale traiettoria computare
{

  trajectory_point traj;

  switch(sel) {

    case 1:
      traj=compute_trajectory_circ_cp(time); //circular trajectory, cubic polynomial curvilinear abscissa s
    break;

    case 2:
      traj=compute_trajectory_circ_tv(time); //circular trajectory, trapezoidal velocity curvilinear abscissa s
    break;

    case 3:
      traj=compute_trajectory_lin_cp(time); //linear trajectory, cubic polynomial curvilinear abscissa s
    break;

    case 4:
      traj=compute_trajectory_lin_tv(time); //linear trajectory, trapezoidal velocity curvilinear abscissa s
    break;

  }
  
  return traj;

}

/////////////////////////////////////////////////////////////////////////////////

