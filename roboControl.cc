#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/Time.hh>
#include <boost/filesystem.hpp>
#include <ctime>
#include "aquashoko.h"


//PID values Jaw
float ProJaw = 1.0; //proportional control
float InteJaw = 0.10; //integral control
float DereJaw = 1.4; //derivative control
float iMaxYaw = 8.4; 


//PID values Pitch #1
float ProPitch1 = 0.8; //proportional control
float IntePitch1 = 0.8; //integral control
float DerePitch1 = 0.2; //derivative control
float iMaxPitch1 = 8.4; 

//PID values Pitch #2
float ProPitch2 = 0.5; //250; //proportional control
float IntePitch2 = 0.2; //integral control
float DerePitch2 = 0.35; //derivative control
float iMaxPitch2 = 8.4; 


//for use in reading joint position			
float arrayJointPositions[12] = {}; 	
//for use to request joint position
float arrayJointRequest[12] = {};	


//status
bool busy = false;
//poses  
bool store = false;
bool home = false;
//Model Rotations
int rotationCount;
bool modelCCw = false;
bool modelCw = false;
//Model Walks, number(s) indicate leading leg(s)
int walkies;
bool modelWalk = false;



//pose transitions
bool step1,  step2,  step3,  step4,  step5,  step6,  step7,  step8,  step9,  step10,
	 step11, step12, step13, step14, step15, step16, step17, step18, step19, step20,
	 step21, step22, step23, step24, step25, step26, step27, step28, step29, step30, 
	 step31, step32, step33, step34, step35, step36, step37, step38, step39, step40,
	 step41, step42, step43, step44, step45, step46, step47, step48, step49, step50 = true;



namespace gazebo
{
  class roboControl : public ModelPlugin
  {

  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {





		/* Setup Aquashoko-Ach interface */
		std::cout << "aquashoko_init() status: " <<  aquashoko_init() << std::endl;


		// Store the pointer to the model
		this->model = _parent;
		
		
		//  -----------------------------  Joint Controller Setup -----------------------------
		//Store the pointer to controller
		this->ShokoControl = new physics::JointController(_parent);
		// Setup a PID controller
 		this->pid11 = common::PID(ProJaw, InteJaw, DereJaw, iMaxYaw, -iMaxYaw);
 		this->pid21 = common::PID(ProJaw, InteJaw, DereJaw, iMaxYaw, -iMaxYaw);
 		this->pid31 = common::PID(ProJaw, InteJaw, DereJaw, iMaxYaw, -iMaxYaw);
 		this->pid41 = common::PID(ProJaw, InteJaw, DereJaw, iMaxYaw, -iMaxYaw);
 			this->pid12 = common::PID(ProPitch1, IntePitch1, DerePitch1, iMaxPitch1, -iMaxPitch1);
 			this->pid22 = common::PID(ProPitch1, IntePitch1, DerePitch1, iMaxPitch1, -iMaxPitch1);
 			this->pid32 = common::PID(ProPitch1, IntePitch1, DerePitch1, iMaxPitch1, -iMaxPitch1);
 			this->pid42 = common::PID(ProPitch1, IntePitch1, DerePitch1, iMaxPitch1, -iMaxPitch1);
 				this->pid13 = common::PID(ProPitch2, IntePitch2, DerePitch2, iMaxPitch2, -iMaxPitch2);
 				this->pid23 = common::PID(ProPitch2, IntePitch2, DerePitch2, iMaxPitch2, -iMaxPitch2);
 				this->pid33 = common::PID(ProPitch2, IntePitch2, DerePitch2, iMaxPitch2, -iMaxPitch2);
 				this->pid43 = common::PID(ProPitch2, IntePitch2, DerePitch2, iMaxPitch2, -iMaxPitch2);
 				
 		this->ID11 = this->model->GetJoint("jaw11");
 		this->ID21 = this->model->GetJoint("jaw21");
 		this->ID31 = this->model->GetJoint("jaw31");
 		this->ID41 = this->model->GetJoint("jaw41");
 			this->ID12 = this->model->GetJoint("pitch12");
 			this->ID22 = this->model->GetJoint("pitch22");
 			this->ID32 = this->model->GetJoint("pitch32");
 			this->ID42 = this->model->GetJoint("pitch42");
 				this->ID13 = this->model->GetJoint("pitch13");
 				this->ID23 = this->model->GetJoint("pitch23");
 				this->ID33 = this->model->GetJoint("pitch33");
 				this->ID43 = this->model->GetJoint("pitch43");

 		this->model->GetJointController()->SetPositionPID(this->ID11->GetScopedName(), this->pid11);
  		this->model->GetJointController()->SetPositionPID(this->ID21->GetScopedName(), this->pid21);
		this->model->GetJointController()->SetPositionPID(this->ID31->GetScopedName(), this->pid31);
		this->model->GetJointController()->SetPositionPID(this->ID41->GetScopedName(), this->pid41);
			this->model->GetJointController()->SetPositionPID(this->ID12->GetScopedName(), this->pid12);
			this->model->GetJointController()->SetPositionPID(this->ID22->GetScopedName(), this->pid22);
			this->model->GetJointController()->SetPositionPID(this->ID32->GetScopedName(), this->pid32);
			this->model->GetJointController()->SetPositionPID(this->ID42->GetScopedName(), this->pid42);
				this->model->GetJointController()->SetPositionPID(this->ID13->GetScopedName(), this->pid13);
				this->model->GetJointController()->SetPositionPID(this->ID23->GetScopedName(), this->pid23);
				this->model->GetJointController()->SetPositionPID(this->ID33->GetScopedName(), this->pid33);
				this->model->GetJointController()->SetPositionPID(this->ID43->GetScopedName(), this->pid43);
			//  -----------------------------  Joint Controller Setup End  -----------------------------	
		
		
		
		
		// Listen to the update event. This event is broadcast every
		// simulation iteration.
    	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&roboControl::OnUpdate, this, _1));
    	std::cout << "*************************** roboControl ******************************" << std::endl;	
    	std::cout << "--------- Controller PID Jaw:" <<" P:" << ProJaw <<" I:" << InteJaw << " D:"<< DereJaw <<std::endl;
    	std::cout << "------ Controller PID Pitch1:" <<" P:" << ProPitch1 <<" I:" << IntePitch1 << " D:"<< DerePitch1 <<std::endl;
    	std::cout << "------ Controller PID Pitch2:" <<" P:" << ProPitch2 <<" I:" << IntePitch2 << " D:"<< DerePitch2 <<std::endl;
	
    }
     
  
     
// _----__--_-_-__-_-- Called by the world update start event  -----___-----__------__----___----loop that makes the stuff happen _______----______------_________------_____---
	public: void OnUpdate(const common::UpdateInfo & /*_info*/)
	{
		//Home();
		achControl();

/*	

		if(store == false)
			{
				Store();
			}
		if(store == true && home == false)
			{
				Home();
			}				
		if( modelWalk == false && home == true)
			{
				ModelWalkies(6);
			}				
*/
	}
	
    //  ----------------------------- Pointer to the model -----------------------------
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    public: physics::JointPtr ID11;
    public: physics::JointPtr ID21;
    public: physics::JointPtr ID31;
    public: physics::JointPtr ID41;
    	public: physics::JointPtr ID12;
    	public: physics::JointPtr ID22;
    	public: physics::JointPtr ID32;
    	public: physics::JointPtr ID42;
    		public: physics::JointPtr ID13;
    		public: physics::JointPtr ID23;
    		public: physics::JointPtr ID33;
    		public: physics::JointPtr ID43;
	//important Stuff
    private: event::ConnectionPtr updateConnection;
    //Pointer to controller
    private: physics::JointController * ShokoControl;
	/// \brief A PID controller for the joint.
	private: common::PID pid11,pid21,pid31,pid41,
							pid12,pid22,pid32,pid42,
								pid13,pid23,pid33,pid43;


// ----------------------------- Start Functions -----------------------------


    void achControl()
    {
	/* get update */
	
	aquashoko_pull();

	/* set ref to gazebo */
	int ii = 0;
        //std::cout << "achControl()" << std::endl;
	for(int i = 0; i < AQUASHOKO_LEG_NUM; i++){
		for(int j = 0; j < AQUASHOKO_LEG_JOINT_NUM; j++){
			arrayJointRequest[ii] = aquashoko_get(i,j);
    			//std::cout << "arrayJointRequest[" << ii <<"] = " << arrayJointRequest[ii]<< std::endl;	
			ii++;
		}
	}
 
    setJointsPosition(300);

    printJointPositions();

    }




    void getJointPositions()
    {
    	math::Angle id11 = this->model->GetJoint("jaw11")-> GetAngle(0);
    	arrayJointPositions[0] = id11.Degree();
    		math::Angle id21 = this->model->GetJoint("jaw21")-> GetAngle(0);
    		arrayJointPositions[1] = id21.Degree();
				math::Angle id31 = this->model->GetJoint("jaw31")-> GetAngle(0);
    			arrayJointPositions[2] = id31.Degree();
    				math::Angle id41 = this->model->GetJoint("jaw41")-> GetAngle(0);
    				arrayJointPositions[3] = id41.Degree();
    	math::Angle id12 = this->model->GetJoint("pitch12")-> GetAngle(0);
    	arrayJointPositions[4] = id12.Degree();
    		math::Angle id22 = this->model->GetJoint("pitch22")-> GetAngle(0);
    		arrayJointPositions[5] = id22.Degree();
				math::Angle id32 = this->model->GetJoint("pitch32")-> GetAngle(0);
    			arrayJointPositions[6] = id32.Degree();
    				math::Angle id42 = this->model->GetJoint("pitch42")-> GetAngle(0);
    				arrayJointPositions[7] = id42.Degree();
    	math::Angle id13 = this->model->GetJoint("pitch13")-> GetAngle(0);
    	arrayJointPositions[8] = id13.Degree();
    		math::Angle id23 = this->model->GetJoint("pitch23")-> GetAngle(0);
    		arrayJointPositions[9] = id23.Degree();
				math::Angle id33 = this->model->GetJoint("pitch33")-> GetAngle(0);
    			arrayJointPositions[10] = id33.Degree();
    				math::Angle id43 = this->model->GetJoint("pitch43")-> GetAngle(0);
    				arrayJointPositions[11] = id43.Degree();	
    }
    
    
    void  printJointPositions()
    {   				
    	getJointPositions();
    	//print current positions to terminal			
    	//std::cout <<" p11:"<< arrayJointPositions[0] << " p21:"<< arrayJointPositions[1] << " p31:"<< arrayJointPositions[2] << " p41:"<< arrayJointPositions[3] << std::endl;
    	//std::cout << " p12:"<< arrayJointPositions[4] << " p22:"<< arrayJointPositions[5] << " p32:"<< arrayJointPositions[6] << " p42:"<< arrayJointPositions[7] << std::endl;
    	//std::cout << " p13:"<< arrayJointPositions[8] << " p23:"<< arrayJointPositions[9] << " p33:"<< arrayJointPositions[10] << " p43:"<< arrayJointPositions[11] << std::endl;
    	
    	aquashoko_set(0,0, arrayJointPositions[0]);
    	aquashoko_set(0,1, arrayJointPositions[4]);
    	aquashoko_set(0,2, arrayJointPositions[8]);
    	aquashoko_set(1,0, arrayJointPositions[1]);
    	aquashoko_set(1,1, arrayJointPositions[5]);
    	aquashoko_set(1,2, arrayJointPositions[9]);
    	aquashoko_set(2,0, arrayJointPositions[2]);
    	aquashoko_set(2,1, arrayJointPositions[6]);
    	aquashoko_set(2,2, arrayJointPositions[10]);
    	aquashoko_set(3,0, arrayJointPositions[3]);
    	aquashoko_set(3,1, arrayJointPositions[7]);
    	aquashoko_set(3,2, arrayJointPositions[11]);

    	aquashoko_put();
    	
    }
    
    
    void setJointsPosition(float rpm)     //int rpm sets max joint speed
	{
		float r11, r21, r31, r41, 		//jaw joints
				r12, r22, r32, r42, 	//first pitch joint
					r13, r23, r33, r43; 	//end pitch joint
		
		//convert from degree to radians			
		r11 = arrayJointRequest[0] * 3.1415 / 180;
		r21 = arrayJointRequest[3] * 3.1415 / 180;
		r31 = arrayJointRequest[6] * 3.1415 / 180;
		r41 = arrayJointRequest[9] * 3.1415 / 180;
			r12 = arrayJointRequest[1] * 3.1415 / 180;
			r22 = arrayJointRequest[4] * 3.1415 / 180;
			r32 = arrayJointRequest[7] * 3.1415 / 180;
			r42 = arrayJointRequest[10] * 3.1415 / 180;
				r13 = arrayJointRequest[2] * 3.1415 / 180;
				r23 = arrayJointRequest[5] * 3.1415 / 180;
				r33 = arrayJointRequest[8] * 3.1415 / 180;
				r43 = arrayJointRequest[11] * 3.1415 / 180;

		//Set controller Target Positions
		this->model->GetJointController()->SetPositionTarget(this->ID11->GetScopedName(), r11);
		this->model->GetJointController()->SetPositionTarget(this->ID21->GetScopedName(), r21);
		this->model->GetJointController()->SetPositionTarget(this->ID31->GetScopedName(), r31);
		this->model->GetJointController()->SetPositionTarget(this->ID41->GetScopedName(), r41);
			this->model->GetJointController()->SetPositionTarget(this->ID12->GetScopedName(), r12);
			this->model->GetJointController()->SetPositionTarget(this->ID22->GetScopedName(), r22);
			this->model->GetJointController()->SetPositionTarget(this->ID32->GetScopedName(), r32);
			this->model->GetJointController()->SetPositionTarget(this->ID42->GetScopedName(), r42);
				this->model->GetJointController()->SetPositionTarget(this->ID13->GetScopedName(), r13);
				this->model->GetJointController()->SetPositionTarget(this->ID23->GetScopedName(), r23);
				this->model->GetJointController()->SetPositionTarget(this->ID33->GetScopedName(), r33);
				this->model->GetJointController()->SetPositionTarget(this->ID43->GetScopedName(), r43);
		
		
		float rps;
		//convert rpm to 
		rps = rpm * 0.1047;
		//Set joint max velocity
		this->model->GetJoint("jaw11")->SetVelocityLimit(0, rps);
		this->model->GetJoint("jaw21")->SetVelocityLimit(0, rps);
		this->model->GetJoint("jaw31")->SetVelocityLimit(0, rps);
		this->model->GetJoint("jaw41")->SetVelocityLimit(0, rps);
			this->model->GetJoint("pitch12")->SetVelocityLimit(0, rps);
			this->model->GetJoint("pitch22")->SetVelocityLimit(0, rps);
			this->model->GetJoint("pitch32")->SetVelocityLimit(0, rps);
			this->model->GetJoint("pitch42")->SetVelocityLimit(0, rps);
				this->model->GetJoint("pitch13")->SetVelocityLimit(0, rps);
				this->model->GetJoint("pitch23")->SetVelocityLimit(0, rps);
				this->model->GetJoint("pitch33")->SetVelocityLimit(0, rps);
				this->model->GetJoint("pitch43")->SetVelocityLimit(0, rps);

	}

	// Returns true if target joint positions reached 
	bool compareRequestAndPossition(float precision )
	{
		printJointPositions();
		int b = 0;
		for( int a = 0; a < 12; a = a + 1 ) 
		{
      		if( arrayJointRequest[a] < (arrayJointPositions[a] + precision )  &&  arrayJointRequest[a] > (arrayJointPositions[a] - precision )  )  //allows for +-precision degrees of miss alignment
      		{
      		}
      		else
      		{
      			b++;
      		}
   		}	
   		if ( b == 0 )
   		{
   			std::cout << "*************************** Target Reached ******************************" << std::endl;
   			return true;	
   		}
   		else
   		{
   			return false;
   		}
	}

	// ---------------------------------  Poses ------------------------------------------------
	void Store()
	{
		if(busy == false)
			{
				busy = true;
				//Store home joint positions
				arrayJointRequest[0] = 0; 	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 	arrayJointRequest[3] = 0; 
				arrayJointRequest[4] = 0; 	arrayJointRequest[5] = 0;	arrayJointRequest[6] = 0; 	arrayJointRequest[7] = 0;
				arrayJointRequest[8] = 0; 	arrayJointRequest[9] = 0; 	arrayJointRequest[10] = 0; 	arrayJointRequest[11] = 0;
				setJointsPosition(3);
			}
		if(compareRequestAndPossition(10))
		{
			printJointPositions();
			busy = false; 
			store = true;
		}
		else
		{
			printJointPositions();
		}
	}//______________ end Store() ___________________
	
	
	void Home()
	{
		if(busy == false )
			{
			busy = true;
			//steps to be taken for home pose
			step1 = false;  step2 = false;  step3 = false;  step4 = false;  
			std::cout << "*************************** Home Pose In Progress ******************************" << std::endl;
			}
			
		//Movements 
		if(step1 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 	arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = 0;	arrayJointRequest[5] = 0; 	arrayJointRequest[6] = 0; 	arrayJointRequest[7] = 0;
			arrayJointRequest[8] = 0;	arrayJointRequest[9] = 0; arrayJointRequest[10] = 0; 	arrayJointRequest[11] = 0;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Home Pose In Progress: Step1 ******************************" << std::endl;
			if(compareRequestAndPossition(2))
				{
				step1 = true;
				}
			}
		else if(step2 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = 20;	arrayJointRequest[5] = 20; 	arrayJointRequest[6] = 20; 		arrayJointRequest[7] = 20;
			arrayJointRequest[8] = -0;	arrayJointRequest[9] = -0; arrayJointRequest[10] = -0; 	arrayJointRequest[11] = -0;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Home Pose In Progress: Step2 ******************************" << std::endl;
			if(compareRequestAndPossition(.5))
				{
				step2 = true;
				}
			}
		else if(step3 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = 20;	arrayJointRequest[5] = 20; 	arrayJointRequest[6] = 20; 		arrayJointRequest[7] = 20;
			arrayJointRequest[8] = -20;	arrayJointRequest[9] = -20; arrayJointRequest[10] = -20; 	arrayJointRequest[11] = -20;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Home Pose In Progress: Step3 ******************************" << std::endl;
			if(compareRequestAndPossition(.5))
				{
				step3 = true;
				}
			}
		else if(step4 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -3;	arrayJointRequest[5] = -3; 	arrayJointRequest[6] = -3; 		arrayJointRequest[7] = -3;
			arrayJointRequest[8] = -29;	arrayJointRequest[9] = -29; arrayJointRequest[10] = -29; 	arrayJointRequest[11] = -29;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Home Pose In Progress: Step4 ******************************" << std::endl;
			if(compareRequestAndPossition(.5))
				{
				step4 = true;
				}
			}	
		else if(step5 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Home Pose In Progress: Step5 ******************************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step5 = true;
				}
			}		
		else 
			{
			std::cout << "*************************** Home Pose Complete  ******************************" << std::endl;
			busy = false; 
			home = true;
			printJointPositions();
			}
	} //_____________________ end Home() ___________________________________________
	
	
	void ModelWalkies (int number)
	{
		if(busy == false)
			{
			busy = true;
			//steps to be taken for home pose
			step1 = false; 	step2 = false;  step3 = false;  step4 = false;  step5 = false;  step6 = false; 	step7 = false; 	step8 = false;  step9 = false; 	step10 = false; 
			step11 = false; step12 = false; step13 = false; step14 = false; step15 = false; step16 = false;	step17 = false; step18 = false; step19 = false; step20 = false; 
			step21 = false; step22 = false; step23 = false;
			std::cout << "********** Model Walkies In Progress - loop:"<< walkies << "/" << number << "************" << std::endl;
			}
		if(step1 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 1 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step1 = true;
				}
			}
		else if(step2 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = 15; 	arrayJointRequest[2] = -15; 	arrayJointRequest[3] = -34;    //shift CoM to  leg 1-4
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -43; arrayJointRequest[6] = -43; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -69; arrayJointRequest[10] = -69; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 2 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step2 = true;
				}
			}
		else if(step3 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = 15; 	arrayJointRequest[2] = -24; 	arrayJointRequest[3] = -34;    //lift and rotate 1/2 way leg 3
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -43; arrayJointRequest[6] = -10; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -69; arrayJointRequest[10] = -40; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 2 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step3 = true;
				}
			}
		else if(step4 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = 15; 	arrayJointRequest[2] = -33; 	arrayJointRequest[3] = -34;    //place leg 3
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -43; arrayJointRequest[6] = -38; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -69; arrayJointRequest[10] = -59; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 4 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step4 = true;
				}
			}
		else if(step5 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = 9.5; arrayJointRequest[2] = -33; 	arrayJointRequest[3] = -34;    //lift and rotate 1/2 way leg 2
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -10; arrayJointRequest[6] = -38; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -40; arrayJointRequest[10] = -59; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 5 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step5 = true;
				}
			}
		else if(step6 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = 2; 	arrayJointRequest[2] = -33; 	arrayJointRequest[3] = -34;    //place leg 2
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -52; arrayJointRequest[6] = -38; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -85; arrayJointRequest[10] = -59; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 6 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step6 = true;
				}
			}	
		else if(step7 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -0;	arrayJointRequest[1] = -16; arrayJointRequest[2] = -31; 	arrayJointRequest[3] = -0;    //shift CoM to center(1)
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -44; arrayJointRequest[6] = -24; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -69; arrayJointRequest[10] = -32; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 7 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step7 = true;
				}
			}
		else if(step8 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -33;	arrayJointRequest[1] = -0; 	arrayJointRequest[2] = -0; 		arrayJointRequest[3] = -15;    //shift CoM to center(2)
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -43;
			arrayJointRequest[8] = -30;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -69;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 8 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step8 = true;
				}
			}
		else if(step9 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -33;	arrayJointRequest[1] = -30; arrayJointRequest[2] = 34; 		arrayJointRequest[3] = 2;    //shift CoM to leg 2-3
			arrayJointRequest[4] = -38;	arrayJointRequest[5] = -22; arrayJointRequest[6] = -25; 	arrayJointRequest[7] = -52;
			arrayJointRequest[8] = -59;	arrayJointRequest[9] = -28; arrayJointRequest[10] = -36; 	arrayJointRequest[11] = -85;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 9 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step9 = true;
				}
			}	
		else if(step10 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -33;	arrayJointRequest[1] = -30; arrayJointRequest[2] = 34; 		arrayJointRequest[3] = 18;    	//lift and rotate 1/2 way  leg 4
			arrayJointRequest[4] = -38;	arrayJointRequest[5] = -22; arrayJointRequest[6] = -25; 	arrayJointRequest[7] = -10;
			arrayJointRequest[8] = -59;	arrayJointRequest[9] = -28; arrayJointRequest[10] = -36; 	arrayJointRequest[11] = -40;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 10 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step10 = true;
				}
			}	
		else if(step11 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -33;	arrayJointRequest[1] = -30; arrayJointRequest[2] = 34; 		arrayJointRequest[3] = 33;    	//place leg 4
			arrayJointRequest[4] = -38;	arrayJointRequest[5] = -22; arrayJointRequest[6] = -25; 	arrayJointRequest[7] = -38;
			arrayJointRequest[8] = -59;	arrayJointRequest[9] = -28; arrayJointRequest[10] = -36; 	arrayJointRequest[11] = -59;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 11 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step11 = true;
				}
			}	
		else if(step12 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -9.5;	arrayJointRequest[1] = -30; arrayJointRequest[2] = 34; 		arrayJointRequest[3] = 33;    	//lift and rotate 1/2 way 1
			arrayJointRequest[4] = -10;	arrayJointRequest[5] = -22; arrayJointRequest[6] = -25; 	arrayJointRequest[7] = -38;
			arrayJointRequest[8] = -40;	arrayJointRequest[9] = -28; arrayJointRequest[10] = -36; 	arrayJointRequest[11] = -59;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 12 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step12 = true;
				}
			}	
		else if(step13 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -2;	arrayJointRequest[1] = -30; arrayJointRequest[2] = 34; 		arrayJointRequest[3] = 33;    	//place leg 1
			arrayJointRequest[4] = -52;	arrayJointRequest[5] = -22; arrayJointRequest[6] = -25; 	arrayJointRequest[7] = -38;
			arrayJointRequest[8] = -85;	arrayJointRequest[9] = -28; arrayJointRequest[10] = -36; 	arrayJointRequest[11] = -59;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 13 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step13 = true;
				}
			}	
		else if(step14 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] =  16;	arrayJointRequest[1] = -0; 	arrayJointRequest[2] =  0; 		arrayJointRequest[3] = 31;    //shift CoM to center(1)
			arrayJointRequest[4] = -44;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -24;
			arrayJointRequest[8] = -69;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -32;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 14 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step14 = true;
				}
			}	
		else if(step15 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -0;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = 15; 		arrayJointRequest[3] = -0;    //shift CoM to center(2)
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -22; arrayJointRequest[6] = -43; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -30; arrayJointRequest[10] = -69; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 15 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step15 = true;
				}
			}
		else if(step16 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 	30;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = -2; 		arrayJointRequest[3] = -34;    //shift CoM to leg 1-2
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -38; arrayJointRequest[6] = -52; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -59; arrayJointRequest[10] = -85; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 16 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step16 = true;
				}
			}		
		else if(step17 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 	30;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = -18; 	arrayJointRequest[3] = -34;    //lift and rotate 1/2 way  leg 3
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -38; arrayJointRequest[6] = -10; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -59; arrayJointRequest[10] = -40; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 17 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step17 = true;
				}
			}	
		else if(step18 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 	30;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = -33; 	arrayJointRequest[3] = -34;    //place  leg 3
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -38; arrayJointRequest[6] = -38; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -59; arrayJointRequest[10] = -59; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 18 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step18 = true;
				walkies++;
				if ( walkies < number)
					{
					step5 = false;  step6 = false; step7 = false;  step8 = false;  step9 = false; step10 = false; 
					step11 = false;  step12 = false;  step13 = false;  step14 = false;  step15 = false;  step16 = false; step17 = false;  step18 = false;
					}
				}
			}	
		else if(step19 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 	30;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = -24; 	arrayJointRequest[3] = -34;    //lift and rotate 1/2 way  leg 3
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -38; arrayJointRequest[6] = -10; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -59; arrayJointRequest[10] = -40; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 19 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step19 = true;
				}
			}	
		else if(step20 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 	30;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = -15; 	arrayJointRequest[3] = -34;    //place  leg 3
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -38; arrayJointRequest[6] = -43; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -59; arrayJointRequest[10] = -69; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 20 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step20 = true;
				}
			}
		else if(step21 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 	30;	arrayJointRequest[1] = 17; 	arrayJointRequest[2] = -15; 	arrayJointRequest[3] = -34;    //lift and rotate 1/2 way leg 2
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -10; arrayJointRequest[6] = -43; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -40; arrayJointRequest[10] = -69; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 21 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step21 = true;
				}
			}
		else if(step22 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = 15; 	arrayJointRequest[2] = -15; 	arrayJointRequest[3] = -34;    //place leg 2
			arrayJointRequest[4] = -22;	arrayJointRequest[5] = -43; arrayJointRequest[6] = -43; 	arrayJointRequest[7] = -25;
			arrayJointRequest[8] = -28;	arrayJointRequest[9] = -69; arrayJointRequest[10] = -69; 	arrayJointRequest[11] = -36;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 22 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step22 = true;
				}
			}
		else if(step23 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0;    //shift CoM to Center
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Walkies In Progress Step 23 - loop:"<< walkies << "/" << number << "************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step23 = true;
				}
			}		
		else 
			{
			std::cout << "*************************** Walkies Complete  ******************************" << std::endl;
			printJointPositions();
			busy = false; 
			modelWalk = true;
			}
	}//_____________________ end ModelWalkies() ___________________________________________
	

	
// ----------------------------- End Functions -----------------------------	

};
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(roboControl)
}


/*	


	
	void ModelRotate(int number , float angle)  //number of loops, degree desired per loop
	{
		
		if(busy == false)
			{
			busy = true;
			//steps to be taken for home pose
			step1 = false;  step2 = false;  step3 = false;  step4 = false;  step5 = false;  step6 = false; step7 = false;  step8 = false;  step9 = false; step10 = false; 
			step11 = false;  step12 = false;  step13 = false;  step14 = false;  step15 = false;  step16 = false; step17 = false;  step18 = false;  step19 = false; step20 = false; 
			step21 = false;
			std::cout << "********** Model Rotation In Progress- Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			}
			
		if(step1 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 1 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step1 = true;
				}
			}		
		else if(step2 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] =  30; arrayJointRequest[2] = 0; 	arrayJointRequest[3] = -30; 
			arrayJointRequest[4] = -26;	arrayJointRequest[5] = -34; arrayJointRequest[6] = -45; 	arrayJointRequest[7] = -34;
			arrayJointRequest[8] = -19;	arrayJointRequest[9] = -47; arrayJointRequest[10] = -67; 	arrayJointRequest[11] = -47;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 2 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step2 = true;
				}
			}	
		else if(step3 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] =  30; arrayJointRequest[2] = 0; 		arrayJointRequest[3] = -30; 	
			arrayJointRequest[4] = -26;	arrayJointRequest[5] = -34; arrayJointRequest[6] = -10; 	arrayJointRequest[7] = -34;		//lift 3
			arrayJointRequest[8] = -19;	arrayJointRequest[9] = -47; arrayJointRequest[10] = -30; 	arrayJointRequest[11] = -47;	//lift 3
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 3 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step3 = true;
				}
			}	
		else if(step4 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -angle;	arrayJointRequest[1] = 30 - angle; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = -30-angle;  		//subtract 20: ID 4,1,2     
			arrayJointRequest[4] = -26;		arrayJointRequest[5] = -34; 		arrayJointRequest[6] = -10; 	arrayJointRequest[7] = -34;   	//lift 3
			arrayJointRequest[8] = -19;		arrayJointRequest[9] = -47;		 	arrayJointRequest[10] = -30; 	arrayJointRequest[11] = -47;	//lift 3
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 4 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step4 = true;
				}
			}	
		else if(step5 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -angle;	arrayJointRequest[1] =  30 - angle; arrayJointRequest[2] = 0; 		arrayJointRequest[3] = -30 - angle;  		//subtract "angle": ID 4,1,2     
			arrayJointRequest[4] = -26;		arrayJointRequest[5] = -34; 		arrayJointRequest[6] = -45; 	arrayJointRequest[7] = -34;   	
			arrayJointRequest[8] = -19;		arrayJointRequest[9] = -47; 		arrayJointRequest[10] = -67; 	arrayJointRequest[11] = -47;	
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 5 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step5 = true;
				}
			}		
		else if(step6 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -angle;	arrayJointRequest[1] = -angle; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = -angle; 		//subtract 20: ID 4,1,2     
			arrayJointRequest[4] = -30;		arrayJointRequest[5] = -30; 	arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;		arrayJointRequest[9] = -45; 	arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 6 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step6 = true;
				}
			}
		else if(step7 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -angle;	arrayJointRequest[1] = -30-angle; 	arrayJointRequest[2] = -0; 		arrayJointRequest[3] = 30-angle; 		//subtract 20: ID 4,1,2     
			arrayJointRequest[4] = -45;		arrayJointRequest[5] = -34; 		arrayJointRequest[6] = -26; 	arrayJointRequest[7] = -34;
			arrayJointRequest[8] = -67;		arrayJointRequest[9] = -47; 		arrayJointRequest[10] = -19; 	arrayJointRequest[11] = -47;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 7 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step7 = true;
				}
			}	
		else if(step8 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -angle;	arrayJointRequest[1] = -30-angle; 	arrayJointRequest[2] = -0; 		arrayJointRequest[3] = 30-angle; 		//subtract 20: ID 4,1,2    
			arrayJointRequest[4] = -10;	arrayJointRequest[5] = -34; arrayJointRequest[6] = -26; 	arrayJointRequest[7] = -34;		//lift 1
			arrayJointRequest[8] = -40;	arrayJointRequest[9] = -47; arrayJointRequest[10] = -19; 	arrayJointRequest[11] = -47;	//lift 1
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 8 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step8 = true;
				}
			}
		else if(step9 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -0;	arrayJointRequest[1] = -30 - angle; arrayJointRequest[2] = -0; 		arrayJointRequest[3] = 30- angle; 		//subtract 20: ID 4,2     
			arrayJointRequest[4] = -10;	arrayJointRequest[5] = -34; 		arrayJointRequest[6] = -26; 	arrayJointRequest[7] = -34;		//lift 1
			arrayJointRequest[8] = -30;	arrayJointRequest[9] = -47; 		arrayJointRequest[10] = -19; 	arrayJointRequest[11] = -47;	//lift 1
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 9 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step9 = true;
				}
			}
		else if(step10 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -0;	arrayJointRequest[1] = -30 - angle; arrayJointRequest[2] = -0; 		arrayJointRequest[3] = 30- angle; 		//subtract 20: ID 4,2      
			arrayJointRequest[4] = -45;	arrayJointRequest[5] = -34; 		arrayJointRequest[6] = -26; 	arrayJointRequest[7] = -34;		
			arrayJointRequest[8] = -67;	arrayJointRequest[9] = -47; 		arrayJointRequest[10] = -19; 	arrayJointRequest[11] = -47;	
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 10 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step10 = true;
				}
			}
		else if(step11 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = -angle; arrayJointRequest[2] = 0; 	arrayJointRequest[3] = -angle; 		//subtract 20: ID 4,2
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 11 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step11 = true;
				}
			}	
		else if(step12 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -30;	arrayJointRequest[1] =  -angle; arrayJointRequest[2] = 30; 		arrayJointRequest[3] = -angle; 		//subtract 20: ID 4,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -26; 	arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -45;
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -19; 	arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -67;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 12 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step12 = true;
				}
			}
		else if(step13 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -30;	arrayJointRequest[1] =  -angle; arrayJointRequest[2] = 30; 	arrayJointRequest[3] = -angle; 		//subtract 20: ID 4,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -26; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -10;			//lift 4
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -19; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -30;		//lift 4
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 13 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step13 = true;
				}
			}
		else if(step14 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -30;	arrayJointRequest[1] =  -angle; arrayJointRequest[2] = 30; 	arrayJointRequest[3] = -0; 		//subtract 20: ID ,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -26; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -10;			//lift 4
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -19; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -30;		//lift 4
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 14 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step14 = true;
				}
			}
		else if(step15 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -30;	arrayJointRequest[1] =  -angle; arrayJointRequest[2] = 30; 	arrayJointRequest[3] = -0; 		//subtract 20: ID ,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -26; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -45;			
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -19; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -67;		
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 15 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step15 = true;
				}
			}
		else if(step16 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = -angle; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 		//subtract 20: ID ,2
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; 	arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; 	arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 16 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step16 = true;
				}
			}
		else if(step17 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = -angle; arrayJointRequest[2] = -30; 		arrayJointRequest[3] = -0; 		//subtract 20: ID ,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -45; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -26;
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -67; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -19;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 17 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step17 = true;
				}
			}
		else if(step18 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = -angle; arrayJointRequest[2] = -30; 	arrayJointRequest[3] = -0; 		//subtract 20: ID ,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -10; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -26;		//lift2
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -30; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -19;	//lift2
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 18 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step18 = true;
				}
			}
		else if(step19 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] = -0; arrayJointRequest[2] = -30; 	arrayJointRequest[3] = -0; 		//subtract 20: ID ,2
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -10; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -26;		//lift2
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -30; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -19;	//lift2
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 19 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step19 = true;
				}
			}
		else if(step20 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] =  0; arrayJointRequest[2] = -30; 		arrayJointRequest[3] = -0; 
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -45; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -26;
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -67; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -19;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 20 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step20 = true;
				}
			}	
		else if(step21 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "********** Model Rotation In Progress Step 21 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step21 = true;
				}
			}	
			
		else
			{
			std::cout << "********** Model Rotation In Progress Step 1 - Loop:" << rotationCount<<"/" << number <<" Angel Per Loop:"<< angle << " ************" << std::endl;
			printJointPositions();
			busy = false; 
			rotationCount++;	
			if (angle > 0)
				{
				if ( rotationCount == number)
					{
					std::cout << "*************************** Model Rotation Complete  ******************************" << std::endl;
					modelCCw = true;
					rotationCount = 0;
					}
				}
			if (angle < 0)
				{
				if ( rotationCount == number)
					{
					std::cout << "*************************** Model Rotation Complete  ******************************" << std::endl;
					modelCw = true;
					rotationCount = 0;
					}
				}	
			}
	}// _________________________ end ModelCCw()__________________________





	
	void CoMCCw()
	{
		if(busy == false)
			{
			busy = true;
			//steps to be taken for home pose
			step1 = false;  step2 = false;  step3 = false;  step4 = false;  step5 = false;  step6 = false; step7 = false;  step8 = false;  step9 = false; step10 = false; 
			std::cout << "*************************** Model CoMCCw Progress ******************************" << std::endl;
			}
		if(step1 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw  leg #1 Progress: Step1 ******************************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step1 = true;
				}
			}	
		else if(step2 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] =  30; arrayJointRequest[2] = 0; 	arrayJointRequest[3] = -30; 
			arrayJointRequest[4] = -26;	arrayJointRequest[5] = -34; arrayJointRequest[6] = -45; 	arrayJointRequest[7] = -34;
			arrayJointRequest[8] = -19;	arrayJointRequest[9] = -47; arrayJointRequest[10] = -67; 	arrayJointRequest[11] = -47;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #1 Progress: Step2 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step2 = true;
				}
			}
		else if(step3 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -33;	arrayJointRequest[1] = 33; 	arrayJointRequest[2] = 17; 		arrayJointRequest[3] = -17; 
			arrayJointRequest[4] = -28;	arrayJointRequest[5] = -28; arrayJointRequest[6] = -41; 	arrayJointRequest[7] = -41;
			arrayJointRequest[8] = -30;	arrayJointRequest[9] = -30; arrayJointRequest[10] = -60; 	arrayJointRequest[11] = -60;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #1-2 Progress: Step3 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step3 = true;
				}
			}
		else if(step4 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -30;	arrayJointRequest[1] =  0; arrayJointRequest[2] = 30; 		arrayJointRequest[3] = -0; 
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -26; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -45;
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -19; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -67;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #2 Progress: Step4 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step4 = true;
				}
			}
		else if(step5 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -17;	arrayJointRequest[1] = -33; arrayJointRequest[2] = 33; 		arrayJointRequest[3] = 17; 
			arrayJointRequest[4] = -41;	arrayJointRequest[5] = -28; arrayJointRequest[6] = -28; 	arrayJointRequest[7] = -41;
			arrayJointRequest[8] = -60;	arrayJointRequest[9] = -30; arrayJointRequest[10] = -30; 	arrayJointRequest[11] = -60;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #2-3 Progress: Step5 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step5 = true;
				}
			}
		else if(step6 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = -0;	arrayJointRequest[1] = -30; arrayJointRequest[2] = -0; 		arrayJointRequest[3] = 30; 
			arrayJointRequest[4] = -45;	arrayJointRequest[5] = -34; arrayJointRequest[6] = -26; 	arrayJointRequest[7] = -34;
			arrayJointRequest[8] = -67;	arrayJointRequest[9] = -47; arrayJointRequest[10] = -19; 	arrayJointRequest[11] = -47;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #3 Progress: Step6 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step6 = true;
				}
			}
		else if(step7 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 17;	arrayJointRequest[1] = -17; arrayJointRequest[2] = -33; 	arrayJointRequest[3] = 33; 
			arrayJointRequest[4] = -41;	arrayJointRequest[5] = -41; arrayJointRequest[6] = -28; 	arrayJointRequest[7] = -28;
			arrayJointRequest[8] = -60;	arrayJointRequest[9] = -60; arrayJointRequest[10] = -30; 	arrayJointRequest[11] = -30;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #3-4 Progress: Step7 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step7 = true;
				}
			}
		else if(step8 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 30;	arrayJointRequest[1] =  0; arrayJointRequest[2] = -30; 		arrayJointRequest[3] = -0; 
			arrayJointRequest[4] = -34;	arrayJointRequest[5] = -45; arrayJointRequest[6] = -34; 	arrayJointRequest[7] = -26;
			arrayJointRequest[8] = -47;	arrayJointRequest[9] = -67; arrayJointRequest[10] = -47; 	arrayJointRequest[11] = -19;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #4 Progress: Step8 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step8 = true;
				}
			}
		else if(step9 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 33;	arrayJointRequest[1] = 17; arrayJointRequest[2] = -17; 	arrayJointRequest[3] = -33; 
			arrayJointRequest[4] = -28;	arrayJointRequest[5] = -41; arrayJointRequest[6] = -41; 	arrayJointRequest[7] = -28;
			arrayJointRequest[8] = -30;	arrayJointRequest[9] = -60; arrayJointRequest[10] = -60; 	arrayJointRequest[11] = -30;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw Toward leg #4-1 Progress: Step9 ******************************" << std::endl;
			if(compareRequestAndPossition(1.5))
				{
				step9 = true;
				}
			}
		else if(step10 == false)
			{
			//Step Joint Positions
			arrayJointRequest[0] = 0;	arrayJointRequest[1] = 0; 	arrayJointRequest[2] = 0; 		arrayJointRequest[3] = 0; 
			arrayJointRequest[4] = -30;	arrayJointRequest[5] = -30; arrayJointRequest[6] = -30; 	arrayJointRequest[7] = -30;
			arrayJointRequest[8] = -45;	arrayJointRequest[9] = -45; arrayJointRequest[10] = -45; 	arrayJointRequest[11] = -45;
			//apply step to controller
			setJointsPosition(0.5);
			std::cout << "*************************** Model CoMCCw  leg #1 Progress: Step1 ******************************" << std::endl;
			if(compareRequestAndPossition(1))
				{
				step10 = true;
				}
			}				
		else 
			{
			std::cout << "*************************** Model CoMCCw Complete  ******************************" << std::endl;
			busy = false; 
			comCCw = true;
			printJointPositions();
			}
	}//_____________________ end CoMCCw() ___________________________________________
	
*/	
