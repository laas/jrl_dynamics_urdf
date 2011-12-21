
// Copyright (c) 2011 CNRS
// Authors: Florent Lamiraux, Guido Manfredi


// This file is part of jrl_dynamics_urdf
// jrl_dynamics_urdf is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.

// jrl_dynamics_urdf is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// jrl_dynamics_urdf  If not, see
// <http://www.gnu.org/licenses/>.

# include "jrl/dynamics/urdf/parser.hh"
 
namespace jrl {
  namespace dynamics {
    namespace urdf {

	  typedef std::map< std::string,
	  boost::shared_ptr< ::urdf::Joint > > MapJointType;

	  Parser::Parser()
	  {

	  }

	  Parser::~Parser()
	  {

	  }

      CjrlHumanoidDynamicRobot*
      Parser::parse (const std::string& filename, const std::string& rootJointName) 
      {
		dynamicsJRLJapan::ObjectFactory factory;
		CjrlHumanoidDynamicRobot* robot = factory.createHumanoidDynamicRobot ();

		model_.initFile(filename);			

		// create each joint an return rootJoint
		CjrlJoint* rootJoint = parseActuatedJoints(rootJointName);
		// set actuated joints
		std::vector<CjrlJoint*> actJointsVect = actuatedJoints();
		robot->setActuatedJoints (actJointsVect);
		// create cinematic tree
	    connectJoints(rootJoint);
		
		// Notifying special joints
		robot->waist(jointsMap_["base_footprint_joint"]);
		robot->chest(jointsMap_["torso_lift_joint"]);
		robot->leftWrist(jointsMap_["l_gripper_joint"]);
		robot->rightWrist(jointsMap_["r_gripper_joint"]);

		// Add corresponding body(link) to each joint
		addBodiesToJoints();

		// add the cinematic tree to the robot
		robot->rootJoint(*rootJoint);
		printf("Children added to parent joints\n");
 
		robot->initialize();
		
		std::cout << "joints parsed: " << jointsMap_.size () << std::endl;
		std::cout << "joints in robot: " << robot->jointVector().size () << std::endl;
		std::cout << "actuated joints in robot: " << robot->getActuatedJoints().size () << std::endl;

		return robot;
      }
/*
	  MapJrlJoint* Parser::retrieveActuatedJoints()
	  {
		std::cout << "Retrieving revolute, prismatic, and continuous joints" << std::endl;		

		
		return jointsMap_;
	  }
*/

	  CjrlJoint* Parser::parseActuatedJoints (const std::string rootJointName)
	  {
		dynamicsJRLJapan::ObjectFactory factory;

		// retrieve actuated joints	
		CjrlJoint* tmpJoint;
		matrix4d position;
		position.setIdentity();
	
		// create root free floating joint	
		model_.getJoint(rootJointName);
		// TODO set root joint position
		CjrlJoint* rootJoint = factory.createJointFreeflyer(position);
		rootJoint->setName(rootJointName);
		jointsMap_.insert(::std::pair<std::string, CjrlJoint*>(rootJointName, rootJoint));

		// iterate through each "true cinematic" joint and create a corresponding CjrlJoint	
		for(MapJointType::const_iterator it = model_.joints_.begin(); it != model_.joints_.end(); ++it)
		{
		  // TODO set joint position
		  //position = getPoseInReferenceFrame("base_footprint_joint", it->first);	
		  //create the right joint type, and get rid of the fixed joints and associated links		
		  
		  switch(it->second->type)
		  {
			case ::urdf::Joint::REVOLUTE:
		      tmpJoint = factory.createJointRotation(position);
			  tmpJoint->setName(it->first);
			  tmpJoint->lowerBound(it->second->limits->lower);
			  tmpJoint->upperBound(it->second->limits->upper);
		      jointsMap_.insert(::std::pair<std::string, CjrlJoint*>(it->second->name,tmpJoint));
			  break;
			case ::urdf::Joint::PRISMATIC:
		      tmpJoint = factory.createJointTranslation(position);
			  tmpJoint->setName(it->first);
			  tmpJoint->lowerBound(it->second->limits->lower);
			  tmpJoint->upperBound(it->second->limits->upper);
		  	  jointsMap_.insert(::std::pair<std::string, CjrlJoint*>(it->second->name,tmpJoint));
			case ::urdf::Joint::CONTINUOUS:
		      tmpJoint = factory.createJointRotation(position);
			  tmpJoint->setName(it->first);
			  // continuous joints doesn't have limits, so we use fake ones
			  tmpJoint->lowerBound(-3.14);
			  tmpJoint->upperBound(3.14);
		  	  jointsMap_.insert(::std::pair<std::string, CjrlJoint*>(it->second->name,tmpJoint));
			  break;
			default:
			  ;// do nothing
		  }
		}

		return rootJoint;
	  }

	  std::vector<CjrlJoint*> Parser::actuatedJoints ( void )
	  {
		std::vector<CjrlJoint*> jointsVect;
		jointsVect.push_back (jointsMap_["base_footprint_joint"]);
		jointsVect.push_back (jointsMap_["torso_lift_joint"]);
		jointsVect.push_back (jointsMap_["r_shoulder_pan_joint"]);
		jointsVect.push_back (jointsMap_["r_shoulder_lift_joint"]);
		jointsVect.push_back (jointsMap_["r_upper_arm_roll_joint"]);
		jointsVect.push_back (jointsMap_["r_elbow_flex_joint"]);
		jointsVect.push_back (jointsMap_["r_forearm_roll_joint"]);
		jointsVect.push_back (jointsMap_["r_wrist_flex_joint"]);
		jointsVect.push_back (jointsMap_["r_wrist_roll_joint"]);
		jointsVect.push_back (jointsMap_["l_shoulder_pan_joint"]);
		jointsVect.push_back (jointsMap_["l_shoulder_lift_joint"]);
		jointsVect.push_back (jointsMap_["l_upper_arm_roll_joint"]);
		jointsVect.push_back (jointsMap_["l_elbow_flex_joint"]);
		jointsVect.push_back (jointsMap_["l_forearm_roll_joint"]);
		jointsVect.push_back (jointsMap_["l_wrist_flex_joint"]);
		jointsVect.push_back (jointsMap_["l_wrist_roll_joint"]);

		return jointsVect;
	  }

	  void Parser::connectJoints(CjrlJoint* rootJoint)
	  {
		// Connect root joint first with all children
		::std::vector<std::string> childJointNameVect;
		childJointNameVect = getChildrenJoint(rootJoint->getName());
		for(uint i=0; i<childJointNameVect.size(); ++i)
		{
			rootJoint->addChildJoint(*jointsMap_[childJointNameVect[i]]);
		}

		// Connect all other joints to its child joint if any
        for(MapJrlJoint::const_iterator it = jointsMap_.begin(); it != jointsMap_.end(); ++it)
        {
			childJointNameVect = getChildrenJoint(it->first);
			for(uint i=0; i<childJointNameVect.size(); ++i)
			{
				it->second->addChildJoint(*jointsMap_[childJointNameVect[i]]);
			}
		}
	  }

	  void Parser::addBodiesToJoints(void)
	  {
		dynamicsJRLJapan::ObjectFactory factory;

		::std::string childLinkName;
		boost::shared_ptr<const ::urdf::Link> tmpLink;
		CjrlBody* tmpBody;
		double tmpMass;
		vector3d tmpLocalCom;
		matrix3d tmpInertiaMatrix;
        for(MapJrlJoint::const_iterator it = jointsMap_.begin(); it != jointsMap_.end(); ++it)
        {
			// get child link
			childLinkName = model_.getJoint(it->first)->child_link_name;
			tmpLink = model_.getLink(childLinkName);	
			// TODO get center of mass in local frame, inertia matrix in global frame and body mass
			tmpLocalCom = vector3d(0,0,0);
			tmpInertiaMatrix.setIdentity();
			tmpMass = 1;

			// create body
	        tmpBody = factory.createBody();
			tmpBody->mass(tmpMass);
			// set center of mass and intertia matrix
			tmpBody->localCenterOfMass(tmpLocalCom);
			tmpBody->inertiaMatrix(tmpInertiaMatrix);
			//link body to joint
			it->second->setLinkedBody(*tmpBody);	
		}
	  }

	  // returns, in a vector, the children of a joint, or a subchildren if
	  // no children si present in the actuated joints map.
	  std::vector<std::string> Parser::getChildrenJoint (std::string jointName)
	  {
		::std::string childLinkName;
		::std::vector< boost::shared_ptr< ::urdf::Joint> > childJointVect;

		//vector containing children
		std::vector<std::string> v;
		std::vector<std::string> tmpV;

		// first search among children	
		childLinkName = model_.getJoint(jointName)->child_link_name;
       	childJointVect = model_.getLink(childLinkName)->child_joints;
		for(uint i=0; i<childJointVect.size(); ++i)
		{
			
			// Add child only if it is in the interesting joint list
			if( jointsMap_.count(childJointVect[i]->name) != 0)
			{
				v.push_back(childJointVect[i]->name);
			}
			// if a child is not in the actuated joint list
			// then search for subchildren
			else
			{	
				tmpV = getChildrenJoint(childJointVect[i]->name);
				for ( uint j=0 ; j < tmpV.size() ; ++j )
				{
					v.push_back(tmpV[j]);
				}
			}

		}
	
		return v;	
	  }

	  matrix4d Parser::getPoseInReferenceFrame(::std::string referenceJointName, ::std::string currentJointName)
	  {
		if(referenceJointName.compare(currentJointName) == 0)
			return poseToMatrix(model_.getJoint(currentJointName)->parent_to_joint_origin_transform);

		// get transform to parent link
		::urdf::Pose jointToParentTransform = model_.getJoint(currentJointName)->parent_to_joint_origin_transform;
		matrix4d transform = poseToMatrix(jointToParentTransform);
		// move to next parent joint
		::std::string parentLinkName = model_.getJoint(currentJointName)->parent_link_name;
		::std::string parentJointName = model_.getLink(parentLinkName)->parent_joint->name;
		transform *= getPoseInReferenceFrame(referenceJointName, parentJointName);

		return transform;
	  }

	  matrix4d Parser::poseToMatrix(::urdf::Pose p)
	  {
		matrix4d t;
		double x= p.rotation.x;
		double y= p.rotation.y;
		double z= p.rotation.z;
		double w= p.rotation.w;
		double vx= p.position.x;
		double vy= p.position.y;
		double vz= p.position.z;

		// formula for quaternion to rotation matrix from wikipedia
		t(0, 0)=x*x+y*y-z*z-w*w;  t(0, 1)=2*y*z-2*x*w;      t(0, 2)=2*y*w+2*x*z;      t(0, 3)=vx; 
		t(1, 0)=2*y*z+2*x*w;      t(1, 1)=x*x-y*y+z*z-w*w; 	t(1, 2)=2*z*w-2*x*y; 	  t(1, 3)=vy; 
		t(2, 0)=2*y*w-2*x*z;	  t(2, 1)=2*z*w-2*x*y; 		t(2, 2)=x*x-y*y-z*z+w*w;  t(2, 3)=vz;
		t(3, 0)=0; 	 	          t(3, 1)=0; 		        t(3, 2)=0; 		          t(3, 3)=1;

		return t; 
	  }
/*
	  	matrix4d Parser::positionFromAxisAndCenter(const Vector3d& inAxis,
										  const Point3d& inCenter)
		{
		  Vector3d v1(inAxis);
		  Vector3d v2(0, 0, 0);
		  Vector3d v3(0, 0, 0);

		  //TODO v1.normalize();

		  unsigned int smallestComponent=0;
		  double valueSmallestComponent = fabs(v1[0]);

		  if (fabs(v1[1]) < fabs(v1[smallestComponent])) {
			smallestComponent = 1;
			valueSmallestComponent = fabs(v1[1]);
		  }

		  if (fabs(v1[2]) < fabs(v1[smallestComponent])) {
			smallestComponent = 2;
			valueSmallestComponent = fabs(v1[2]);
		  }

		  v2[smallestComponent] = 1;

		  v3 = v1*v2;
		  v2 = v3*v1;

		  // (v1, v2, v3) form an orthonormal basis

		  matrix4d outPositionMatrix;

		  for (unsigned int iRow=0; iRow < 3; iRow++) {
			outPositionMatrix(iRow, 0) = v1[iRow];
			outPositionMatrix(iRow, 1) = v2[iRow];
			outPositionMatrix(iRow, 2) = v3[iRow];
			outPositionMatrix(iRow, 3) = inCenter[iRow];
		  }
		  return outPositionMatrix;
		}
*/
    } // urdf
  } // dynamics
} // jrl
