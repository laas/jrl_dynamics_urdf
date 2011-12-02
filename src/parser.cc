
// Copyright (c) 2011 CNRS
// Authors: Florent Lamiraux


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

#include <map>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <jrl/dynamics/dynamicsfactory.hh>

#include "jrl/dynamics/urdf/parser.hh"

namespace jrl {
  namespace dynamics {
    namespace urdf {

	  typedef std::map< std::string,
	  boost::shared_ptr< ::urdf::Joint > > MapJointType;
	  typedef std::map< std::string,
	  boost::shared_ptr< ::urdf::Link > > MapLinkType;
	  typedef std::map< ::std::string, CjrlJoint* > MapJrlJoint;

	  Parser::Parser()
	  {

	  }

	  Parser::~Parser()
	  {

	  }

      CjrlHumanoidDynamicRobot*
      Parser::parse (const std::string& filename) 
      {
		dynamicsJRLJapan::ObjectFactory factory;
		CjrlHumanoidDynamicRobot* robot = factory.createHumanoidDynamicRobot ();

		model_.initFile(filename);
		
		// vars used in matrix creation
		MapJrlJoint jointsMap;
		CjrlJoint* tmpJoint;
		matrix4d position;
		position.setIdentity();
	
		// create root free floating joint	
		model_.getJoint("base_footprint_joint");
		// TODO set root joint position
		CjrlJoint* rootJoint = factory.createJointFreeflyer(position);
		jointsMap.insert(::std::pair<std::string, CjrlJoint*>("base_footprint_joint", rootJoint));

		// iterate through each "true cinematic" joint and create a corresponding CjrlJoint	
		for(MapJointType::const_iterator it = model_.joints_.begin(); it != model_.joints_.end(); ++it)
		{
		  // TODO set joint position

		  //create the right joint type, and get rid of the fixed joints and associated links		
		  switch(it->second->type)
		  {
			case ::urdf::Joint::REVOLUTE:
		      tmpJoint = factory.createJointRotation(position);
		      jointsMap.insert(::std::pair<std::string, CjrlJoint*>(it->second->name,tmpJoint));
			  break;
			case ::urdf::Joint::PRISMATIC:
		      tmpJoint = factory.createJointTranslation(position);
		  	  jointsMap.insert(::std::pair<std::string, CjrlJoint*>(it->second->name,tmpJoint));
			  break;
			default:
			  ;// do nothing
		  }
		}
		
		printf("Cinematic joints retrieved\n");		
		
		// Connect all joint to its child joint if any
		::std::string childLinkName;
		::std::vector< boost::shared_ptr< ::urdf::Joint> > childJointVect;
        for(MapJrlJoint::const_iterator it = jointsMap.begin(); it != jointsMap.end(); ++it)
        {
			childLinkName = model_.getJoint(it->first)->child_link_name;
        	childJointVect = model_.getLink(childLinkName)->child_joints;
			for(uint i=0; i<childJointVect.size(); ++i)
			{
				// Add child only if it is in the interesting joint list
				if( (tmpJoint = jointsMap[childJointVect[i]->name]) != NULL)
				{
					it->second->addChildJoint(*tmpJoint);
				}	
			}
		}

		printf("Children added to parent joints\n");

/*
		// Add corresponding body(link) to each joint
		::std::string childLinkName;
		::urdf::Link tmpLink;
		CjrlBody* tmpBody;
		Vector3d tmpLocalCom
		Matrix3d tmpInertiaMatrix;
        for(MapJrlJoint::const_iterator it = jointsMap.begin(); it != jointsMap.end(); ++it)
        {
			// get child link
			childLinkName = model_.getJoint(it->first)->child_link_name;
			tmpLink = model_.getLink(childLinkName);	
			// TODO get center of mass in local frame and inertia matrix in global frame
			tmpLocalCom=;
			tmpInertiaMatrix=;

			// create body
	        tmpBody = factory.createBody();
			// set center of mass and intertia matrix
			tmpBody->localCenterOfMass(tmpLocalCom);
			tmpBody->inertiaMatrix(tmpInertiaMatrix);
			//link body to joint
			it->second->setLinkedBody(*tmpBody)	
		}
*/ 
 
		/*
		matrix4d position;
		position.setIdentity ();
		// Create root joint
		CjrlJoint* rootJoint = factory.createJointFreeflyer(position);
		// Create root body and set inertia parameters
		CjrlBody* body = factory.createBody();
		vector3d localCom(0,0,0);
		matrix3d inertiaMatrix;
		inertiaMatrix.setIdentity();
		body->localCenterOfMass(localCom);
		body->inertiaMatrix(inertiaMatrix);
			// attach body to joint
		rootJoint->setLinkedBody(*body);
		*/

		return robot;
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
    } // urdf
  } // dynamics
} // jrl
