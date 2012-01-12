// Copyright 2011, 2012, Florent Lamiraux, Guido Manfredi, Thomas
// Moulard, JRL, CNRS/AIST.
//
// This file is part of jrl_dynamics_bridge.
// sot-motion-planner is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-motion-planner is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include "jrl/dynamics/urdf/parser.hh"

namespace jrl
{
  namespace dynamics
  {
    namespace urdf
    {

      CjrlJoint*
      makeJointRotation (Parser::MapJrlJoint& jointsMap,
			 const matrix4d& position,
			 const std::string& name,
			 const double& lower,
			 const double& upper,
			 dynamicsJRLJapan::ObjectFactory& factory)
      {
	CjrlJoint* joint = factory.createJointRotation (position);
	joint->setName (name);
	joint->lowerBound (0, lower);
	joint->upperBound (0, upper);
	jointsMap[name] = joint;
	return joint;
      }

      CjrlJoint*
      makeJointContinuous (Parser::MapJrlJoint& jointsMap,
			   const matrix4d& position,
			   const std::string& name,
			   dynamicsJRLJapan::ObjectFactory& factory)
      {
	return makeJointRotation
	  (jointsMap, position, name, -3.14, 3.14, factory);
      }

      CjrlJoint*
      makeJointTranslation (Parser::MapJrlJoint& jointsMap,
			    const matrix4d& position,
			    const std::string& name,
			    const double& lower,
			    const double& upper,
			    dynamicsJRLJapan::ObjectFactory& factory)
      {
	CjrlJoint* joint = factory.createJointTranslation (position);
	joint->setName (name);
	joint->lowerBound (0, lower);
	joint->upperBound (0, upper);
	jointsMap[name] = joint;
	return joint;
      }

      CjrlJoint*
      makeJointFreeFlyer (Parser::MapJrlJoint& jointsMap,
			  const matrix4d& position,
			  const std::string& name,
			  dynamicsJRLJapan::ObjectFactory& factory)
      {
	CjrlJoint* joint = factory.createJointFreeflyer (position);
	joint->setName (name);
	jointsMap[name] = joint;
	return joint;
      }

      CjrlJoint*
      makeJointAnchor (Parser::MapJrlJoint& jointsMap,
		       const matrix4d& position,
		       const std::string& name,
		       dynamicsJRLJapan::ObjectFactory& factory)
      {
	CjrlJoint* joint = factory.createJointAnchor (position);
	joint->setName (name);
	jointsMap[name] = joint;
	return joint;
      }


      Parser::Parser ()
	: model_ (),
	  robot_ (),
	  rootJoint_ (),
	  jointsMap_ (),
	  factory_ ()
      {}

      Parser::~Parser ()
      {}

      CjrlHumanoidDynamicRobot*
      Parser::parse (const std::string& filename,
		     const std::string& rootJointName)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	model_.clear ();
	robot_ = factory_.createHumanoidDynamicRobot ();
	rootJoint_ = 0;
	jointsMap_.clear ();

	// Parse urdf model.
	if (!model_.initFile (filename))
	  throw std::runtime_error ("failed to open URDF file."
				    " Is the filename location correct?");

	// Look for actuated joints into the urdf model tree.
	parseActuatedJoints (rootJointName);
	if (!rootJoint_)
	  throw std::runtime_error ("failed to parse actuated joints");

	// Set model actuated joints.
	std::vector<CjrlJoint*> actJointsVect = actuatedJoints ();
	robot_->setActuatedJoints (actJointsVect);

	// Create the kinematic tree.
	connectJoints(rootJoint_);

	// Notifying special joints
	robot_->waist(jointsMap_["base_footprint_joint"]);
	robot_->chest(jointsMap_["torso_lift_joint"]);
	robot_->leftWrist(jointsMap_["l_gripper_joint"]);
	robot_->rightWrist(jointsMap_["r_gripper_joint"]);

	// Add corresponding body(link) to each joint
	addBodiesToJoints();

	robot_->initialize();
	return robot_;
      }

      void
      Parser::parseActuatedJoints (const std::string rootJointName)
      {
	// Create free floating joint.
	// FIXME: position set to identity for now.
	matrix4d position;
	position.setIdentity ();
	rootJoint_ = makeJointFreeFlyer (jointsMap_, position, rootJointName,
					 factory_);
	if (!rootJoint_)
	  throw std::runtime_error
	    ("failed to create root joint (free floating)");
	robot_->rootJoint(*rootJoint_);

	// Iterate through each "true cinematic" joint and create a
	// corresponding CjrlJoint.
	for(MapJointType::const_iterator it = model_.joints_.begin();
	    it != model_.joints_.end(); ++it)
	  {
	    // FIXME: compute joint position
	    // position =
	    // getPoseInReferenceFrame("base_footprint_joint",
	    // it->first);

	    switch(it->second->type)
	      {
	      case ::urdf::Joint::UNKNOWN:
		throw std::runtime_error
		  ("parsed joint has UNKNOWN type, this should not happen");
		break;
	      case ::urdf::Joint::REVOLUTE:
		makeJointRotation (jointsMap_, position, it->first,
				   it->second->limits->lower,
				   it->second->limits->upper,
				   factory_);
		break;
	      case ::urdf::Joint::CONTINUOUS:
		makeJointContinuous (jointsMap_, position, it->first, factory_);
		break;
	      case ::urdf::Joint::PRISMATIC:
		makeJointTranslation (jointsMap_, position, it->first,
				      it->second->limits->lower,
				      it->second->limits->upper,
				      factory_);
		break;
	      case ::urdf::Joint::FLOATING:
		makeJointFreeFlyer (jointsMap_, position, it->first, factory_);
		break;
	      case ::urdf::Joint::PLANAR:
		throw std::runtime_error ("PLANAR joints are not supported");
		break;
	      case ::urdf::Joint::FIXED:
		makeJointAnchor (jointsMap_, position, it->first, factory_);
		break;
	      default:
		boost::format fmt
		  ("unknown joint type %1%: should never happen");
		fmt % (int)it->second->type;
		throw std::runtime_error (fmt.str ());
	      }
	  }
      }

      std::vector<CjrlJoint*> Parser::actuatedJoints ()
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

      void
      Parser::connectJoints (CjrlJoint* rootJoint)
      {
	BOOST_FOREACH (const std::string& childName,
		       getChildrenJoint (rootJoint->getName ()))
	  {
	    MapJrlJoint::const_iterator child = jointsMap_.find (childName);
	    if (child == jointsMap_.end () && !!child->second)
	      throw std::runtime_error ("failed to connect joints");
	    rootJoint->addChildJoint(*child->second);
	    connectJoints (child->second);
	  }
      }

      void
      Parser::addBodiesToJoints ()
      {
	std::string childLinkName;
	boost::shared_ptr<const ::urdf::Link> tmpLink;
	CjrlBody* tmpBody;
	double tmpMass;
	vector3d tmpLocalCom;
	matrix3d tmpInertiaMatrix;
        for(MapJrlJoint::const_iterator it = jointsMap_.begin();
	    it != jointsMap_.end(); ++it)
	  {
	    // get child link
	    childLinkName = model_.getJoint(it->first)->child_link_name;
	    tmpLink = model_.getLink(childLinkName);
	    // TODO get center of mass in local frame, inertia matrix
	    // in global frame and body mass
	    tmpLocalCom = vector3d(0,0,0);
	    tmpInertiaMatrix.setIdentity();
	    tmpMass = 1;

	    // create body
	    tmpBody = factory_.createBody();
	    tmpBody->mass(tmpMass);
	    // set center of mass and intertia matrix
	    tmpBody->localCenterOfMass(tmpLocalCom);
	    tmpBody->inertiaMatrix(tmpInertiaMatrix);
	    //link body to joint
	    it->second->setLinkedBody(*tmpBody);
	  }
      }

      std::vector<std::string>
      Parser::getChildrenJoint (const std::string& jointName)
      {
	std::vector<std::string> result;
	getChildrenJoint (jointName, result);
	return result;
      }

      void
      Parser::getChildrenJoint (const std::string& jointName,
				std::vector<std::string>& result)
      {
	typedef boost::shared_ptr< ::urdf::Joint> jointPtr_t;

	boost::shared_ptr<const ::urdf::Joint> joint =
	  model_.getJoint(jointName);

	if (!joint)
	  throw std::runtime_error ("failed to retrieve children joint");

	boost::shared_ptr<const ::urdf::Link> childLink =
	  model_.getLink (joint->child_link_name);

	if (!childLink)
	  throw std::runtime_error ("failed to retrieve children link");

       	const std::vector<jointPtr_t>& jointChildren =
	  childLink->child_joints;

	BOOST_FOREACH (const jointPtr_t& joint, jointChildren)
	  {
	    if (jointsMap_.count(joint->name) > 0)
	      result.push_back (joint->name);
	    else
	      getChildrenJoint (joint->name, result);
	  }
      }

      matrix4d Parser::getPoseInReferenceFrame(std::string referenceJointName,
					       std::string currentJointName)
      {
	if(referenceJointName.compare(currentJointName) == 0)
	  return poseToMatrix
	    (model_.getJoint
	     (currentJointName)->parent_to_joint_origin_transform);

	// get transform to parent link
	::urdf::Pose jointToParentTransform =
	    model_.getJoint(currentJointName)->parent_to_joint_origin_transform;
	matrix4d transform = poseToMatrix(jointToParentTransform);
	// move to next parent joint
	std::string parentLinkName =
	  model_.getJoint(currentJointName)->parent_link_name;
	std::string parentJointName =
	  model_.getLink(parentLinkName)->parent_joint->name;
	transform *= getPoseInReferenceFrame(referenceJointName,
					     parentJointName);

	return transform;
      }

      matrix4d Parser::poseToMatrix(::urdf::Pose p)
      {
	matrix4d t;

	// Fill rotation part.
	btQuaternion q (p.rotation.x, p.rotation.y,
			p.rotation.z, p.rotation.w);
	btMatrix3x3 rotationMatrix (q);
	for (unsigned i = 0; i < 3; ++i)
	  for (unsigned j = 0; j < 3; ++j)
	    t (i, j) = rotationMatrix[i][j];

	// Fill translation part.
	t (0, 3) = p.position.x;
	t (1, 3) = p.position.y;
	t (2, 3) = p.position.z;
	t (3, 3) = 1.;

	t(3, 0) = t(3, 1) = t(3, 2) = 0.;

	return t;
      }
    } // end of namespace urdf.
  } // end of namespace dynamics.
} // end of namespace  jrl.
