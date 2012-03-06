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

#include <boost/filesystem/fstream.hpp>
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
			 const Parser::UrdfJointLimitsPtrType& limits,
			 dynamicsJRLJapan::ObjectFactory& factory)
      {
	if (jointsMap.find (name) != jointsMap.end ())
	  throw std::runtime_error ("duplicated rotation joint");

	CjrlJoint* joint = factory.createJointRotation (position);
	joint->setName (name);
	if (limits)
	  {
	    joint->lowerBound (0, limits->lower);
	    joint->upperBound (0, limits->upper);
	    joint->lowerVelocityBound (0, -limits->velocity);
	    joint->upperVelocityBound (0, limits->velocity);
	  }
	jointsMap[name] = joint;
	return joint;
      }

      CjrlJoint*
      makeJointContinuous (Parser::MapJrlJoint& jointsMap,
			   const matrix4d& position,
			   const std::string& name,
			   dynamicsJRLJapan::ObjectFactory& factory)
      {
	//FIXME: handle properly continuous joints.
	Parser::UrdfJointLimitsPtrType emptyLimits;
	return makeJointRotation
	  (jointsMap, position, name, emptyLimits, factory);
      }

      CjrlJoint*
      makeJointTranslation (Parser::MapJrlJoint& jointsMap,
			    const matrix4d& position,
			    const std::string& name,
			    const Parser::UrdfJointLimitsPtrType& limits,
			    dynamicsJRLJapan::ObjectFactory& factory)
      {
	if (jointsMap.find (name) != jointsMap.end ())
	  throw std::runtime_error ("duplicated translation joint");

	CjrlJoint* joint = factory.createJointTranslation (position);
	joint->setName (name);
	if (limits)
	  {
	    joint->lowerBound (0, limits->lower);
	    joint->upperBound (0, limits->upper);
	    joint->lowerVelocityBound (0, -limits->velocity);
	    joint->upperVelocityBound (0, limits->velocity);
	  }
	jointsMap[name] = joint;
	return joint;
      }

      CjrlJoint*
      makeJointFreeFlyer (Parser::MapJrlJoint& jointsMap,
			  const matrix4d& position,
			  const std::string& name,
			  dynamicsJRLJapan::ObjectFactory& factory)
      {
	if (jointsMap.find (name) != jointsMap.end ())
	  throw std::runtime_error ("duplicated free flyer joint");

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
	if (jointsMap.find (name) != jointsMap.end ())
	  throw std::runtime_error ("duplicated anchor joint");

	CjrlJoint* joint = factory.createJointAnchor (position);
	joint->setName (name);
	jointsMap[name] = joint;
	return joint;
      }

      CjrlJoint*
      findJoint (const std::string& jointName,
		 const Parser::MapJrlJoint& map)
      {
	Parser::MapJrlJoint::const_iterator it = map.find (jointName);
	if (it == map.end ())
	  return 0;
	return it->second;
      }


      Parser::Parser ()
	: model_ (),
	  robot_ (),
	  rootJoint_ (),
	  jointsMap_ (),
	  factory_ (),
	  waistJointName_ (),
	  chestJointName_ (),
	  leftWristJointName_ (),
	  rightWristJointName_ (),
	  leftHandJointName_ (),
	  rightHandJointName_ (),
	  leftAnkleJointName_ (),
	  rightAnkleJointName_ (),
	  leftFootJointName_ (),
	  rightFootJointName_ (),
	  gazeJointName_ ()
      {}

      Parser::~Parser ()
      {}

      CjrlHumanoidDynamicRobot*
      Parser::parse (const std::string& filename,
		     const std::string& rootJointName)
      {
	boost::filesystem::fstream stream
	  (filename, boost::filesystem::fstream::in);
	std::string robotDescription;
	if (!stream.good ())
	  throw std::runtime_error ("failed to open robot description");
	while (stream)
	  {
	    std::string str;
	    std::getline (stream, str);
	    robotDescription += str;
	    robotDescription += "\n";
	  }
	return parseStream (robotDescription, rootJointName);
      }

      CjrlHumanoidDynamicRobot*
      Parser::parseStream (const std::string& robotDescription,
			   const std::string& rootJointName)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	model_.clear ();
	robot_ = factory_.createHumanoidDynamicRobot ();
	rootJoint_ = 0;
	jointsMap_.clear ();

	// Parse urdf model.
	if (!model_.initString (robotDescription))
	  throw std::runtime_error ("failed to open URDF file."
				    " Is the filename location correct?");

	findSpecialJoints ();

	// Look for actuated joints into the urdf model tree.
	parseActuatedJoints (rootJointName);
	if (!rootJoint_)
	  throw std::runtime_error ("failed to parse actuated joints");

	// Set model actuated joints.
	std::vector<CjrlJoint*> actJointsVect = actuatedJoints ();
	robot_->setActuatedJoints (actJointsVect);

	// Create the kinematic tree.
	// We iterate over the URDF root joints to connect them to the
	// root link that we added "manually" before. Then we iterate
	// in the whole tree using the connectJoints method.
	boost::shared_ptr<const ::urdf::Link> rootLink = model_.getRoot ();
	if (!rootLink)
	  throw std::runtime_error ("URDF model is missing a root link");

	typedef boost::shared_ptr<const ::urdf::Joint> JointPtr_t;
	BOOST_FOREACH (const JointPtr_t& joint, rootLink->child_joints)
	  {
	    if (!joint)
	      throw std::runtime_error ("null shared pointer in URDF model");
	    MapJrlJoint::const_iterator child = jointsMap_.find (joint->name);
	    if (child == jointsMap_.end () || !child->second)
	      throw std::runtime_error ("missing node in kinematics tree");
	    rootJoint_->addChildJoint (*child->second);
	    connectJoints(child->second);
	  }

	// Look for special joints and attach them to the model.
	robot_->waist (findJoint (waistJointName_, jointsMap_));
	robot_->chest (findJoint (chestJointName_, jointsMap_));
	robot_->leftWrist (findJoint (leftWristJointName_, jointsMap_));
	robot_->rightWrist (findJoint (rightWristJointName_, jointsMap_));
	robot_->leftAnkle (findJoint (leftAnkleJointName_, jointsMap_));
	robot_->rightAnkle (findJoint (rightAnkleJointName_, jointsMap_));
	robot_->gazeJoint (findJoint (rightFootJointName_, jointsMap_));

	// Add corresponding body (link) to each joint.
	addBodiesToJoints();

	fillHandsAndFeet ();

	//FIXME: disabled for now as jrl-dynamics anchor support is buggy.
	robot_->initialize();
	return robot_;
      }

      void
      Parser::findSpecialJoint (const std::string& repName, std::string& jointName)
      {
	UrdfLinkPtrType linkPtr = model_.links_[repName];
	if (linkPtr)
	  {
	    UrdfJointPtrType joint = linkPtr->parent_joint;
	    if (joint)
	      jointName = joint->name;
	  }
      }

      void
      Parser::findSpecialJoints ()
      {
	findSpecialJoint ("base_link", waistJointName_);
	findSpecialJoint ("torso", chestJointName_);
	findSpecialJoint ("l_wrist", leftWristJointName_);
	findSpecialJoint ("r_wrist", rightWristJointName_);
	findSpecialJoint ("l_gripper", leftHandJointName_);
	findSpecialJoint ("r_gripper", rightHandJointName_);
	findSpecialJoint ("l_ankle", leftAnkleJointName_);
	findSpecialJoint ("r_ankle", rightAnkleJointName_);
	findSpecialJoint ("l_sole", leftFootJointName_);
	findSpecialJoint ("r_sole", rightFootJointName_);
	findSpecialJoint ("gaze", gazeJointName_);
	//FIXME: we are missing toes in abstract-robot-dynamics for now.
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
	    position =
	      getPoseInReferenceFrame("base_footprint_joint", it->first);

	    switch(it->second->type)
	      {
	      case ::urdf::Joint::UNKNOWN:
		throw std::runtime_error
		  ("parsed joint has UNKNOWN type, this should not happen");
		break;
	      case ::urdf::Joint::REVOLUTE:
		makeJointRotation (jointsMap_, position, it->first,
				   it->second->limits,
				   factory_);
		break;
	      case ::urdf::Joint::CONTINUOUS:
		makeJointContinuous (jointsMap_, position, it->first, factory_);
		break;
	      case ::urdf::Joint::PRISMATIC:
		makeJointTranslation (jointsMap_, position, it->first,
				      it->second->limits,
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

	typedef std::map<std::string, boost::shared_ptr< ::urdf::Joint > >
	  jointMap_t;

        for(jointMap_t::const_iterator it = model_.joints_.begin ();
	    it != model_.joints_.end (); ++it)
	  {
	    if (!it->second)
	      throw std::runtime_error ("null joint shared pointer");
	    if (it->second->type == ::urdf::Joint::UNKNOWN
		|| it->second->type == ::urdf::Joint::FLOATING
		|| it->second->type == ::urdf::Joint::FIXED)
	      continue;
	    MapJrlJoint::const_iterator child = jointsMap_.find (it->first);
	    if (child == jointsMap_.end () || !child->second)
	      throw std::runtime_error ("failed to compute actuated joints");

	    // The joints already exists in the vector, do not add it twice.
	    if (std::find(jointsVect.begin (),
			  jointsVect.end (), child->second) != jointsVect.end ())
	      continue;
	    jointsVect.push_back (child->second);
	  }
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
        for(MapJrlJoint::const_iterator it = jointsMap_.begin();
	    it != jointsMap_.end(); ++it)
	  {
	    // Retrieve associated URDF joint.
	    UrdfJointConstPtrType joint = model_.getJoint (it->first);
	    if (!joint)
	      continue;

	    // Retrieve joint name.
	    std::string childLinkName = joint->child_link_name;

	    // Get child link.
	    UrdfLinkConstPtrType link = model_.getLink (childLinkName);
	    if (!link)
	      throw std::runtime_error ("inconsistent model");

	    // Retrieve inertial information.
	    boost::shared_ptr< ::urdf::Inertial> inertial =
	      link->inertial;

	    vector3d localCom (0., 0., 0.);
	    matrix3d inertiaMatrix;
	    inertiaMatrix.setIdentity();
	    double mass = 0.;
	    if (inertial)
	      {
		//FIXME: properly re-orient the frames.
		localCom[0] = inertial->origin.position.x;
		localCom[1] = inertial->origin.position.y;
		localCom[2] = inertial->origin.position.z;

		mass = inertial->mass;

		inertiaMatrix (0, 0) = inertial->ixx;
		inertiaMatrix (0, 1) = inertial->ixy;
		inertiaMatrix (0, 2) = inertial->ixz;

		inertiaMatrix (1, 0) = inertial->ixy;
		inertiaMatrix (1, 1) = inertial->iyy;
		inertiaMatrix (1, 2) = inertial->iyz;

		inertiaMatrix (2, 0) = inertial->ixz;
		inertiaMatrix (2, 1) = inertial->iyz;
		inertiaMatrix (2, 2) = inertial->izz;
	      }
	    else
	      std::cerr
		<< "WARNING: missing inertial information in model"
		<< std::endl;

	    // Create body and fill its fields..
	    BodyPtrType body = factory_.createBody ();
	    body->mass (mass);
	    body->localCenterOfMass (localCom);
	    body->inertiaMatrix (inertiaMatrix);
	    // Link body to joint.
	    it->second->setLinkedBody (*body);
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
	  {
	    boost::format fmt
	      ("failed to retrieve children joints of joint %s");
	    fmt % jointName;
	    throw std::runtime_error (fmt.str ());
	  }

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

      matrix4d
      Parser::getPoseInReferenceFrame(const std::string& referenceJointName,
				      const std::string& currentJointName)
      {
	if (referenceJointName == currentJointName)
	  return poseToMatrix
	    (model_.getJoint
	     (currentJointName)->parent_to_joint_origin_transform);

	// Retrieve corresponding joint in URDF tree.
	UrdfJointConstPtrType joint = model_.getJoint(currentJointName);
	if (!joint)
	  throw std::runtime_error
	    ("failed to retrieve parent while computing joint position");

	// Get transform from parent link to joint.
	::urdf::Pose jointToParentTransform =
	    joint->parent_to_joint_origin_transform;

	matrix4d transform = poseToMatrix (jointToParentTransform);

	// Get parent joint name.
	std::string parentLinkName = joint->parent_link_name;
	UrdfLinkConstPtrType parentLink = model_.getLink(parentLinkName);

	if (!parentLink)
	  return transform;
	UrdfJointConstPtrType parentJoint = parentLink->parent_joint;
	if (!parentJoint)
	  return transform;

	// Compute previous transformation with current one.
	transform =
	  getPoseInReferenceFrame (referenceJointName,
				   parentJoint->name) * transform;
	return transform;
      }

      matrix4d
      Parser::poseToMatrix(::urdf::Pose p)
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

      void
      Parser::fillHandsAndFeet ()
      {
	MapJrlJoint::const_iterator leftHand =
	  jointsMap_.find (leftHandJointName_);
	MapJrlJoint::const_iterator rightHand =
	  jointsMap_.find (rightHandJointName_);

	MapJrlJoint::const_iterator leftFoot =
	  jointsMap_.find (leftFootJointName_);
	MapJrlJoint::const_iterator rightFoot =
	  jointsMap_.find (rightFootJointName_);
	MapJrlJoint::const_iterator leftAnkle =
	  jointsMap_.find (leftAnkleJointName_);
	MapJrlJoint::const_iterator rightAnkle =
	  jointsMap_.find (rightAnkleJointName_);

	if (leftHand != jointsMap_.end ())
	  {
	    HandPtrType hand = factory_.createHand (leftHand->second);
	    robot_->leftHand (hand);
	  }

	if (rightHand != jointsMap_.end ())
	  {
	    HandPtrType hand = factory_.createHand (rightHand->second);
	    robot_->rightHand (hand);
	  }

	if (leftFoot != jointsMap_.end ())
	  {
	    FootPtrType foot = factory_.createFoot (leftFoot->second);

	    // Compute ankle position in local frame.
	    matrix4d anklePositionInLocalFrame;
	    anklePositionInLocalFrame.setIdentity ();
	    if (leftAnkle != jointsMap_.end ())
	      {
		matrix4d ankleInv;
		leftAnkle->second->initialPosition ().Inversion (ankleInv);
		anklePositionInLocalFrame =
		  leftFoot->second->initialPosition () * ankleInv;
	      }
	    vector3d v (anklePositionInLocalFrame (0, 3),
			anklePositionInLocalFrame (1, 3),
			anklePositionInLocalFrame (2, 3));
	    foot->setAnklePositionInLocalFrame (v);

	    robot_->leftFoot (foot);
	  }

	if (rightFoot != jointsMap_.end ())
	  {
	    FootPtrType foot = factory_.createFoot (rightFoot->second);

	    // Compute ankle position in local frame.
	    matrix4d anklePositionInLocalFrame;
	    anklePositionInLocalFrame.setIdentity ();
	    if (rightAnkle != jointsMap_.end ())
	      {
		matrix4d ankleInv;
		rightAnkle->second->initialPosition ().Inversion (ankleInv);
		anklePositionInLocalFrame =
		  rightFoot->second->initialPosition () * ankleInv;
	      }
	    vector3d v (anklePositionInLocalFrame (0, 3),
			anklePositionInLocalFrame (1, 3),
			anklePositionInLocalFrame (2, 3));
	    foot->setAnklePositionInLocalFrame (v);

	    robot_->rightFoot (foot);
	  }
      }

    } // end of namespace urdf.
  } // end of namespace dynamics.
} // end of namespace  jrl.
