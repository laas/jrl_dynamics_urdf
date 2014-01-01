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

#include <resource_retriever/retriever.h>
#include <urdf_parser/urdf_parser.h>

#include "jrl/dynamics/urdf/parser.hh"


matrix3d
matrix4dTo3d (const matrix4d & m)
{
  return matrix3d (m(0,0), m(0,1), m(0,2),
                   m(1,0), m(1,1), m(1,2),
                   m(2,0), m(2,1), m(2,2));
}


namespace jrl
{
  namespace dynamics
  {
    namespace urdf
    {
      namespace
      {
	static const char* rootJointName = "base_joint";

	/// \brief Convert joint orientation to standard
	/// jrl-dynamics accepted orientation.
	///
	/// abstract-robot-dynamics do not contain any information
	/// about around which axis a rotation joint rotates.
	/// On the opposite, it makes the assumption it is around the X
	/// axis. We have to make sure this is the case here.
	///
	/// We use Gram-Schmidt process to compute the rotation matrix.
	///
	/// [1] http://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
	matrix4d
	normalizeFrameOrientation (Parser::UrdfJointConstPtrType urdfJoint)
	{
	  if (!urdfJoint)
	    throw std::runtime_error
	      ("invalid joint in normalizeFrameOrientation");
	  matrix4d result;
	  result.setIdentity ();

	  vector3d x (urdfJoint->axis.x,
		      urdfJoint->axis.y,
		      urdfJoint->axis.z);
	  x.normalize ();

	  vector3d y (0., 0., 0.);
	  vector3d z (0., 0., 0.);

	  unsigned smallestComponent = 0;
	  for (unsigned i = 0; i < 3; ++i)
	    if (std::fabs(x[i]) < std::fabs(x[smallestComponent]))
	      smallestComponent = i;

	  y[smallestComponent] = 1.;
	  z = x ^ y;
	  y = z ^ x;
	  // (x, y, z) is an orthonormal basis.

	  for (unsigned i = 0; i < 3; ++i)
	    {
	      result (i, 0) = x[i];
	      result (i, 1) = y[i];
	      result (i, 2) = z[i];
	    }

	  return result;
	}
      } // end of anonymous namespace.


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
            gazeJointName_ (),
            JointsNamesByRank_ ()
      {
	initREPNames();
      }

      Parser::~Parser ()
      {}

      void Parser::initREPNames()
      {
	linkREPNames_["BODY"]       = "BODY";
	linkREPNames_["torso"]      = "torso";
	linkREPNames_["l_wrist"]    = "l_wrist";
	linkREPNames_["r_wrist"]    = "r_wrist";
	linkREPNames_["l_gripper"]  = "l_gripper";
	linkREPNames_["r_gripper"]  = "r_gripper";
	linkREPNames_["l_ankle"]    = "l_ankle";
	linkREPNames_["r_ankle"]    = "r_ankle";
	linkREPNames_["l_sole"]     = "l_sole";
	linkREPNames_["r_sole"]     = "r_sole";
	linkREPNames_["gaze"]       = "gaze";
      }

      void
      Parser::specifyREPName(const std::string &link,
			     const std::string &repName)
      {
	linkREPNames_[link] = repName;
      }

      boost::shared_ptr< ::urdf::ModelInterface>
      Parser::urdfModel () const
      {
	return model_;
      }

      Parser::MapJrlJoint
      Parser::mapJrlJoint () const
      {
	return jointsMap_;
      }

      CjrlHumanoidDynamicRobot*
      Parser::parse (const std::string& filename)
      {
	resource_retriever::Retriever resourceRetriever;

	resource_retriever::MemoryResource resource =
	  resourceRetriever.get(filename);
	std::string robotDescription;
	robotDescription.resize(resource.size);
	unsigned i = 0;
	for (; i < resource.size; ++i)
	  robotDescription[i] = resource.data.get()[i];
	return parseStream (robotDescription);
      }

      CjrlHumanoidDynamicRobot*
      Parser::parseStream (const std::string& robotDescription)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	model_.reset ();
	robot_ = factory_.createHumanoidDynamicRobot ();
	rootJoint_ = 0;
	jointsMap_.clear ();

	// Parse urdf model.
	model_ = ::urdf::parseURDF (robotDescription);
	if (!model_)
	  throw std::runtime_error ("failed to open URDF file."
				    " Is the filename location correct?");

	findSpecialJoints ();

	// Look for actuated joints into the urdf model tree.
	parseJoints (rootJointName);
	if (!rootJoint_)
	  throw std::runtime_error ("failed to parse actuated joints");

	// Create the kinematic tree.
	// We iterate over the URDF root joints to connect them to the
	// root link that we added "manually" before. Then we iterate
	// in the whole tree using the connectJoints method.
	boost::shared_ptr<const ::urdf::Link> rootLink = model_->getRoot ();
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
	robot_->gazeJoint (findJoint (gazeJointName_, jointsMap_));

	// Add corresponding body (link) to each joint.
	addBodiesToJoints();

	robot_->initialize();

	// Set model actuated joints.
	std::vector<CjrlJoint*> actJointsVect = actuatedJoints ();
	robot_->setActuatedJoints (actJointsVect);

	// re-orient the frames for com and inertia
	for(MapJrlJoint::iterator it = jointsMap_.begin();
	    it != jointsMap_.end(); ++it)
	  {
	    if(it->second->linkedBody() != 0x0)
	      {
		matrix3d rotation = (matrix4dTo3d(it->second->initialPosition ()));
		vector3d com = it->second->linkedBody()->localCenterOfMass();
		matrix3d inertia = it->second->linkedBody()->inertiaMatrix();
		it->second->linkedBody()->localCenterOfMass(rotation.Transpose() * com);
		it->second->linkedBody()->inertiaMatrix(rotation.Transpose() * inertia * rotation);
	      }
	  }

	// Here we need to use joints initial positions. Make sure to
	// call this *after* initializating the structure.
	fillHandsAndFeet ();

    // Load a list of joints ordered by rank
    std::vector<CjrlJoint*> tmp_jv = robot_->jointVector();
    for (int i=0;i<tmp_jv.size();i++)
        if (std::find(actJointsVect.begin(), actJointsVect.end(),tmp_jv[i])!=actJointsVect.end())
            JointsNamesByRank_[tmp_jv[i]->rankInConfiguration()-6] = tmp_jv[i]->getName();

	return robot_;
      }

      void
      Parser::findSpecialJoint (const std::string& repName,
				std::string& jointName)
      {
	UrdfLinkPtrType linkPtr = model_->links_[repName];
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
	findSpecialJoint (linkREPNames_["BODY"      ],  waistJointName_     );
	findSpecialJoint (linkREPNames_["torso"     ],  chestJointName_     );
	findSpecialJoint (linkREPNames_["l_wrist"   ],  leftWristJointName_ );
	findSpecialJoint (linkREPNames_["r_wrist"   ],  rightWristJointName_);
	findSpecialJoint (linkREPNames_["l_gripper" ],  leftHandJointName_  );
	findSpecialJoint (linkREPNames_["r_gripper" ],  rightHandJointName_ );
	findSpecialJoint (linkREPNames_["l_ankle"   ],  leftAnkleJointName_ );
	findSpecialJoint (linkREPNames_["r_ankle"   ],  rightAnkleJointName_);
	findSpecialJoint (linkREPNames_["l_sole"    ],  leftFootJointName_  );
	findSpecialJoint (linkREPNames_["r_sole"    ],  rightFootJointName_ );
	findSpecialJoint (linkREPNames_["gaze"      ],  gazeJointName_      );
      }

      void
      Parser::parseJoints (const std::string rootJointName)
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
	for(MapJointType::const_iterator it = model_->joints_.begin();
	    it != model_->joints_.end(); ++it)
	  {
	    position =
	      getPoseInReferenceFrame("base_footprint_joint", it->first);

	    // Normalize orientation if this is a rotation joint.
	    UrdfJointConstPtrType joint = model_->getJoint (it->first);
	    if (joint->type == ::urdf::Joint::REVOLUTE
		|| joint->type == ::urdf::Joint::CONTINUOUS
		|| joint->type == ::urdf::Joint::PRISMATIC)
	      position = position * normalizeFrameOrientation (joint);

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

        for(jointMap_t::const_iterator it = model_->joints_.begin ();
	    it != model_->joints_.end (); ++it)
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
	    if (std::find
		(jointsVect.begin (),
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
	    UrdfLinkConstPtrType link;
	    // Retrieve associated URDF joint.
	    UrdfJointConstPtrType joint = model_->getJoint (it->first);
	    if (!joint)
	      {
		//Dealing with the Free-Flyer joint, not part of urdf model
		link = model_->getRoot ();
	      }
	    else
	      {
		// Retrieve joint name.
		std::string childLinkName = joint->child_link_name;

		// Get child link.
		link = model_->getLink (childLinkName);
	      }
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
		<< "WARNING: missing inertial information in model "
		<< ((joint != 0X0)? joint->child_link_name : "root link")
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
	  model_->getJoint(jointName);

	if (!joint)
	  {
	    boost::format fmt
	      ("failed to retrieve children joints of joint %s");
	    fmt % jointName;
	    throw std::runtime_error (fmt.str ());
	  }

	boost::shared_ptr<const ::urdf::Link> childLink =
	  model_->getLink (joint->child_link_name);

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
	    (model_->getJoint
	     (currentJointName)->parent_to_joint_origin_transform);

	// Retrieve corresponding joint in URDF tree.
	UrdfJointConstPtrType joint = model_->getJoint(currentJointName);
	if (!joint)
	  throw std::runtime_error
	    ("failed to retrieve parent while computing joint position");

	// Get transform from parent link to joint.
	::urdf::Pose jointToParentTransform =
	    joint->parent_to_joint_origin_transform;

	matrix4d transform = poseToMatrix (jointToParentTransform);

	// Get parent joint name.
	std::string parentLinkName = joint->parent_link_name;
	UrdfLinkConstPtrType parentLink = model_->getLink(parentLinkName);

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

	// Fill rotation part: convert quaternion to rotation matrix.
	double q0 = p.rotation.w;
	double q1 = p.rotation.x;
	double q2 = p.rotation.y;
	double q3 = p.rotation.z;
	t(0,0) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	t(0,1) = 2*q1*q2 - 2*q0*q3;
	t(0,2) = 2*q1*q3 + 2*q0*q2;
	t(1,0) = 2*q1*q2 + 2*q0*q3;
	t(1,1) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	t(1,2) = 2*q2*q3 - 2*q0*q1;
	t(2,0) = 2*q1*q3 - 2*q0*q2;
	t(2,1) = 2*q2*q3 + 2*q0*q1;
	t(2,2) = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	// Fill translation part.
	t (0, 3) = p.position.x;
	t (1, 3) = p.position.y;
	t (2, 3) = p.position.z;
	t (3, 3) = 1.;

	t(3, 0) = t(3, 1) = t(3, 2) = 0.;

	return t;
      }

      vector3d
      Parser::computeAnklePositionInLocalFrame
      (MapJrlJoint::const_iterator& foot,
       MapJrlJoint::const_iterator& ankle)
	const
      {
	matrix4d world_M_foot =
	  foot->second->initialPosition ();
	matrix4d world_M_ankle =
	  ankle->second->initialPosition ();
	matrix4d foot_M_world;
	world_M_foot.Inversion (foot_M_world);

	matrix4d foot_M_ankle = foot_M_world * world_M_ankle;
	return vector3d (foot_M_ankle (0, 3),
			 foot_M_ankle (1, 3),
			 foot_M_ankle (2, 3));
      }

      namespace
      {
	vector3d
	vector4dTo3d (vector4d v)
	{
	  return vector3d (v[0], v[1], v[2]);
	}
      } // end of anonymous namespace.

      void
      Parser::computeHandsInformation
      (MapJrlJoint::const_iterator& hand,
       MapJrlJoint::const_iterator& wrist,
       vector3d& center,
       vector3d& thumbAxis,
       vector3d& foreFingerAxis,
       vector3d& palmNormal) const
      {
	matrix4d world_M_hand =
	  hand->second->initialPosition ();
	matrix4d world_M_wrist =
	  wrist->second->initialPosition ();
	matrix4d wrist_M_world;
	world_M_wrist.Inversion (wrist_M_world);

	matrix4d wrist_M_hand = wrist_M_world * world_M_hand;

	for (unsigned i = 0; i < 3; ++i)
	  center[i] = wrist_M_hand (i, 3);

	thumbAxis = vector4dTo3d
	  (wrist_M_hand * vector4d (0., 0., 1., 0.));
	foreFingerAxis = vector4dTo3d
	  (wrist_M_hand * vector4d (1., 0., 0., 0.));
	palmNormal = vector4dTo3d
	  (wrist_M_hand * vector4d (0., 1., 0., 0.));
      }

      void
      Parser::fillHandsAndFeet ()
      {
	MapJrlJoint::const_iterator leftHand =
	  jointsMap_.find (leftHandJointName_);
	MapJrlJoint::const_iterator rightHand =
	  jointsMap_.find (rightHandJointName_);
	MapJrlJoint::const_iterator leftWrist =
	  jointsMap_.find (leftWristJointName_);
	MapJrlJoint::const_iterator rightWrist =
	  jointsMap_.find (rightWristJointName_);

	MapJrlJoint::const_iterator leftFoot =
	  jointsMap_.find (leftFootJointName_);
	MapJrlJoint::const_iterator rightFoot =
	  jointsMap_.find (rightFootJointName_);
	MapJrlJoint::const_iterator leftAnkle =
	  jointsMap_.find (leftAnkleJointName_);
	MapJrlJoint::const_iterator rightAnkle =
	  jointsMap_.find (rightAnkleJointName_);

	if (leftHand != jointsMap_.end () && leftWrist != jointsMap_.end ())
	  {
	    HandPtrType hand = factory_.createHand (leftWrist->second);

	    vector3d center (0., 0., 0.);
	    vector3d thumbAxis (0., 0., 0.);
	    vector3d foreFingerAxis (0., 0., 0.);
	    vector3d palmNormal (0., 0., 0.);

	    computeHandsInformation
	      (leftHand, leftWrist,
	       center, thumbAxis, foreFingerAxis, palmNormal);

	    hand->setCenter (center);
	    hand->setThumbAxis (thumbAxis);
	    hand->setForeFingerAxis (foreFingerAxis);
	    hand->setPalmNormal (palmNormal);
	    robot_->leftHand (hand);
	  }

	if (rightHand != jointsMap_.end () && rightWrist != jointsMap_.end ())
	  {
	    HandPtrType hand = factory_.createHand (rightWrist->second);

	    vector3d center (0., 0., 0.);
	    vector3d thumbAxis (0., 0., 0.);
	    vector3d foreFingerAxis (0., 0., 0.);
	    vector3d palmNormal (0., 0., 0.);

	    computeHandsInformation
	      (leftHand, leftWrist,
	       center, thumbAxis, foreFingerAxis, palmNormal);

	    hand->setCenter (center);
	    hand->setThumbAxis (thumbAxis);
	    hand->setForeFingerAxis (foreFingerAxis);
	    hand->setPalmNormal (palmNormal);

	    robot_->rightHand (hand);
	  }

	if (leftFoot != jointsMap_.end () && leftAnkle != jointsMap_.end ())
	  {
	    FootPtrType foot = factory_.createFoot (leftAnkle->second);
	    foot->setAnklePositionInLocalFrame
	      (computeAnklePositionInLocalFrame (leftFoot, leftAnkle));

	    //FIXME: to be determined using robot contact points definition.
	    foot->setSoleSize (0., 0.);

	    robot_->leftFoot (foot);
	  }

	if (rightFoot != jointsMap_.end () && rightAnkle != jointsMap_.end ())
	  {
	    FootPtrType foot = factory_.createFoot (rightAnkle->second);
	    foot->setAnklePositionInLocalFrame
	      (computeAnklePositionInLocalFrame (rightFoot, rightAnkle));

	    //FIXME: to be determined using robot contact points definition.
	    foot->setSoleSize (0., 0.);

	    robot_->rightFoot (foot);
	  }
      }

    } // end of namespace urdf.
  } // end of namespace dynamics.
} // end of namespace  jrl.
