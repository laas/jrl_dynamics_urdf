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

#ifndef JRL_DYNAMICS_URDF_PARSER
# define JRL_DYNAMICS_URDF_PARSER
# include <string>
# include <map>

# include <boost/shared_ptr.hpp>

# include <urdf/model.h>

# include <jrl/mal/matrixabstractlayer.hh>
# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
# include <jrl/dynamics/dynamicsfactory.hh>

class CjrlHumanoidDynamicRobot;

namespace jrl
{
  namespace dynamics
  {
    namespace urdf
    {
      /// \brief Parse an URDF file and return a CjrlHumanoidDynamicRobot.
      ///
      /// This class automatically reorient frames to match the
      /// abstact robot dynamics standard. I.e. rotation joints
      /// rotates along the X axis.
      ///
      /// The humanoid robot bodies location are guessed by making the
      /// assumption that the draft REP "Coordinate Frames for
      /// Humanoids Robots" is respected. See
      /// https://github.com/laas/rep-coordinate-frames-for-biped-robots
      /// for more information. If this is not the case, the tree
      /// will exist but the pointers toward specific parts of the
      /// body will be null.
      ///
      ///
      /// Implementation notes:
      ///
      /// The robot data are stored as attributes and reset each time
      /// a robot is returned to the user through the parse method.
      /// Practically speaking, this class never desallocates robots
      /// nor joints. It is the user responsibility to do so.
      class Parser
      {
      public:
	typedef boost::shared_ptr< ::urdf::Link> UrdfLinkPtrType;
	typedef boost::shared_ptr< ::urdf::Joint> UrdfJointPtrType;
	typedef boost::shared_ptr< ::urdf::JointLimits> UrdfJointLimitsPtrType;
	typedef boost::shared_ptr<const ::urdf::Link> UrdfLinkConstPtrType;
	typedef boost::shared_ptr<const ::urdf::Joint> UrdfJointConstPtrType;

	typedef CjrlJoint* JointPtrType;
	typedef CjrlBody* BodyPtrType;
	typedef CjrlHand* HandPtrType;
	typedef CjrlFoot* FootPtrType;

	/// \brief Map of abstract robot dynamics compatible joints.
	typedef std::map<std::string, JointPtrType> MapJrlJoint;
	/// \brief Map of URDF joints.
	typedef std::map<std::string, UrdfJointPtrType> MapJointType;

	/// \brief Default constructor, do nothing.
	explicit Parser ();
	/// \brief Destructor.
	virtual ~Parser ();

	/// \brief Parse an URDF file and return a humanoid robot.
	CjrlHumanoidDynamicRobot*
	parse (const std::string& filename,
	       const std::string& rootJointName);

	/// \brief Parse an URDF sent as a stream and return a
	/// humanoid robot.
	CjrlHumanoidDynamicRobot*
	parseStream (const std::string& robotDescription,
		     const std::string& rootJointName);

      protected:
	void
	parseActuatedJoints (const std::string rootJointName);

	std::vector<CjrlJoint*> actuatedJoints();
	void connectJoints(CjrlJoint* rootJoint);
	void addBodiesToJoints();

	void getChildrenJoint (const std::string& jointName,
			       std::vector<std::string>& result);

	// returns, in a vector, the children of a joint, or a
	// subchildren if no children si present in the actuated
	// joints map.
	std::vector<std::string>
	getChildrenJoint (const std::string& jointName);

	matrix4d getPoseInReferenceFrame(const std::string& referenceJoint,
					 const std::string& currentJoint);
	matrix4d poseToMatrix(::urdf::Pose p);
      private:
	::urdf::Model model_;
	CjrlHumanoidDynamicRobot* robot_;
	JointPtrType rootJoint_;
	MapJrlJoint jointsMap_;
	dynamicsJRLJapan::ObjectFactory factory_;

	/// \brief Special joints names.
	/// \{
	std::string waistJointName_;
	std::string chestJointName_;
	std::string leftWristJointName_;
	std::string rightWristJointName_;
	std::string leftHandJointName_;
	std::string rightHandJointName_;
	std::string leftAnkleJointName_;
	std::string rightAnkleJointName_;
	std::string leftFootJointName_;
	std::string rightFootJointName_;
	std::string gazeJointName_;
	/// \}

      }; // class Parser
    } // end of namespace urdf.
  } // end of namespace  dynamics.
} // end of namespace  jrl.

#endif // JRL_DYNAMICS_URDF_PARSER
