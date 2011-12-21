
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

#ifndef JRL_DYNAMICS_URDF_PARSER
# define JRL_DYNAMICS_URDF_PARSER

# include <string>
# include <map>

# include <urdf/model.h>

# include <jrl/mal/matrixabstractlayer.hh>
# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
# include <jrl/dynamics/dynamicsfactory.hh>

class CjrlHumanoidDynamicRobot;

namespace jrl {
  namespace dynamics {
    namespace urdf {

	  typedef std::map< ::std::string, CjrlJoint* > MapJrlJoint;

      /// Parser that builds a robot of type CjrlHumanoidDynamicRobot
      /// The robot is read from an urdf file.
      class Parser {
        public:
		  ::urdf::Model model_;
		  MapJrlJoint jointsMap_;

	      Parser();
          virtual ~Parser();
          CjrlHumanoidDynamicRobot* parse(const std::string& filename,
										 const std::string& rootJointName);
	  	  CjrlJoint* parseActuatedJoints ( const std::string rootJointName);
	  	  std::vector<CjrlJoint*> actuatedJoints ( void );
	  	  void connectJoints(CjrlJoint* rootJoint);
	      void addBodiesToJoints(void); 

	  	  std::vector<std::string> getChildrenJoint(std::string jointName);
		  matrix4d getPoseInReferenceFrame(::std::string referenceJoint, ::std::string currentJoint);
	      matrix4d poseToMatrix(::urdf::Pose p);
      }; // class Parser
    } // urdf
  } // dynamics
} // jrl
#endif // JRL_DYNAMICS_URDF_PARSER
