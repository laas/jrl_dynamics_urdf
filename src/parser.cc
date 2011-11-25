
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

#include <urdf/model.h>
#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>
#include <jrl/dynamics/dynamicsfactory.hh>

#include "jrl/dynamics/urdf/parser.hh"

namespace jrl {
  namespace dynamics {
    namespace urdf {
      CjrlHumanoidDynamicRobot*
      Parser::parse (const std::string& filename) const
      {
	dynamicsJRLJapan::ObjectFactory factory;
	::urdf::Model model;
	model.initFile(filename);

	CjrlHumanoidDynamicRobot* robot = factory.createHumanoidDynamicRobot ();
	matrix4d position;
	position.setIdentity ();
	// Create root joint
	CjrlJoint* rootJoint = factory.createJointFreeflyer(position);
	// Create root body and set inertia parameters
	CjrlBody* body = createBody ();
	vector3d localCom(0,0,0);
	matrix3d inertiaMatrix;
	inertiaMatrix.setIdentity ();
	body.localCenterOfMass (localCom);
	body.inertiaMatrix (inertiaMatrix);
	// attach body to joint
	rootJoint->setLinkedBody (body);

	return robot;
      }
    } // urdf
  } // dynamics
} // jrl
