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
#include <cmath>
#include <boost/test/unit_test.hpp>

#include "jrl/dynamics/urdf/parser.hh"


#ifndef TEST_MODEL_DIRECTORY
# error "TEST_MODEL_DIRECTORY undefined"
#endif //! TEST_MODEL_DIRECTORY

BOOST_AUTO_TEST_CASE (oneDofRevoluteX)
{
  jrl::dynamics::urdf::Parser parser;
  CjrlHumanoidDynamicRobot* robot = parser.parse
    ("file://" TEST_MODEL_DIRECTORY "/one_dof_revolute_x.urdf");

  CjrlJoint* root = robot->rootJoint ();
  BOOST_CHECK (root);
  BOOST_CHECK_EQUAL (1, root->countChildJoints());
  BOOST_CHECK_EQUAL ("base_joint", root->getName());

  CjrlJoint* joint = root->childJoint (0);
  BOOST_CHECK (joint);
  BOOST_CHECK_EQUAL ("joint1", joint->getName());

  // Create robot configuration.
  const unsigned ndofs = 7;
  vectorN q (ndofs);
  vectorN dq (ndofs);
  vectorN ddq (ndofs);

  for (unsigned i = 0; i < ndofs; ++i)
    q (i) = dq (i) = ddq (i) = 0.;

  // Check configuration at zero.
  std::cout << q << std::endl;
  BOOST_CHECK (robot->currentConfiguration(q));
  BOOST_CHECK (robot->currentVelocity(dq));
  BOOST_CHECK (robot->currentAcceleration(ddq));
  BOOST_CHECK (robot->computeForwardKinematics());

  matrix4d jointPosition;

  jointPosition.setIdentity ();
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      BOOST_CHECK_EQUAL
	(jointPosition (i, j),
	 root->currentTransformation () (i, j));

  jointPosition.setIdentity ();
  jointPosition (1, 3) = 1.;

  // std::cout << "current value:" << std::endl;
  // std::cout << joint->currentTransformation () << std::endl;
  // std::cout << "expected value:" << std::endl;
  // std::cout << jointPosition << std::endl;

  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      BOOST_CHECK_CLOSE
	(jointPosition (i, j),
	 joint->currentTransformation () (i, j),
	 1e-9);

  // Check configuration at pi/2.
  q (6) = M_PI / 2.;
  std::cout << q << std::endl;
  BOOST_CHECK (robot->currentConfiguration(q));
  BOOST_CHECK (robot->currentVelocity(dq));
  BOOST_CHECK (robot->currentAcceleration(ddq));
  BOOST_CHECK (robot->computeForwardKinematics());

  jointPosition.setIdentity ();
  jointPosition (1, 1) = 0.;
  jointPosition (1, 2) = -1.;
  jointPosition (2, 1) = 1.;
  jointPosition (2, 2) = 0.;
  jointPosition (1, 3) = 1.;

  std::cout << "current value:" << std::endl;
  std::cout << joint->currentTransformation () << std::endl;
  std::cout << "expected value:" << std::endl;
  std::cout << jointPosition << std::endl;

  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      BOOST_CHECK_CLOSE
	(jointPosition (i, j),
	 joint->currentTransformation () (i, j),
	 1e-9);
}
