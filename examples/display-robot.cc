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
#include <boost/foreach.hpp>
#include "jrl/dynamics/urdf/parser.hh"

void displayJoint (CjrlJoint* joint, const std::string& prefix = "")
{
  static const char* appendToPrefix = " ";

  if (!joint)
    {
      std::cout << "null joint" << std::endl;
      return;
    }

  std::string newPrefix = prefix + appendToPrefix;

  std::cout
    << newPrefix << "- ptr: " << joint << std::endl
    << newPrefix << "- name: " << joint->getName () << std::endl
    << newPrefix << "- parent (ptr): " << joint->parentJoint () << std::endl
    << newPrefix << "- dof(s): " << joint->numberDof () << std::endl
    << newPrefix << "- children: " << joint->countChildJoints () << std::endl;

  for (unsigned i = 0; i < joint->countChildJoints (); ++i)
    displayJoint (joint->childJoint (i), newPrefix);
}

void displayActuatedJoints (const std::vector<CjrlJoint*>& joints)
{
  std::cout << "Actuated joints:" << std::endl;
  if (joints.empty ())
    std::cout << "no actuated joints" << std::endl;
  else
    BOOST_FOREACH (CjrlJoint* joint, joints)
      std::cout << "- " << joint->getName () << std::endl;
}

void displayRobot (CjrlHumanoidDynamicRobot* robot)
{
  assert (robot);
  std::cout << "Humanoid robot:" << std::endl
	    << " - waist: " << robot->waist () << std::endl
	    << std::endl
	    << "Dynamic robot:" << std::endl
	    << " - root joint: " << std::endl;
  displayJoint (robot->rootJoint ());
  displayActuatedJoints (robot->getActuatedJoints ());
}

int main (int argc, char** argv)
{
  jrl::dynamics::urdf::Parser parser;

  if (argc < 2)
    {
      std::cerr << "Usage: " << argv[0] << " URDF_MODEL_PATH" << std::endl
		<< "URDF_MODEL_PATH is the location of the"
		<< " robot model to be displayed." << std::endl
		<< "Hint: use display-pr2 instead to avoid "
		<< "looking for the PR2 model manually."
		<< std::endl;
      return 1;	
    }

  CjrlHumanoidDynamicRobot* robot = parser.parse
    (argv[1], "base_footprint_joint");

  if (!robot)
    {
      std::cerr << "Parse method returned a null pointer." << std::endl;
      return 1;
    }

  displayRobot (robot);
  return 0;
}
