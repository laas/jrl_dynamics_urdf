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
#include <boost/format.hpp>
#include "jrl/dynamics/urdf/parser.hh"

#include <ros/ros.h>

void displayPosition (const matrix4d & position, const std::string& prefix = "")
{
  static const char* appendToPrefix = " ";
  std::string newPrefix = prefix + appendToPrefix;
  for (unsigned i = 0; i < 4; ++i)
    {
      std::cout << newPrefix;
      for (unsigned j = 0; j < 4; ++j)
	std::cout << " " << position (i, j);
      std::cout << std::endl;
    }
}

void displayBody (const CjrlBody* body, const std::string& prefix = "")
{
  static const char* appendToPrefix = " ";

  if (!body)
    {
      std::cout << "null body" << std::endl;
      return;
    }

  std::string newPrefix = prefix + appendToPrefix;
  const matrix3d& inertia = body->inertiaMatrix();

  std::cout
    << newPrefix << "- ptr: " << body << std::endl
    << newPrefix << "- joint (ptr): " << body->joint () << std::endl
    << newPrefix << "- local center of mass: " << std::endl
    << newPrefix << " - X: " << body->localCenterOfMass ()[0] << std::endl
    << newPrefix << " - Y: " << body->localCenterOfMass ()[1] << std::endl
    << newPrefix << " - Z: " << body->localCenterOfMass ()[2] << std::endl
    << newPrefix << "- inertia: " << std::endl
    << newPrefix << inertia(0,0) << "\t\t" << inertia(0,1) << "\t\t" << inertia(0,2) << "\t\t" << std::endl
    << newPrefix << inertia(1,0) << "\t\t" << inertia(1,1) << "\t\t" << inertia(1,2) << "\t\t" << std::endl
    << newPrefix << inertia(2,0) << "\t\t" << inertia(2,1) << "\t\t" << inertia(2,2) << "\t\t" << std::endl
    << newPrefix << "- mass: " << body->mass () << std::endl;
}

void displayHand (const CjrlHand* hand, const std::string& prefix = "")
{
  static const char* appendToPrefix = " ";
  std::string newPrefix = prefix + appendToPrefix;

  if (!hand)
    {
      std::cout
	<< newPrefix << "null pointer" << std::endl;
      return;
    }

  vector3d center;
  hand->getCenter (center);

  vector3d thumbAxis;
  hand->getThumbAxis (thumbAxis);

  vector3d foreFingerAxis;
  hand->getForeFingerAxis (foreFingerAxis);

  vector3d palmNormal;
  hand->getPalmNormal (palmNormal);

  std::cout
    << newPrefix << "- associated wrist: " << hand->associatedWrist ()
    << std::endl
    << newPrefix << "- center: " << center
    << std::endl
    << newPrefix << "- thumb axis: " << thumbAxis
    << std::endl
    << newPrefix << "- fore finger axis: " << foreFingerAxis
    << std::endl
    << newPrefix << "- palm normal: " << palmNormal
    << std::endl;
}

void displayFoot (const CjrlFoot* foot, const std::string& prefix = "")
{
  static const char* appendToPrefix = " ";
  std::string newPrefix = prefix + appendToPrefix;

  if (!foot)
    {
      std::cout
	<< newPrefix << "null pointer" << std::endl;
      return;
    }
  vector3d anklePosition;
  foot->getAnklePositionInLocalFrame (anklePosition);

  double soleLength = 0.;
  double soleWidth = 0.;
  foot->getSoleSize (soleLength, soleWidth);

  std::cout
    << newPrefix << "- associated ankle: " << foot->associatedAnkle ()
    << std::endl
    << newPrefix << "- ankle position in local frame: " << anklePosition
    << std::endl
    << newPrefix << "- sole size: " << soleLength << " " << soleWidth
    << std::endl;
}

void displayJoint (const CjrlJoint* joint, const std::string& prefix = "")
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
    << newPrefix << "- rank: " ;

  if (joint->numberDof () > 0)
    std::cout << joint->rankInConfiguration ();
  else
    std::cout << "none";

  std::cout
    << std::endl
    << newPrefix << "- parent (ptr): " << joint->parentJoint () << std::endl
    << newPrefix << "- dof(s): " << joint->numberDof () << std::endl;
  for (unsigned i = 0; i < joint->numberDof (); ++i)
    {
      boost::format fmt
	("- bounds for dof %d: position : [%f; %f]");
      boost::format fmtVel
	("                    velocity : [%f; %f]");
      fmt % i % joint->lowerBound (i) % joint->upperBound (i);
      fmtVel % joint->lowerVelocityBound (i) % joint->upperVelocityBound (i);
      std::cout << newPrefix << fmt.str () << std::endl
		<< newPrefix << fmtVel.str () << std::endl;
    }
  std::cout
    << newPrefix << "- body: " << std::endl;
  displayBody (joint->linkedBody (), newPrefix);
  std::cout
    << newPrefix << "- initial position: " << std::endl;
  displayPosition (joint->initialPosition (), newPrefix);
  std::cout
    << newPrefix << "- current position: " << std::endl;
  displayPosition (joint->currentTransformation (), newPrefix);
  std::cout
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

void displayRobot (const CjrlHumanoidDynamicRobot* robot)
{
  assert (robot);
  std::cout << "Humanoid robot:" << std::endl
	    << " - waist: " << robot->waist () << std::endl
	    << " - chest: " << robot->chest () << std::endl
	    << " - left wrist: " << robot->leftWrist () << std::endl
	    << " - right wrist: " << robot->rightWrist () << std::endl
	    << " - left hand: " << robot->leftHand () << std::endl;
  displayHand (robot->leftHand (), " ");
  std::cout << " - right hand: " << robot->rightHand () << std::endl;
  displayHand (robot->rightHand (), " ");
  std::cout << " - left ankle: " << robot->leftAnkle () << std::endl
	    << " - right ankle: " << robot->rightAnkle () << std::endl
	    << " - left foot: " << robot->leftFoot () << std::endl;
  displayFoot (robot->leftFoot (), " ");
  std::cout << " - right foot: " << robot->rightFoot () << std::endl;
  displayFoot (robot->rightFoot (), " ");
  std::cout << " - gaze: " << robot->gazeJoint () << std::endl
	    << std::endl
	    << "Dynamic robot:" << std::endl
	    << " - total dof(s): " << robot->numberDof () << std::endl
	    << " - root joint: " << std::endl;
  displayJoint (robot->rootJoint ());
  displayActuatedJoints (robot->getActuatedJoints ());
}

void usage (int /*argc*/, char** argv)
{
  std::cerr << "Usage: " << argv[0] << " URDF_MODEL_PATH" << std::endl
	    << "or:    " << argv[0] << std::endl
	    << "URDF_MODEL_PATH is the location of the"
	    << " robot model to be displayed." << std::endl
	    << "Hint: use display-pr2 instead to avoid "
	    << "looking for the PR2 model manually."
	    << std::endl
	    << "When no argument is passed, robot_description"
	    << "ROS parameter is used"
	    << std::endl;

}

int main (int argc, char** argv)
{
  jrl::dynamics::urdf::Parser parser;

  CjrlHumanoidDynamicRobot* robot = 0;

  if (argc < 2)
    {
      std::cout
	<< "No model description has been given, "
	<< "retrieving model using ROS parameter (robot_description)."
	<< std::endl;

      ros::init (argc, argv, "display_robot");
      ros::NodeHandle nh;
      std::string robotDescription;
      ros::param::param<std::string>
	("robot_description", robotDescription, "");
      if (robotDescription.empty ())
	{
	  std::cout
	    << "No model available as ROS parameter. Fail."
	    << std::endl;
	  usage (argc, argv);
	  return 1;
	}
      robot = parser.parseStream (robotDescription);
    }
  else
    robot = parser.parse (argv[1]);

  if (!robot)
    {
      std::cerr << "Parse method returned a null pointer." << std::endl;
      return 1;
    }

  displayRobot (robot);
  return 0;
}
