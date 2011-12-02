#include "jrl/dynamics/urdf/parser.hh"

int main (int argc, char** argv)
{
	jrl::dynamics::urdf::Parser parser;

    matrix4d res; 
	parser.parse("/opt/ros/diamondback/stacks/pr2_mechanism/pr2_mechanism_model/pr2.urdf");
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_shoulder_pan_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_shoulder_lift_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_upper_arm_roll_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_elbow_flex_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_forearm_roll_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_wrist_flex_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("torso_lift_joint"),::std::string("r_wrist_roll_joint"));
	//res= parser.getPoseInReferenceFrame(::std::string("base_footprint_joint"),::std::string("base_footprint_joint"));
	

/*
	::urdf::Pose p;
	p.rotation.x= 0;
	p.rotation.y= 0;
	p.rotation.z= 0;
	p.rotation.w= 1;
	p.position.x= 1;
	p.position.y= 1;
	p.position.z= 1;
	res= parser.poseToMatrix(p);


	for(int i=0; i<4; ++i)
	{
		for(int j=0; j<4; ++j)
		{
			printf("%f ", res(i,j));
		}
		printf("\n");
	}
*/

	return 0;
}
