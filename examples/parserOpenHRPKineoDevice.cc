/*
 *  Copyright 2007 CNRS/AIST
 *
 *  Authors: Florent Lamiraux and Fumio Kanehiro
 */

/************************

INCLUDE

************************/

#include <iostream>
#include "hppOpenHRP/parserOpenHRPKineoDevice.h" 


#include "KineoModel/kppSolidComponentRef.h"

#include "hpp/model/anchor-joint.hh"
#include "hpp/model/freeflyer-joint.hh"
#include "hpp/model/rotation-joint.hh"
#include "hpp/model/translation-joint.hh"
#include "hpp/model/body.hh"

#if DEBUG==2
#undef NDEBUG
#define ODEBUG2(x) std::cout << "CparserOpenHRPKineoDevice:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CparserOpenHRPKineoDevice:" << x << std::endl
#elif DEBUG==1
#undef NDEBUG
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CparserOpenHRPKineoDevice:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

using hpp::model::HumanoidRobotShPtr;
using hpp::model::HumanoidRobot;
using hpp::model::JointShPtr;
using hpp::model::DeviceShPtr;
using hpp::model::BodyShPtr;
using hpp::model::Body;
using hpp::model::AnchorJoint;
using hpp::model::FreeflyerJoint;
using hpp::model::RotationJoint;
using hpp::model::TranslationJoint;

/************************

METHODS

************************/

//=========================================================================

CparserOpenHRPKineoDevice::CparserOpenHRPKineoDevice( ModelInfo_ptr InOpenHRPModelInfo){
  
  attOpenHRPModelInfo = InOpenHRPModelInfo ;
  attOpenHRPModelSeq  = attOpenHRPModelInfo->getCharObject()->modelObjectSeq(); 
  
}

//=========================================================================

CparserOpenHRPKineoDevice::~CparserOpenHRPKineoDevice(){

  std::map<unsigned int, CkitMat4*>::iterator mapIter = jointAbsPosMap.begin();
  for (mapIter = jointAbsPosMap.begin(); 
       mapIter != jointAbsPosMap.end();
       mapIter++) {
    delete mapIter->second;
  }

}

//=========================================================================

void CparserOpenHRPKineoDevice::parser(HumanoidRobotShPtr& io_device)
{
  if (!io_device){
    string modelName = attOpenHRPModelInfo->getCharObject()->name() ;
    io_device = HumanoidRobot::create(modelName);
  }

  if (!io_device) {
    cerr << "  CparserOpenHRPKineoDevice::parser : Failed to initialize hppDevice" << endl ; 
    io_device.reset();
    return;
  }
 
  if (buildHppDevice(io_device) != KD_OK) {
    cerr << " CparserOpenHRPKineoDevice::parser :Failed to load model"  << endl;
    io_device.reset() ;
    return;
  }
    // Initialize dynamic part of robot.
  io_device->initialize();

  ODEBUG2("hppDevice after initialization: " << *(io_device.get()));

}


// ==============================================================================

ktStatus CparserOpenHRPKineoDevice::buildHppDevice(HumanoidRobotShPtr& hppDevice)
{
  // OpenHrp Joints (class ParserModelObject) are numbered by integer
  // ids.  For each Device joint, we build a KineoWorks device (class
  // CkwsJoint).  In order to keep track of the joint association, we
  // use an associative array.

  JointShPtr hppJoint;

  //
  // Loop over OpenHrp joints and build corresponding KineoWorks joints.
  //

  for(unsigned int iJoint=0; iJoint<attOpenHRPModelSeq->length(); iJoint++){

    (*(attOpenHRPModelSeq))[iJoint]->mother();
    std::string jointName((*(attOpenHRPModelSeq))[iJoint]->name());

    //debug ----------------- display info on display
    //cout << iJoint << "th model: name " << jointName
    // << " Id " <<  iJoint
    // << " mother " << pId << endl;
    //------------------------------------
    
    
    // Create KineoWorks joint corresponding to OpenHrp joint.
    hppJoint = createHppJoint((*(attOpenHRPModelSeq))[iJoint], iJoint, hppDevice, jointName);
    if (!hppJoint) {
      return KD_ERROR;
    }

    // Build bodies from OpenHrp triangles.
    if (createHppBodyAndAttachToJoint((*(attOpenHRPModelSeq))[iJoint], iJoint, hppJoint, jointName) != KD_OK) {
      return KD_ERROR;
    }


    // Store pair (jointId, hppJoint) in jointMap.
    jointMap[iJoint] = hppJoint;

  }

  return KD_OK;
}

// ==============================================================================

JointShPtr CparserOpenHRPKineoDevice::createHppJoint(const TopenHrpJoint& openHrpJoint,
						     long jointId,
						     DeviceShPtr hppDevice, const std::string jName)
{
  JointShPtr hppJoint, hppParentJoint;
  CkitPoint3 center;
  CkitVect3 axis;
  ValueMatrix *relPos;
  ValueMatrix *relAttitude;
  double jointValue;
  CkitMat4 *kwsAbsPosParentMatPtr=NULL;
  CkitMat4 kwsRelPosMat;
  CkitMat4 *kwsAbsPosMatPtr;
  // Id of OpenHrp parent joint.
  long parentId = openHrpJoint->mother();
    
  //
  // Get Parent of input hpp::model::Joint.
  //
    
  // test if joint is root joint.
  if (parentId >= 0) {
    //
    // Joint is not root joint.
    // test if parent joint is stored in map.
    if (jointMap.count(parentId) == 1) {
      hppParentJoint =  jointMap[parentId];
    } else {
      // If parent joint has not yet been put in jointMap return error.
      cerr << "ChppciOpenHrpClient::createKwsJoint: joint has no parent"
	   << endl;
      return JointShPtr();
    }
      
    // Get absolute position matrix of parent joint.
    kwsAbsPosParentMatPtr = jointAbsPosMap[parentId];
  } else {
    //
    // Joint is root joint, set position matrix of parent joint to Identity.
    //
    kwsAbsPosParentMatPtr = new CkitMat4; 
    kwsAbsPosParentMatPtr->identity();
  }
  relPos = openHrpJoint->relPos();
  relAttitude = openHrpJoint->relAttitude();
  jointValue = openHrpJoint->jointValue();
  if (jointValue != 0) {
    cerr << "ChppciOpenHrpClient::createKwsJoint: Joint value should be 0"
	 << endl;
    return hppJoint;
  }
  // Get values of rotation matrix from relative position and attitude.
  getPositionMatrix(relPos, relAttitude, kwsRelPosMat);
    
  //
  // Absolute position of current joint = absolute position parent joint * relative position current joint.
  kwsAbsPosMatPtr = new CkitMat4((*kwsAbsPosParentMatPtr)*kwsRelPosMat);

  // load the device from the center of the scene
  if(parentId < 0) {
    kwsAbsPosMatPtr->identity();
  }

  // Store absolute position of current joint in associative array.
  jointAbsPosMap[jointId] = kwsAbsPosMatPtr;
    
  //
  // determine type of joint
  //
  std::string jointType = openHrpJoint->jointType();
    
  if (jointType=="rotate" || jointType=="slide") {
    //
    // Rotation joint
    // 
      
    const std::string jointAxisStr[3] = {"X", "Y", "Z"};
    CkitPoint3 kwsLocCenter, kwsGlobCenter;
    CkitVect3 kwsLocAxis, kwsGlobAxis;
    // Get OpenHrp joint axis.
    std::string axis = openHrpJoint->jointAxis();
    for(unsigned int i=0; i<3; i++){
      if(axis == jointAxisStr[i]){
	kwsLocAxis[i] = 1.0;
      }
    }
    // Rotation axis is expressed in joint local frame. We need to
    // express it in the global frame.
    kwsGlobAxis = (*kwsAbsPosMatPtr)*kwsLocAxis;
    kwsGlobCenter = (*kwsAbsPosMatPtr)*kwsLocCenter;

    CkitMat4 jointPos = positionFromAxisAndCenter(kwsGlobAxis, kwsGlobCenter);

    if (jointType=="rotate"){
      //
      // Create rotation joint about X axis and move this joint
      // to absolute position.
      hppJoint = RotationJoint::create(jName,jointPos);
    }else{
      //
      // Create translation joint along X axis and move this joint
      // to absolute position.
      hppJoint = TranslationJoint::create(jName, jointPos);  
    }

    // Set joint limits.
    Value* upperBound = openHrpJoint->ulimit();
    Value* lowerBound = openHrpJoint->llimit();
    if (upperBound->length() > 0 && lowerBound->length() > 0) {
      hppJoint->bounds(0, (*lowerBound)[0], (*upperBound)[0]);
    }
    hppJoint->isBounded(0, true);

    // Set joint velocity limits.
    Value* uppervBound = openHrpJoint->uvlimit();
    Value* lowervBound = openHrpJoint->lvlimit();
    if (uppervBound->length() > 0 && lowervBound->length()){
      hppJoint->velocityBounds(0, (*lowervBound)[0], (*uppervBound)[0]);
    }
    

  } else if (jointType=="free") {
    //
    // Free joint.
    //
    hppJoint = FreeflyerJoint::create(jName, *kwsAbsPosMatPtr);

  } else if (jointType=="fixed") {
    // 
    // Fixed joint.
    //
    hppJoint = AnchorJoint::create(jName, *kwsAbsPosMatPtr);
  } else {
    return JointShPtr();  
  }
  // Attach joint to parent or set rootJoint.
  if (hppParentJoint) {
    hppParentJoint->addChildJoint(hppJoint);
  } else {
    // Set Root joint to device.
    hppDevice->setRootJoint(hppJoint);
  }
  return hppJoint;
}

   
// ==============================================================================

ktStatus
CparserOpenHRPKineoDevice::createHppBodyAndAttachToJoint
(const TopenHrpJoint& openHrpJoint, long jointId, 
 JointShPtr hppJoint, const std::string jName)
{
  CkppKCDPolyhedronShPtr kppPolyhedron;
  kppPolyhedron = buildHppPolyhedron(attOpenHRPModelInfo, openHrpJoint,
				     openHrpJoint->name());
  if (!kppPolyhedron){
    return KD_ERROR;
  }

  // make collision entity
  kppPolyhedron->makeCollisionEntity();

  // Get absolute position of joint reference frame.
  CkitMat4* jointAbsPosMatPtr;
  jointAbsPosMatPtr = jointAbsPosMap[jointId];
    
  // Express polyhedron in global reference frame.
  // Don't use it any more: will be done when specifying inner objects.
  // kppPolyhedron->handleTransform(*jointAbsPosMatPtr);
  // kppPolyhedron->transform(*jointAbsPosMatPtr);
    
  // create a body.
  hpp::model::impl::ObjectFactory factory;
  CjrlBody* jrlBody = factory.createBody();
    
  // Set physical properties.
  // Mass
  jrlBody->mass(openHrpJoint->mass());
  // Intertia matrix
  matrix3d inertiaInOpenHrpFrame = valueMatrixToMatrix3d(openHrpJoint->inertia());
  matrix3d inertiaInHppJointFrame = inertiaMatrixInHppJointFrame(inertiaInOpenHrpFrame, hppJoint);
  jrlBody->inertiaMatrix(inertiaInHppJointFrame);
  // Position of the center of mass.
  ValueMatrix *relCom = openHrpJoint->relComPos();
    
  // to get relative com posisiton for kineo Joint,
  // get inverse of kwsJoint rotation matrix and multiply its inverse to the vector.
    
  CkitVect3 openHRPRelCom((*relCom)[0], (*relCom)[1], (*relCom)[2]);
  CkitVect3 kwsRelCom = kwsTransRelJoint(openHRPRelCom, hppJoint->kppJoint()->kwsJoint());

  // Convert CkitVect3 to vector 3D of MatrixAbstractionLayer
  MAL_S3_VECTOR(jrlBodyRelCom, double);
  MAL_S3_VECTOR_ACCESS(jrlBodyRelCom, 0) = kwsRelCom[0];
  MAL_S3_VECTOR_ACCESS(jrlBodyRelCom, 1) = kwsRelCom[1];
  MAL_S3_VECTOR_ACCESS(jrlBodyRelCom, 2) = kwsRelCom[2];

  jrlBody->localCenterOfMass(jrlBodyRelCom);
    
  // Attach body to joint
  hpp::model::BodyShPtr hppBody = hpp::model::Body::create(jName + "-body");
  hppJoint->kppJoint()->kwsJoint()->setAttachedBody(hppBody);
  hppJoint->jrlJoint()->setLinkedBody(*jrlBody);
 
  hppBody->addInnerObject(CkppSolidComponentRef::create(kppPolyhedron), 
			  *jointAbsPosMatPtr, true);

  //debug -------------------- print infos on display
  //cout<<" joint "<<jointId<<": value "<<hppJoint->kwsJoint()->dof(0)->v()<<", name "
  //<<kppPolyhedron->name()<<endl;
  //cout<<"input value: ";
  //---------------------------   
 
  return KD_OK;
}

// ==============================================================================

matrix3d CparserOpenHRPKineoDevice::valueMatrixToMatrix3d(ValueMatrix* mat) 
{
  matrix3d m3;

  for(unsigned int iRow=0; iRow<3; iRow++) {
    for(unsigned int iCol=0; iCol<3; iCol++) {
      MAL_S3x3_MATRIX_ACCESS_I_J(m3, iRow, iCol) = (*mat)[iCol+3*iRow];
    }
  }
  return m3;
}

void CparserOpenHRPKineoDevice::CkitMat4ToMatrix3d(const CkitMat4& inMat4, matrix3d& outMatrix3d)
{
  for(unsigned int iRow=0; iRow<3; iRow++) {
    for(unsigned int iCol=0; iCol<3; iCol++) {
      MAL_S3x3_MATRIX_ACCESS_I_J(outMatrix3d, iRow, iCol) = inMat4(iRow, iCol);
    }
  }
}

// ==============================================================================

CkitVect3 CparserOpenHRPKineoDevice::kwsTransRelJoint(CkitVect3 &openHRPRelCom, CkwsJointShPtr kwsJoint) {

  CkitMat4 initMat = CkitMat4(kwsJoint->initialPosition()); // making copy
  // get only the rotation. = set translation zero.
  CkitMat4 invInitMat = initMat.t(); // transpose

  // By multipying mat4 and vect3, translation is not taken into account.
  CkitVect3 kwsRelCom = invInitMat * openHRPRelCom;

  return kwsRelCom;
}

// ==============================================================================

matrix3d CparserOpenHRPKineoDevice::inertiaMatrixInHppJointFrame(const matrix3d& inOpenHrpInertiaMatrix, 
								 JointShPtr inHppJoint)
{
  const CkitMat4& kwsJointPosition = inHppJoint->kppJoint()->kwsJoint()->initialPosition();
  const matrix3d& J0 = inOpenHrpInertiaMatrix;

  // P is the matrix of change of basis between the global frame basis to the basis
  // corresponding to Kineo Joint Orientation.
  MAL_S3x3_MATRIX(P, double);
  CkitMat4ToMatrix3d(kwsJointPosition, P);

  MAL_S3x3_MATRIX(invP, double);
  invP = MAL_S3x3_RET_TRANSPOSE(P);

  MAL_S3x3_MATRIX(J1, double);

  J1 = invP * J0 * P;

  return J1;
}

CkitMat4 CparserOpenHRPKineoDevice::positionFromAxisAndCenter(const CkitVect3& inAxis, 
							      const CkitPoint3& inCenter)
{
  CkitVect3 v1(inAxis);
  CkitVect3 v2(0, 0, 0);
  CkitVect3 v3(0, 0, 0);

  v1.normalize();

  unsigned int smallestComponent=0;
  double valueSmallestComponent = fabs(v1[0]);

  if (fabs(v1[1]) < fabs(v1[smallestComponent])) {
    smallestComponent = 1;
    valueSmallestComponent = fabs(v1[1]);
  }

  if (fabs(v1[2]) < fabs(v1[smallestComponent])) {
    smallestComponent = 2;
    valueSmallestComponent = fabs(v1[2]);
  }

  v2[smallestComponent] = 1;

  v3 = v1*v2;
  v2 = v3*v1;

  // (v1, v2, v3) form an orthonormal basis

  CkitMat4 outPositionMatrix;

  for (unsigned int iRow=0; iRow < 3; iRow++) {
    outPositionMatrix(iRow, 0) = v1[iRow];
    outPositionMatrix(iRow, 1) = v2[iRow];
    outPositionMatrix(iRow, 2) = v3[iRow];
    outPositionMatrix(iRow, 3) = inCenter[iRow];
  }
  return outPositionMatrix;
}

