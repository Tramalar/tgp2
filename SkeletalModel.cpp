#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	ifstream skel;
	string line;
	skel.open(filename);

	while(getline(skel,line)){
		stringstream ss(line);
		Vector3f pos;
		int par;
		ss>>pos[0]>>pos[1]>>pos[2]>>par;
		Joint *joint=new Joint();
		joint->transform=Matrix4f::translation(pos);
		if(par==-1) 
			m_rootJoint=joint;
		else 
			m_joints[par]->children.push_back(joint);
		m_joints.push_back(joint);
	}
	m_addRoots.push_back(Vector3f(0,0,.5));	
	m_addRoots.push_back(Vector3f(0,0,-.5));
	m_addRoots.push_back(Vector3f(.5,0,0));	
	m_addRoots.push_back(Vector3f(-.5,0,0));
	m_addRoots.push_back(Vector3f(.5,0,.5));	
	m_addRoots.push_back(Vector3f(-.5,0,.5));
	m_addRoots.push_back(Vector3f(.5,0,-.5));	
	m_addRoots.push_back(Vector3f(-.5,0,-.5));
	// Load the skeleton from file here.
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.
	recuJoints(m_rootJoint);

	for(int i=0;i<m_addRoots.size();i++){
		Joint *newRoot=new Joint(*m_rootJoint);
		newRoot->transform[12]=newRoot->transform[12]+m_addRoots[i][0];
		newRoot->transform[13]=newRoot->transform[13]+m_addRoots[i][1];
		newRoot->transform[14]=newRoot->transform[14]+m_addRoots[i][2];
		recuJoints(newRoot);
	}
}


void SkeletalModel::recuJoints(Joint *joint){
	if(joint->children.size()==0){
		m_matrixStack.push(joint->transform);
		glLoadMatrixf(m_matrixStack.top());
		glutSolidSphere( 0.025f, 12, 12 );
		m_matrixStack.pop();	
		return;
	}
	for (int i=0;i<joint->children.size();i++){
		m_matrixStack.push(joint->transform);
		glLoadMatrixf(m_matrixStack.top());
		glutSolidSphere( 0.025f, 12, 12 );
		recuJoints(joint->children[i]);
		m_matrixStack.pop();
	}
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
	recuBones(m_rootJoint);

	
	for(int i=0;i<m_addRoots.size();i++){
		Joint *newRoot=new Joint(*m_rootJoint);
		newRoot->transform[12]=newRoot->transform[12]+m_addRoots[i][0];
		newRoot->transform[13]=newRoot->transform[13]+m_addRoots[i][1];
		newRoot->transform[14]=newRoot->transform[14]+m_addRoots[i][2];
		recuBones(newRoot);
	}
}

void SkeletalModel::recuBones(Joint *joint){	
	for (int i=0;i<joint->children.size();i++){
		m_matrixStack.push(joint->transform);
			glLoadMatrixf(m_matrixStack.top());
			Matrix4f ct=joint->children[i]->transform;
			//matriisin viimeinen pystyvektori kertoo translaatiosta suhteessa vanhempaan
			Vector3f offSet=Vector3f(ct[12],ct[13],ct[14]);
			float l=offSet.abs();

			Vector3f z=offSet.normalized();
			Vector3f y=Vector3f::cross(z,Vector3f(0,0,1));
			Vector3f x=Vector3f::cross(y,z);
			Matrix4f rot; rot.setCol(0,Vector4f(x,0)); rot.setCol(1,Vector4f(y,0));	rot.setCol(2,Vector4f(z,0)); rot.setCol(3,Vector4f(0,0,0,1));
			m_matrixStack.push(rot*Matrix4f::scaling(.025,.025,l)*Matrix4f::translation(Vector3f(0,0,.5)));
				glLoadMatrixf(m_matrixStack.top());
				glutSolidCube(1.f);
			m_matrixStack.pop();

			recuBones(joint->children[i]);
		m_matrixStack.pop();
	}
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	Matrix4f newM=Matrix4f::rotateX(rX)*Matrix4f::rotateY(rY)*Matrix4f::rotateZ(rZ);
	newM.setCol(3,Vector4f(m_joints[jointIndex]->transform[12],m_joints[jointIndex]->transform[13],m_joints[jointIndex]->transform[14],1));
	m_joints[jointIndex]->transform=newM;
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
}

