
#include <Windows.h>
#include <iostream>
#include <algorithm>

#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgDB/ReadFile>

#include "Visualizer/gOSGShape.h"
#include "Loader/MotionLoader.h"
#include "Loader/mgSkeletonToBCharacter.h"
#include "AvatarWorld.h"

#include "MocapProcessor/mgMBSUtil.h"
#include "MocapProcessor/mgUtility.h"
#include "Visualizer/gEventHandler.h"

#include "Visualizer/gOSGSkin.h"

//
#include "TAE12/TAE12IK/mw_tiUtil_IK.h"

//


#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgAnimation/Bone>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/StackedScaleElement>
#include <osg/TriangleIndexFunctor>
#include <osgDB/Options>

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test
bCharacter *avatar;
arma::mat refCoord;
mw_tiUtil_CHAIN* tae12_IK;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

const static std::string kTaeilPath = "D:/Taeil_Jin/Development/feature_MWTaeil/Projects/Env_Retargeting/Data/demo/taeil/";
const static std::string kSourceObjectFileName = kTaeilPath + "vitra_noarm.obj"; //"VItra_noArm_tri.obj";// "source.obj";//"source.obj";
const static std::string kSourceObjectFileName2 = kTaeilPath + "test_desk_tri.obj";// "test_desk_tri.obj";
const static std::string kTargetObjectFileName = kTaeilPath + "vitra_noarm.obj";//"VItra_noArm_tri.obj";// "Long_woodbench.obj";//"Loft_small.obj";// Sofa_tri.obj";// "Sofa_tri.obj";// Vitra_tallChair.obj";
const static std::string kTargetObjectFileName2 = kTaeilPath + "WhiteBoard.obj"; // WhiteBoard;

void initializeVisualization()
{
	osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
	options->setOptionString("noRotation");
	osg::ref_ptr<osg::Node> src_obj_node = osgDB::readNodeFile(kSourceObjectFileName, options.get());
	osg::ref_ptr<osg::Node> tar_obj_node = osgDB::readNodeFile(kTargetObjectFileName, options.get());
	src_obj_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	tar_obj_node->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::Vec3 src_p = src_obj_node->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;


	//debugGroup2->addChild(src_obj_node);
	//debugGroup3->addChild(tar_obj_node);

	osg::ref_ptr<osg::Node> src_obj_node2 = osgDB::readNodeFile(kSourceObjectFileName2, options.get());
	osg::ref_ptr<osg::Node> tar_obj_node2 = osgDB::readNodeFile(kTargetObjectFileName2, options.get());
	src_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	tar_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	src_p = src_obj_node2->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;

	//debugGroup2->addChild(src_obj_node2);
	//debugGroup3->addChild(tar_obj_node2);
}
osg::Vec3 gVec32OSG(gVec3 input) {
	osg::Vec3 outvec(input.x(), input.y(), input.z());

	return outvec;
}
//

osgAnimation::BasicAnimationManager* findFirstOsgAnimationManagerNode(osg::Node* node)
{
	osgAnimation::BasicAnimationManager* manager = dynamic_cast<osgAnimation::BasicAnimationManager*>(node->getUpdateCallback());
	if (manager)
	{
		//std::cout << "name: " << node->getName() << std::endl;
		return manager;
	}

	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	//else, traverse children		
	for (int i = 0; i<group->getNumChildren(); ++i)
	{
		manager = findFirstOsgAnimationManagerNode(group->getChild(i));
		if (manager) return manager;
	}
	return NULL;
}

osgAnimation::Skeleton* findFirstOsgAnimationSkeletonNode(osg::Node* node)
{
	//return NULL if not osg::Group
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	//see if node is Skeleton
	osgAnimation::Skeleton* re = dynamic_cast<osgAnimation::Skeleton*>(node);
	if (re)  return re;

	//else, traverse children		
	for (int i = 0; i<group->getNumChildren(); ++i)
	{
		re = findFirstOsgAnimationSkeletonNode(group->getChild(i));
		if (re) return re;
	}
	return NULL;
}

osg::Vec3 drawBone(osg::ref_ptr<osg::Node> node, osg::ref_ptr<osg::Group> view_group)
{
	osg::ref_ptr<osgAnimation::Bone> b = dynamic_cast<osgAnimation::Bone*>(node.get());
	if (!b)
	{
		osg::ref_ptr<osg::Group> group = dynamic_cast<osg::Group*>(node.get());
		if (group)
		{
			for (int i = 0; i < group->getNumChildren(); ++i)
			{
				drawBone(group->getChild(i), view_group);
			}
		}
		return osg::Vec3(0, 0, 0);
	}

	osg::Matrix wMat = b->getWorldMatrices()[0];
	//osg::Matrix wMat = b->getMatrixInSkeletonSpace();
	//osg::Vec3 pos = b->getMatrix().getTrans();
	osg::Vec3 pos = wMat.getTrans();

	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	view_group->addChild(gOSGShape::createPoint(pos, 3.));

	for (int i = 0; i<b->getNumChildren(); ++i)
	{
		osg::Vec3 c_pos = drawBone(b->getChild(i), view_group);

		gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
		view_group->addChild(gOSGShape::createLineShape(pos, c_pos, 1.));

	}

	return pos;
}

osg::Vec3 drawBone(mgBone* bone, double* data, osg::ref_ptr<osg::Group> view_group)
{
	gXMat wMat;
	bone->skeleton->getWMatrixAt(bone->id, data, wMat);

	gVec3 pos_g = wMat.trn();
	osg::Vec3 pos(pos_g.x(), pos_g.y(), pos_g.z());

	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	view_group->addChild(gOSGShape::createPoint(pos, 3.));

	for (int i = 0; i<bone->children.size(); ++i)
	{
		osg::Vec3 c_pos = drawBone(bone->children[i], data, view_group);

		gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
		view_group->addChild(gOSGShape::createLineShape(pos, c_pos, 1.));
	}

	return pos;
}
void drawAxis(gXMat wMat, double pointsize, float linewidth, osg::ref_ptr<osg::Group> view_group)
{
	osg::Vec3 pos = gVec32OSG(wMat.trn()); 
	gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
	view_group->addChild(gOSGShape::createSphereShape(pointsize,pos));
	
	gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
	osg::Vec3 rotX = pos + (gVec32OSG(wMat.rotX())*linewidth);
	view_group->addChild(gOSGShape::createLineShape(pos, rotX, 1.0));
	
	gOSGShape::setColor(osg::Vec4(0, 1, 0, 1));
	osg::Vec3 rotY = pos + (gVec32OSG(wMat.rotY())*linewidth);
	view_group->addChild(gOSGShape::createLineShape(pos, rotY, 1.0));
	
	gOSGShape::setColor(osg::Vec4(0, 0, 1, 1));
	osg::Vec3 rotZ = pos + (gVec32OSG(wMat.rotZ())*linewidth);
	view_group->addChild(gOSGShape::createLineShape(pos, rotZ, 1.0));
}

#include "Loader/FBXLoader.h"

osg::ref_ptr<osg::Node> getNodeFromName(osg::ref_ptr<osg::Node> node, std::string& name)
{
	osg::Group* group = node->asGroup();
	if (!group) return NULL;

	if (node->getName() == name)
		return node;

	for (int i = 0; i < group->getNumChildren(); ++i)
	{
		osg::ref_ptr<osg::Node> n = getNodeFromName(group->getChild(i), name);
		if (n) return n;
	}
	return NULL;
}

gXMat getNodeTransform(osg::ref_ptr<osg::Node> node, FBXLoader* fbx)
{
	return gXMat();
}


arma::mat RemainingPoseGeneration(arma::vec input_pose, gVec3 pos_neck, gVec3 pos_rhand, gVec3 pos_lhand, gVec3 pos_footcenter);

//save motion coordinates
arma::vec coords;
std::ofstream myfile_coordinates;
bool first_bool_coordinates = false;
void writeCSVFile_coords(int n_iter, int n_Motion) {
	using namespace std;

	//write distance of each frame
	if (first_bool_coordinates == false && n_iter < n_Motion) {

		avatar->getCompactCoordArray(coords);

		myfile_coordinates << coords[0] * 180 / M_PI << "," << coords[1] * 180 / M_PI << "," << coords[2] * 180 / M_PI << ",";
		myfile_coordinates << coords[3] << "," << coords[4] << "," << coords[5] << ",";
		//myfile << g_frame << ",";// "," << 3 << "\n";// "a,b,c,\n";
		for (int i = 6; i < coords.n_rows; i++) {
			myfile_coordinates << coords[i] * 180 / M_PI << ",";
		}
		myfile_coordinates << n_iter << "," << "\n";

	}
	else
	{
		std::cout << " write csvFile_ foot position end " << std::endl;
		first_bool_coordinates = true;
		myfile_coordinates.close();
	}

}
//
//save motion curve
std::ofstream myfile_motioncurve;
bool first_bool_motioncurve = false;
gXMat T_pelvis_tr_o; double Dist_MC = 0; gVec3 v_start2end;
void writeCSVFile_motioncurve(int n_iter, int n_Motion) {
	using namespace std;

	//write distance of each frame
	if (first_bool_motioncurve == false && n_iter < n_Motion) {


		gVec3 pelvis_pos = avatar->link(0)->frame().trn(); // pelvis 
		gVec3 rightF_pos = avatar->findLink("r_ankle")->frame().trn(); // rightFoot
		gVec3 leeftF_pos = avatar->findLink("l_ankle")->frame().trn(); // leftFoot

		pelvis_pos = T_pelvis_tr_o.invMultVec4(pelvis_pos);
		rightF_pos = T_pelvis_tr_o.invMultVec4(rightF_pos);
		leeftF_pos = T_pelvis_tr_o.invMultVec4(leeftF_pos);


		myfile_motioncurve << pelvis_pos.x() << "," << pelvis_pos.y() << "," << pelvis_pos.z() << ",";
		myfile_motioncurve << rightF_pos.x() << "," << rightF_pos.y() << "," << rightF_pos.z() << ",";
		myfile_motioncurve << leeftF_pos.x() << "," << leeftF_pos.y() << "," << leeftF_pos.z() << "," << n_iter << "," << "\n";

		Dist_MC = pelvis_pos.z();

		v_start2end.set(pelvis_pos.x(), 0, pelvis_pos.z());
		v_start2end.normalize();
		cout << " D " << Dist_MC << " v" << v_start2end << endl;
	}
	else
	{
		std::cout << " write csvFile_motioncurve end " << std::endl;

		if (first_bool_motioncurve == false)
			myfile_motioncurve << Dist_MC << "," << v_start2end.x() << "," << v_start2end.z() << "," << "\n";

		first_bool_motioncurve = true;
		myfile_motioncurve.close();

	}

}

bool b_Nn = false; bool b_rhn = false; bool b_lhn = false; bool b_Fn = false;
void manipulateNeck(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_Nn = true; b_rhn = false; b_lhn = false; b_Fn = false;
}
void manipulateRhand(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_Nn = false; b_rhn = true; b_lhn = false; b_Fn = false;
}
void manipulateLhand(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_Nn = false; b_rhn = false; b_lhn = true; b_Fn = false;
}
void manipulateFoot(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_Nn = false; b_rhn = false; b_lhn = false; b_Fn = true;
}
bool b_rx = false; bool b_ry = false; bool b_rz = false;
void manipulateRotX(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_rx = true; b_ry = false; b_ry = false; 
}
void manipulateRotY(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_rx = false; b_ry = true; b_rz = false; 
}
void manipulateRotZ(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	b_rx = false; b_ry = false; b_rz = true; 
}

gXMat d_M_rhand; gXMat d_M_lhand; gXMat d_M_neck; gXMat d_M_f_center;
double cur_Ny = 0; double cur_rhy = 0; double cur_lhy = 0; double cur_Fy = 0;
void plusY(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		d_M_rhand.setTrn(d_M_rhand.trn().x(), d_M_rhand.trn().y() + 5, d_M_rhand.trn().z());
	}
	if (b_lhn == true) {
		d_M_lhand.setTrn(d_M_lhand.trn().x(), d_M_lhand.trn().y() + 5, d_M_lhand.trn().z());
	}
	if (b_Nn == true) {
		d_M_neck.setTrn(d_M_neck.trn().x(), d_M_neck.trn().y() + 5, d_M_neck.trn().z());
	}
	if (b_Fn == true) {
		d_M_f_center.setTrn(d_M_f_center.trn().x(), d_M_f_center.trn().y() + 5, d_M_f_center.trn().z());
	}
	
}
void minusY(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		d_M_rhand.setTrn(d_M_rhand.trn().x(), d_M_rhand.trn().y() - 5, d_M_rhand.trn().z());
	}
	if (b_lhn == true) {
		d_M_lhand.setTrn(d_M_lhand.trn().x(), d_M_lhand.trn().y() - 5, d_M_lhand.trn().z());
	}
	if (b_Nn == true) {
		d_M_neck.setTrn(d_M_neck.trn().x(), d_M_neck.trn().y() - 5, d_M_neck.trn().z());
	}
	if (b_Fn == true) {
		d_M_f_center.setTrn(d_M_f_center.trn().x(), d_M_f_center.trn().y() - 5, d_M_f_center.trn().z());
	}
}
double cur_x = 0;
void plusX(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		d_M_rhand.setTrn(d_M_rhand.trn().x() + 5, d_M_rhand.trn().y() , d_M_rhand.trn().z());
	}
	if (b_lhn == true) {
		d_M_lhand.setTrn(d_M_lhand.trn().x() + 5, d_M_lhand.trn().y() , d_M_lhand.trn().z());
	}
	if (b_Nn == true) {
		d_M_neck.setTrn(d_M_neck.trn().x() + 5, d_M_neck.trn().y() , d_M_neck.trn().z());
	}
	if (b_Fn == true) {
		d_M_f_center.setTrn(d_M_f_center.trn().x() + 5, d_M_f_center.trn().y() , d_M_f_center.trn().z());
	}
}
void minusX(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		d_M_rhand.setTrn(d_M_rhand.trn().x() - 5, d_M_rhand.trn().y() , d_M_rhand.trn().z());
	}
	if (b_lhn == true) {
		d_M_lhand.setTrn(d_M_lhand.trn().x() - 5, d_M_lhand.trn().y() , d_M_lhand.trn().z());
	}
	if (b_Nn == true) {
		d_M_neck.setTrn(d_M_neck.trn().x() - 5, d_M_neck.trn().y() , d_M_neck.trn().z());
	}
	if (b_Fn == true) {
		d_M_f_center.setTrn(d_M_f_center.trn().x() - 5, d_M_f_center.trn().y() , d_M_f_center.trn().z());
	}
}
double cur_z = 0;
void plusZ(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		d_M_rhand.setTrn(d_M_rhand.trn().x(), d_M_rhand.trn().y() , d_M_rhand.trn().z() + 5);
	}
	if (b_lhn == true) {
		d_M_lhand.setTrn(d_M_lhand.trn().x(), d_M_lhand.trn().y() , d_M_lhand.trn().z() + 5);
	}
	if (b_Nn == true) {
		d_M_neck.setTrn(d_M_neck.trn().x(), d_M_neck.trn().y() , d_M_neck.trn().z() + 5);
	}
	if (b_Fn == true) {
		d_M_f_center.setTrn(d_M_f_center.trn().x(), d_M_f_center.trn().y() , d_M_f_center.trn().z() + 5);
	}
}
void minusZ(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		d_M_rhand.setTrn(d_M_rhand.trn().x(), d_M_rhand.trn().y(), d_M_rhand.trn().z()-5);
	}
	if (b_lhn == true) {
		d_M_lhand.setTrn(d_M_lhand.trn().x(), d_M_lhand.trn().y(), d_M_lhand.trn().z() - 5);
	}
	if (b_Nn == true) {
		d_M_neck.setTrn(d_M_neck.trn().x(), d_M_neck.trn().y(), d_M_neck.trn().z() - 5);
	}
	if (b_Fn == true) {
		d_M_f_center.setTrn(d_M_f_center.trn().x(), d_M_f_center.trn().y(), d_M_f_center.trn().z() - 5);
	}
}

void plusR(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(+5));
			d_M_rhand.setRot(d_M_rhand.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(+5));
			d_M_rhand.setRot(d_M_rhand.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(+5));
			d_M_rhand.setRot(d_M_rhand.rot() * rh);
		}
	}
	if (b_lhn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(+5));
			d_M_lhand.setRot(d_M_lhand.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(+5));
			d_M_lhand.setRot(d_M_lhand.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(+5));
			d_M_lhand.setRot(d_M_lhand.rot() * rh);
		}
	}
	if (b_Nn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(+5));
			d_M_neck.setRot(d_M_neck.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(+5));
			d_M_neck.setRot(d_M_neck.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(+5));
			d_M_neck.setRot(d_M_neck.rot() * rh);
		}
	}
	if (b_Fn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(+5));
			d_M_f_center.setRot(d_M_f_center.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(+5));
			d_M_f_center.setRot(d_M_f_center.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(+5));
			d_M_f_center.setRot(d_M_f_center.rot() * rh);
		}
	}
}
void minusR(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (b_rhn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(-5));
			d_M_rhand.setRot(d_M_rhand.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(-5));
			d_M_rhand.setRot(d_M_rhand.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(-5));
			d_M_rhand.setRot(d_M_rhand.rot() * rh);
		}
	}
	if (b_lhn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(-5));
			d_M_lhand.setRot(d_M_lhand.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(-5));
			d_M_lhand.setRot(d_M_lhand.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(-5));
			d_M_lhand.setRot(d_M_lhand.rot() * rh);
		}
	}
	if (b_Nn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(-5));
			d_M_neck.setRot(d_M_neck.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(-5));
			d_M_neck.setRot(d_M_neck.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(-5));
			d_M_neck.setRot(d_M_neck.rot() * rh);
		}
	}
	if (b_Fn == true) {
		if (b_rx == true) {
			gRotMat rh;	rh.makeRotateX(tae12_IK->degree2radian(-5));
			d_M_f_center.setRot(d_M_f_center.rot() * rh);
		}
		if (b_ry == true) {
			gRotMat rh;	rh.makeRotateY(tae12_IK->degree2radian(-5));
			d_M_f_center.setRot(d_M_f_center.rot() * rh);
		}
		if (b_rz == true) {
			gRotMat rh;	rh.makeRotateZ(tae12_IK->degree2radian(-5));
			d_M_f_center.setRot(d_M_f_center.rot() * rh);
		}
	}
}

arma::mat getTotalCoordsOfMotion(const char* MotionFile, const char* MBSFile, arma::mat& refCoord) {
	MotionLoader loader;
	loader.loadMotionFile(MotionFile);

	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();

	refCoord.resize(AvatarWorld::getInstance()->avatar->sizeCompactCoordArray() + 1, motion->nMotion);// , arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			AvatarWorld::getInstance()->avatar,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}

	return refCoord;
}

int main(int argc, char **argv)
{
	// construct the viewer.
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	osg::ref_ptr<osg::Group> scene = new osg::Group;
	osg::ref_ptr<osg::MatrixTransform> rootTrans = new osg::MatrixTransform;
	osg::ref_ptr<osg::MatrixTransform> refTrans = new osg::MatrixTransform;

	
	scene->addChild(debugGroup);
	scene->addChild(debugGroup2);
	scene->addChild(debugGroup3);

	viewer->setSceneData(scene);


	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 10;
	traits->y = 10;
	traits->width = 1024;
	traits->height = 768;
	traits->windowDecoration = true;
	traits->supportsResize = false;
	traits->windowName = "test";

	osg::ref_ptr<osg::GraphicsContext> graphicscontext = osg::GraphicsContext::createGraphicsContext(traits);
	graphicscontext->realize();

	viewer->getCamera()->setGraphicsContext(graphicscontext);

	osg::Camera* camera = viewer->getCamera();

	camera->setClearColor(osg::Vec4(1., 1., 1., 1.0));
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width) / static_cast<double>(traits->height), 1.0f, 10000.0f);

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);
	manipulator->setAutoComputeHomePosition(true);
	//manipulator->setHomePosition(osg::Vec3(-100, 100, -100), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0), false);
	manipulator->home(0);

	int margin = 300;
	int w = margin * 2;
	int h = margin * 2;
	double size = 10;

	double startPoint_h = 0;
	double startPoint_w = 0;

	float width = 1.0f;

	gOSGShape::setColor(osg::Vec4(0, 0, 0, 1));
	for (int i = 0; i <= (h / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(startPoint_w, 0.0, i*size + startPoint_h), osg::Vec3(1, 0, 0), w + size, width));
	for (int i = 0; i <= (w / size + 1); i++)
		scene->addChild(gOSGShape::createLineShape(osg::Vec3(i*size + startPoint_w, 0.0, startPoint_h), osg::Vec3(0, 0, 1), h + size, width));
	scene->addChild(gOSGShape::createAxis(5.0, 5.0));

	initializeVisualization();
	osg::ref_ptr<gEventHandler> handler = new gEventHandler;
	handler->assignKey(osgGA::GUIEventAdapter::KEY_W, plusY);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_S, minusY);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_D, plusX);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_A, minusX);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_Q, plusZ);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_E, minusZ);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_Z, plusR);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_C, minusR);

	handler->assignKey(osgGA::GUIEventAdapter::KEY_0, manipulateNeck);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_1, manipulateRhand);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_2, manipulateLhand);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_3, manipulateFoot);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_I, manipulateRotX);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_O, manipulateRotY);
	handler->assignKey(osgGA::GUIEventAdapter::KEY_P, manipulateRotZ);

	viewer->addEventHandler(handler.get());

	
	MotionLoader loader;
	loader.loadMotionFile("skin2/blonde_motion/blonde_Writing_Idle.fbx");

	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();


	//const double mass = 70.;
	//mgSkeletonToBCharacter::saveToBCharacter(skeleton, "CRSF_Model2.txt", mass);

	AvatarWorld::getInstance()->loadAvatarModelFromFile("CRSF_Model2.txt");
	scene->addChild(AvatarWorld::getInstance()->visSys.getOSGGroup());
	avatar = AvatarWorld::getInstance()->avatar;

	
	arma::mat refCoord(avatar->sizeCompactCoordArray() + 1, motion->nMotion, arma::fill::zeros);
	for (int f = 0; f < motion->nMotion; f++)
	{
		arma::vec coord;

		mgMBSUtil::getCoordArrayFromRawData(
			coord,
			avatar,
			skeleton,
			motion->motions[f]
		);

		//refCoord.col(f) = coord;
		refCoord.submat(0, f, arma::SizeMat(coord.n_elem, 1)) = coord;
	}

	arma::mat refCoord_Writing;
	getTotalCoordsOfMotion("skin2/blonde_motion/blonde_Sitting_Idle.fbx", "CRSF_Model2.txt", refCoord_Writing);

	// skin
	gOsgSkin skin(avatar);
	skin.loadSkin(scene, "skin2/blonde.FBX");
	
	// ik
	tae12_IK = new mw_tiUtil_CHAIN(avatar);
	int RA_EE_idx = avatar->findLink("r_middle0")->id();
	tae12_IK->initChainInfo(tae12_IK->sR_ArmChain, 1, tae12_IK->character->m_rArmIdx, gVec3(0, 0, 0), RA_EE_idx, true); //right arm chain information initiation step		
	int LA_EE_idx = avatar->findLink("l_middle0")->id();
	tae12_IK->initChainInfo(tae12_IK->sL_ArmChain, 2, tae12_IK->character->m_lArmIdx, gVec3(0, 0, 0), LA_EE_idx, true); //right arm chain information initiation step		
	int LT_EE_idx = avatar->findLink("vc2")->id();
	tae12_IK->initChainInfo(tae12_IK->sT_TrunkChain,0, tae12_IK->character->m_trunkIdx, gVec3(0, 0, 0), LT_EE_idx, true); //right arm chain information initiation step		
	int RL_EE_idx = avatar->findLink("r_midtarsal")->id();
	tae12_IK->initChainInfo(tae12_IK->sR_LegChain,3, tae12_IK->character->m_rLegIdx, gVec3(0, 0, 0), RL_EE_idx, true); //right arm chain information initiation step		
	int LL_EE_idx = avatar->findLink("l_midtarsal")->id();
	tae12_IK->initChainInfo(tae12_IK->sL_LegChain,4, tae12_IK->character->m_lLegIdx, gVec3(0, 0, 0), LL_EE_idx, true); //right arm chain information initiation step		


	avatar->setFromCompactCoordArray(refCoord.col(1));
	avatar->updateKinematicsUptoPos();
	avatar->updateKinematicBodiesOfCharacterSim();

	d_M_rhand.setTrn(avatar->findLink("r_middle0")->frame().trn());
	d_M_lhand.setTrn(avatar->findLink("l_middle0")->frame().trn());
	d_M_neck.setTrn(avatar->findLink("vc2")->frame().trn());
	d_M_f_center.setTrn(avatar->findLink("l_midtarsal")->frame().multVec4(gVec3(-10, 0, 10)));

	int iter = 0; int nFnt = 0;
	double simulationTime = 0;
	while (!viewer->done())
	{
		viewer->frame(simulationTime);

		if (iter < 1)
			iter = 1;
		if (iter >= motion->nMotion)
			iter = 1;

		//
		avatar->setFromCompactCoordArray(refCoord.col(iter));
		avatar->updateKinematicsUptoPos();
		avatar->updateKinematicBodiesOfCharacterSim();

		d_M_rhand = avatar->findLink("r_middle0")->frame();
		d_M_lhand = avatar->findLink("l_middle0")->frame();
		d_M_neck = avatar->findLink("vc2")->frame();
		d_M_f_center = avatar->findLink("l_midtarsal")->frame();
		d_M_f_center.setTrn(d_M_f_center.multVec4(gVec3(-10, 0, 10)));
		RemainingPoseGeneration(refCoord_Writing.col(1), d_M_neck.trn(), d_M_rhand.trn(), d_M_lhand.trn(), d_M_f_center.trn());

		AvatarWorld::getInstance()->visSys.update();
		
		skin.updateSkin();
		
		debugGroup->removeChildren(0, debugGroup->getNumChildren());

		//desired position
		drawAxis(d_M_rhand, 2.0, 20.0, debugGroup);
		drawAxis(d_M_lhand, 2.0, 20.0, debugGroup);
		drawAxis(d_M_neck, 2.0, 20.0, debugGroup);
		drawAxis(d_M_f_center, 2.0, 20.0, debugGroup);

		

		//drawBone(skeleton->boneRoot, motion->motions[iter], debugGroup);

		simulationTime += 1. / 30.;
		iter++;

	}
	return 0;


}

arma::mat RemainingPoseGeneration(arma::vec input_pose, gVec3 pos_neck, gVec3 pos_rhand, gVec3 pos_lhand, gVec3 pos_footcenter)
{
	//
	using namespace std;

	//reference pose
	bCharacter* tar = AvatarWorld::getInstance()->avatar;
	
	tae12_IK->setTrunkSrcCoordFromRefCoord(&tae12_IK->sT_TrunkChain, input_pose);
	tae12_IK->setSrcCoordFromRefCoord(&tae12_IK->sR_ArmChain, input_pose);
	tae12_IK->setSrcCoordFromRefCoord(&tae12_IK->sL_ArmChain, input_pose);
	tae12_IK->setSrcCoordFromRefCoord(&tae12_IK->sR_LegChain, input_pose);
	tae12_IK->setSrcCoordFromRefCoord(&tae12_IK->sL_LegChain, input_pose);


	//if (tar->m_rArmIdx.size() != 0){
	//	
		gVec3 hnormZ = gVec3(0, 0, 1); gVec3 hnormY = gVec3(0, 1, 0);
		tae12_IK->doChainIK(&tae12_IK->sT_TrunkChain, &tae12_IK->sT_TrkDesiredEE, 0.1, pos_neck, 0.8, d_M_neck.rotZ(), d_M_neck.rotY(), 0.9);

		//Arm IK
		tae12_IK->doChainIK(&tae12_IK->sL_ArmChain, &tae12_IK->sL_ArmDesiredEE, 0.1, pos_lhand, 0.8, d_M_lhand.rotX(), d_M_lhand.rotY(), 0.9);
		tae12_IK->doChainIK(&tae12_IK->sR_ArmChain, &tae12_IK->sR_ArmDesiredEE, 0.1, pos_rhand, 0.8, -1*d_M_rhand.rotX(), d_M_rhand.rotY(), 0.9);
		
		//Foot IK
		int rfl = tae12_IK->sL_LegChain.JointIndexs.size() - 1; rfl = tae12_IK->sL_LegChain.JointIndexs[rfl];
		gVec3 footlpos = pos_footcenter; footlpos.setX(footlpos.x() + 10);
		
		int rf = tae12_IK->sR_LegChain.JointIndexs.size() - 1;	rf = tae12_IK->sR_LegChain.JointIndexs[rf];
		gVec3 footrpos = pos_footcenter; footrpos.setX(footrpos.x() - 10);
		
		tae12_IK->doChainIK(&tae12_IK->sL_LegChain, &tae12_IK->sL_LegDesiredEE, 0.1, footlpos, 0.08, d_M_f_center.rotZ(), d_M_f_center.rotY(), 0.9);
		tae12_IK->doChainIK(&tae12_IK->sR_LegChain, &tae12_IK->sR_LegDesiredEE, 0.1, footrpos, 0.08, d_M_f_center.rotZ(), d_M_f_center.rotY(), 0.9);
	//}

	//
	return arma::mat();
}
