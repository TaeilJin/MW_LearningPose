
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

#include "..\alglib-3.15.0.cpp.gpl\cpp\src\dataanalysis.h"
#include "TAE12\TAE12IK\mw_tiUtil_IK.h"

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test
bCharacter *avatar;
arma::mat refCoord;
mw_tiUtil_CHAIN *ti_IK;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

const static std::string kTaeilPath = "D:/Taeil_Jin/Development/feature_MWTaeil/Projects/Env_Retargeting/Data/demo/taeil/";
const static std::string kTaeilPath_TaskBVH = "D:/Taeil_Jin/Development/DATA/BODYAGENT_BVHFILE/";
const static std::string kTaeilPath_TaskP1 = kTaeilPath_TaskBVH + "P1/P1_1A_UP.bvh";
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


	debugGroup2->addChild(src_obj_node);
	debugGroup3->addChild(tar_obj_node);

	osg::ref_ptr<osg::Node> src_obj_node2 = osgDB::readNodeFile(kSourceObjectFileName2, options.get());
	osg::ref_ptr<osg::Node> tar_obj_node2 = osgDB::readNodeFile(kTargetObjectFileName2, options.get());
	src_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	tar_obj_node2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	src_p = src_obj_node2->getWorldMatrices().begin()->getTrans();
	std::cout << " x " << src_p.x() << " y " << src_p.y() << " z " << src_p.z() << std::endl;

	debugGroup2->addChild(src_obj_node2);
	debugGroup3->addChild(tar_obj_node2);
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

//save motion curve
std::ofstream myfile_TaskPlane;
bool first_bool_TaskPlane = false;
void writeCSVFile_TaskPlane(double cha_height, int n_iter, int n_Motion) {
	using namespace std;

	// input: position of task plane, normal of task plane, angle between normal and hand direction.
	// output: base position(x,z) foot position(x,y,z), leaning direction(spine)

	if (first_bool_TaskPlane == false && n_iter < n_Motion) {

		// Input
		// Task plane
		//-- 1.center position of hands / character height (midHandPos)
		gLink* gLink_rHand = avatar->findLink("RHand"); gLink* gLink_lHand = avatar->findLink("LHand");
		gVec3 rHand = gLink_rHand->frame().multVec4(gLink_rHand->inertia().comInVec3());
		gVec3 lHand = gLink_lHand->frame().multVec4(gLink_lHand->inertia().comInVec3());
		// center position
		gVec3 dirR2L = lHand - rHand; double mag = dirR2L.magnitude();
		mag = mag / 2;	dirR2L.normalize();
		gVec3 pos_CenterHand = rHand + (dirR2L * mag);
		pos_CenterHand /= cha_height;

		// write CSV FILE
		myfile_TaskPlane << pos_CenterHand.x() << "," << pos_CenterHand.y() << "," << pos_CenterHand.z() << ",";

		//-- 2.object normal vector
		gVec3 norm_TaskPlane = gVec3(0, 1, 0);

		// write CSV FILE
		myfile_TaskPlane << norm_TaskPlane.x() << "," << norm_TaskPlane.y() << "," << norm_TaskPlane.z() << ",";


		//-- 3.angle between hand direction and object normal vector

		//right hand
		gVec3 rHandMid = avatar->findLink("RHand")->frame().trn();
		gVec3 dirR2Tip = avatar->findLink("RHand")->frame().rotX();
		dirR2Tip.normalize(); dirR2Tip = -1 * dirR2Tip;
		//left hand
		gVec3 lHandMid = avatar->findLink("LHand")->frame().trn();
		gVec3 dirL2Tip = avatar->findLink("LHand")->frame().rotX();
		dirL2Tip.normalize(); dirL2Tip = dirL2Tip;

		//get normal
		gVec3 norm_Hands = dirR2Tip % dirL2Tip;
		double angles = std::acos(norm_Hands.operator,(norm_TaskPlane));

		// write CSV FILE
		myfile_TaskPlane << angles << ",";

		//Output
		// Foot position (seen from base) / character height (pos_CenterFoot)
		// Base position (relative position with task position) / character height (pos_Base)
		// Spine direction : calculate direction relative with up-vector(world frame normal)

		//--1.Foot position
		gLink* FootR = avatar->findLink("RFoot");	gLink* FootL = avatar->findLink("LFoot");
		gVec3 rFoot = FootR->frame().multVec4(FootR->inertia().comInVec3());
		gVec3 lFoot = FootL->frame().multVec4(FootL->inertia().comInVec3());

		dirR2L = lFoot - rFoot; mag = dirR2L.magnitude();
		mag = mag / 2;	dirR2L.normalize();
		gVec3 pos_CenterFoot = rFoot + (dirR2L * mag);
		pos_CenterFoot = avatar->baseLink()->frame().invMultVec4(pos_CenterFoot);
		pos_CenterFoot /= (cha_height);

		// write CSV FILE
		myfile_TaskPlane << pos_CenterFoot.x() << "," << pos_CenterFoot.y() << "," << pos_CenterFoot.z() << ",";


		//--2.base position (pos_Base)
		gVec3 pos_Base = avatar->baseLink()->frame().trn() - pos_CenterHand;
		pos_Base = pos_Base /= (cha_height);

		// write CSV FILE
		myfile_TaskPlane << pos_Base.x() << "," << pos_Base.y() << "," << pos_Base.z() << ",";


		//--3.Spine direction (dir_spineJoint)
		int nSize = avatar->m_trunkIdx.size();
		for (int p = 0; p < nSize; p++)
		{
			int idx_j = avatar->m_trunkIdx[p];

			gLink* gLink_joint = avatar->link(idx_j);
			gVec3 dir_spineJoint = gLink_joint->frame().rotY(); // world direction of joint
			if (p == nSize - 1)
			{									// write CSV FILE
				myfile_TaskPlane << dir_spineJoint.x() << "," << dir_spineJoint.y() << "," << dir_spineJoint.z() << "\n";
			}
			else {
				myfile_TaskPlane << dir_spineJoint.x() << "," << dir_spineJoint.y() << "," << dir_spineJoint.z() << ",";
			}
		}

	}
	else
	{
		std::cout << " write csvFile_motioncurve end " << std::endl;
		first_bool_TaskPlane = true;
		myfile_TaskPlane.close();

	}

}

//get gVec3 to OSGVec
osg::Vec3 gVec3_2_OsgVec(gVec3 gVec) {
	osg::Vec3 p(gVec.x(), gVec.y(), gVec.z());
	return p;
};
//get gVec3 to OSGVec
gVec3 OsgVec_2_gVec3(osg::Vec3 oVec) {
	gVec3 p(oVec.x(), oVec.y(), oVec.z());
	return p;
};
//print gVec3
void printgVec3(char* txt, gVec3 gVec) {
	std::cout << txt << " " << " x " << gVec.x() << " y " << gVec.y() << " z " << gVec.z() << std::endl;
}
int main(int argc, char **argv)
{
	// construct the viewer.
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;

	osg::ref_ptr<osg::Group> scene = new osg::Group;
	osg::ref_ptr<osg::MatrixTransform> rootTrans = new osg::MatrixTransform;
	osg::ref_ptr<osg::MatrixTransform> refTrans = new osg::MatrixTransform;

	//scene->addChild(rootTrans);
	//scene->addChild(refTrans);

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

	//initializeVisualization();

	MotionLoader loader;
	//loader.loadMotionFile(t_filename.c_str());
	//loader.loadMotionFile("Writing.fbx");
	loader.loadMotionFile((kTaeilPath_TaskBVH + "P1/P1_1A_UP.bvh").c_str());
	//loader.loadMotionFile("CRSF_FemaleStopWalking.fbx");

	mgData* motion = loader.getMotion();
	mgSkeleton* skeleton = loader.getSkeleton();


	//const double mass = 70.;
	//mgSkeletonToBCharacter::saveToBCharacter(skeleton, "P1_cha.txt", mass);

	AvatarWorld::getInstance()->loadAvatarModelFromFile("P1_cha.txt");
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


	myfile_TaskPlane.open("P1_Data.csv");

	std::stringstream him;
	std::ifstream myfile("Network1.txt");
	if (myfile.is_open())
	{
		him << myfile.rdbuf();

		myfile.close();

	}
	else {
		std::cout << " there is no file " << std::endl;
	}


	alglib::multilayerperceptron network;
	alglib::mlpunserialize(him.str(), network);
	alglib::real_1d_array x = "[-0.0124101,	-0.0933417,	0.255745,	0,	1,	0,	1.83689]";// "[-0.235816,	0.682514,	0.0530888,	0,	1,	0,	1.76715]";// "[-0.230186,	0.36868,	0.0459454,	0,	1,	0,	1.73736]";
	alglib::real_1d_array y = "[- 2.84E-07, - 0.467457,	0.0226382,	3.87E-08,	0.489463,	0.0161014,	0,	1,	0,	0,	1,	0,	0,	1,	0,	0	,1,	0,	0,	1,	0,	0,	1,	0]";
	mlpprocess(network, x, y);
	printf("%s\n", y.tostring(1).c_str()); // EXPECTED: [4.000]

	ti_IK = new mw_tiUtil_CHAIN(avatar);
	ti_IK->initChainInfo(ti_IK->sR_LegChain, 3, avatar->m_rLegIdx, gVec3(0, 0, 0), avatar->findLink("RFoot")->id(), true);
	ti_IK->initChainInfo(ti_IK->sL_LegChain, 4, avatar->m_lLegIdx, gVec3(0, 0, 0), avatar->findLink("LFoot")->id(), true);
	ti_IK->initChainInfo(ti_IK->sT_TrunkChain, 0, avatar->m_trunkIdx, gVec3(0, 0, 0), avatar->findLink("Head")->id(), true);
	double def_chair[16];

	def_chair[0] = -0.71115; def_chair[4] = 0;	def_chair[8] = -0.628634;   def_chair[12] = -8.5718;
	def_chair[1] = 0;		 def_chair[5] = 1;	def_chair[9] = 0;			def_chair[13] = 42.2658;
	def_chair[2] = 0.628634; def_chair[6] = 0;	def_chair[10] = -0.71115;   def_chair[14] = 41.8005;
	def_chair[3] = 0;		 def_chair[7] = 0;	def_chair[11] = 0;			def_chair[15] = 1;

	gXMat gMat_default_chair; gMat_default_chair.set(def_chair);


	int iter = 0;
	double simulationTime = 0;

	int nFnt = 0;
	while (!viewer->done())
	{
		viewer->frame(simulationTime);

		if (iter < 0)
			iter = 0;
		if (iter >= motion->nMotion)
			iter = 0;

		std::cout << " frame " << iter << std::endl;


		avatar->setFromCompactCoordArray(refCoord.col(10));
		avatar->updateKinematicsUptoPos();
		avatar->updateKinematicBodiesOfCharacterSim();





		debugGroup->removeChildren(0, debugGroup->getNumChildren());

		//drawBone(skeleton->boneRoot, motion->motions[5], debugGroup);

		//see input task plane
		gVec3 pos_taskPlane(-0.230186, 0.36868, 0.0459454);
		pos_taskPlane = pos_taskPlane * 176;
		gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_taskPlane), 5.0));

		//get base position from regressor
		double cha_length = 176;
		gVec3 pos_DesBasePos; pos_DesBasePos.set(y[3], y[4], y[5]);

		//
		pos_DesBasePos = pos_DesBasePos * cha_length;
		pos_DesBasePos = pos_DesBasePos + gVec3(-0.230186, 0.36868, 0.0459454);
		std::cout << " y  " << y[3] << y[4] << y[5] << std::endl;
		gOSGShape::setColor(osg::Vec4(0, 0, 1, 1));
		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_DesBasePos), 10.0));

		avatar->setBasePosition(pos_DesBasePos);
		avatar->updateKinematicsUptoPos();
		avatar->updateKinematicBodiesOfCharacterSim();



		////get foot position from regressor
		gVec3 pos_CenterFoot(y[0], y[1], y[2]);
		pos_CenterFoot *= cha_length;
		gVec3 pos_FootR = pos_CenterFoot - 10;
		gVec3 pos_FootL = pos_CenterFoot + 10;

		pos_CenterFoot = avatar->baseLink()->frame().multVec4(pos_CenterFoot);
		pos_FootR = avatar->baseLink()->frame().multVec4(pos_FootR);
		pos_FootL = avatar->baseLink()->frame().multVec4(pos_FootL);

		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_CenterFoot), 5.0));
		gOSGShape::setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));
		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_FootR), 10.0));
		gOSGShape::setColor(osg::Vec4(1.0, 0.0, 1.0, 1.0));
		debugGroup->addChild(gOSGShape::createPoint(gVec3_2_OsgVec(pos_FootL), 10.0));
		ti_IK->doChainIK(&ti_IK->sR_LegChain, &ti_IK->sR_LegDesiredEE, 1e-3, pos_FootR, 1.0, avatar->findLink("RToe")->frame().rotX(),
			avatar->findLink("RToe")->frame().rotY(), 0.0);
		ti_IK->doChainIK(&ti_IK->sL_LegChain, &ti_IK->sL_LegDesiredEE, 1e-3, pos_FootL, 1.0, avatar->findLink("RToe")->frame().rotX(),
			avatar->findLink("RToe")->frame().rotY(), 0.0);

		////get spine direction from regressor
		gOSGShape::setColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
		std::vector<int> gV_EEints_Dirs; std::vector<gVec3> gV_EEtarDirs;
		gV_EEints_Dirs.resize(avatar->m_trunkIdx.size());
		gV_EEtarDirs.resize(avatar->m_trunkIdx.size());
		for (int k = 0; k < avatar->m_trunkIdx.size(); k++) {
			double d_x = y[6 + 3 * k + 0];
			double d_y = y[6 + 3 * k + 1];
			double d_z = y[6 + 3 * k + 2];

			gVec3 dir_spine(d_x, d_y, d_z);

			gV_EEints_Dirs[k] = avatar->m_trunkIdx[k];
			gV_EEtarDirs[k] = dir_spine;

			int j_idx = avatar->m_trunkIdx[k];
			gVec3 pos_joint = avatar->link(j_idx)->frame().trn();
			gVec3 pos_end = pos_joint + dir_spine * 5.0;
			debugGroup->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(pos_joint), gVec3_2_OsgVec(pos_end), 7.0));
		}
		ti_IK->doChainIK(&ti_IK->sT_TrunkChain, &ti_IK->sT_TrkDesiredEE, 1e-3,
			avatar->findLink("Head")->frame().trn(), 0.3,
			avatar->findLink("Head")->frame().rotX(), avatar->findLink("Head")->frame().rotY(), 0.0,
			gV_EEtarDirs, gV_EEints_Dirs, 0.7);



		debugGroup2->removeChildren(0, debugGroup2->getNumChildren());


		//writeCSVFile_TaskPlane(176, nFnt, motion->nMotion);
		//nFnt++;

		AvatarWorld::getInstance()->visSys.update();

		simulationTime += 1. / 30.;
		iter++;

	}
	return 0;


}