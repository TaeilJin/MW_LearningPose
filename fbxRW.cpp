
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

#define NCPtoNSP(n) (n+2) // convert # of control point to # of spline control points, which used in BSpline
#define NCPtoNSE(n) (n-1) // convert # of control point to # of spline segment, which used in BSpline


double DEBUG_DRAW_CONSTRAINT_SIZE = 2;
gVec3 MW_GRAVITY_VECTOR(0, -9.8, 0);
gVec3 MW_GROUND_NORMAL(0, 1, 0);


// test
bCharacter *avatar;
arma::mat refCoord;

osg::ref_ptr<osg::Group> debugGroup = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup2 = new osg::Group;
osg::ref_ptr<osg::Group> debugGroup3 = new osg::Group;

const static std::string kTaeilPath = "D:/Taeil_Jin/Development/feature_MWTaeil/Projects/Env_Retargeting/Data/demo/taeil/";
const static std::string kTaeilPath_TaskBVH = "D:/Taeil_Jin/Development/DATA/BODYAGENT_BVHFILE/P1/";
const static std::string kTaeilPath_TaskP1 = kTaeilPath_TaskBVH + "P1_1A_UP.bvh";
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

		v_start2end.set(pelvis_pos.x(), 0 , pelvis_pos.z());
		v_start2end.normalize();
		cout << " D " << Dist_MC << " v" << v_start2end <<endl;
	}
	else
	{
		std::cout << " write csvFile_motioncurve end " << std::endl;
		
		if(first_bool_motioncurve == false)
			myfile_motioncurve << Dist_MC << "," << v_start2end.x() << ","<< v_start2end.z() << "," << "\n";
		
		first_bool_motioncurve = true;
		myfile_motioncurve.close();
		
	}

}

//save motion curve
std::ofstream myfile_TaskPlane;
bool first_bool_TaskPlane = false;
//gXMat T_pelvis_tr_o; double Dist_MC = 0; gVec3 v_start2end;
//void writeCSVFile_motioncurve(int n_iter, int n_Motion) {
//	using namespace std;
//
//	// input: desk height, upper body length
//	// output: base position(x,z) foot position(x,y,z), leaning angle(spine1, spine2, spine3)
//
//
//	//write distance of each frame
//	if (first_bool_TaskPlane == false && n_iter < n_Motion) {
//
//		gLink* gLink_Rhand = avatar->findLink("RHand");
//		gLink* gLink_Lhand = avatar->findLink("LHand");
//		gVec3 rHand = gLink_Rhand->frame().multVec4(gLink_Rhand->inertia().comInVec3());
//		gVec3 lHand = gLink_Lhand->frame().multVec4(gLink_Lhand->inertia().comInVec3());
//		
//		//손의 중간 지점
//		gVec3 dirR2L = lHand - rHand;
//		double mag = dirR2L.magnitude(); mag = mag / 2; dirR2L.normalize();
//		gVec3 midPos = rHand + (dirR2L * mag);
//		
//		//손의 벡터
//		gVec3 rHandMid = avatar->findLink("RFinger2")->frame().trn();
//		gVec3 dirR2Tip = rHandMid - rHand;
//		dirR2Tip.normalize();
//
//		//Task Plane 의 normal 벡터
//		gVec3 Normal = gVec3(0, 1, 0);
//		//gVec3 dirTaskNormal = dirR2L % dirR2Tip; dirTaskNormal.normalize();
//		/*double scale = 4.0;
//		gVec3 NormalPos = midPos + -1 * dirTaskNormal*scale;
//		gVec3 TipPos = midPos + dirR2Tip*scale;
//		gVec3 mid2Pos = midPos + dirR2L*scale;
//		*/
//		/*gVec3 dirPlaneZ = Normal % dirR2L;
//		double scale = 4.0;
//		gVec3 NormalPos = midPos + Normal*scale;
//		gVec3 TipPos = midPos + -1 * dirPlaneZ*scale;
//		gVec3 mid2Pos = midPos + dirR2L*scale;
//*/
//
//
//		/*myfile_TaskPlane << pelvis_pos.x() << "," << pelvis_pos.y() << "," << pelvis_pos.z() << ",";
//		myfile_TaskPlane << rightF_pos.x() << "," << rightF_pos.y() << "," << rightF_pos.z() << ",";
//		myfile_TaskPlane << leeftF_pos.x() << "," << leeftF_pos.y() << "," << leeftF_pos.z() << "," << n_iter << "," << "\n";
//
//		Dist_MC = pelvis_pos.z();
//
//		v_start2end.set(pelvis_pos.x(), 0, pelvis_pos.z());
//		v_start2end.normalize();
//		cout << " D " << Dist_MC << " v" << v_start2end << endl;*/
//	}
//	else
//	{
//		std::cout << " write csvFile_motioncurve end " << std::endl;
//		first_bool_TaskPlane = true;
//		myfile_TaskPlane.close();
//
//	}
//
//}
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
	loader.loadMotionFile(kTaeilPath_TaskP1.c_str());
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

	// skin
	//gOsgSkin skin(avatar);
	//skin.loadSkin(scene, "writing.FBX");
	//skin.loadSkin(scene, "skin2/blonde.fbx");

	//{
	//	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("D:/code/FBXMG/msvc2017/skin/blonde.FBX");
	//	ReaderWriterFBX rw;
	//	osgDB::ReaderWriter::ReadResult res = rw.readNode("D:/code/FBXMG/msvc2017/skin2/blonde.fbx", new osgDB::Options());
	//	osg::ref_ptr<osg::Node> node = res.getNode();

	//	if (!node) {
	//		std::cout << "ERROR: Loading failed" << std::endl;
	//		return -1;
	//	}

	//	scene->addChild(node);
	//}

	//arma::mat m_data(motion->pMotions, motion->nChannel, motion->nMotion);
	//m_data.save("motion.csv", arma::csv_ascii);

	//int n_rows = avatar->numLinks() * 3 + 3;
	////cout << "rows " << avatar->numLinks() << " " << n_rows << " " << avatar->findLink("vc2")->id() << endl;
	//coords.resize(n_rows, 1);
	//myfile_coordinates.open("FemaleStopWalking_Lfirst.csv");
	//myfile_motioncurve.open("FemaleStopWalking_Lfirst_MC.csv");
	//avatar->setFromCompactCoordArray(refCoord.col(0));
	//avatar->updateKinematicsUptoPos();
	//avatar->updateKinematicBodiesOfCharacterSim();
	//T_pelvis_tr_o = avatar->link(0)->frame(); // pelvis world matrix

	double def_chair[16];

	def_chair[0] = -0.71115; def_chair[4] = 0;	def_chair[8] = -0.628634;   def_chair[12] = -8.5718;
	def_chair[1] = 0;		 def_chair[5] = 1;	def_chair[9] = 0;			def_chair[13] = 42.2658;
	def_chair[2] = 0.628634; def_chair[6] = 0;	def_chair[10] = -0.71115;   def_chair[14] = 41.8005;
	def_chair[3] = 0;		 def_chair[7] = 0;	def_chair[11] = 0;			def_chair[15] = 1;

	gXMat gMat_default_chair; gMat_default_chair.set(def_chair);

	/*gOSGShape::setColor(osg::Vec4(1.0, .0, .0, 1.0));
	debugGroup3->addChild(gOSGShape::createArrow(gVec3_2_OsgVec(gMat_default_chair.trn()), gVec3_2_OsgVec(gMat_default_chair.rotX()), gMat_default_chair.rotX().magnitude()*10.0, 1.0));
	gOSGShape::setColor(osg::Vec4(0.0, 1.0, .0, 1.0));
	debugGroup3->addChild(gOSGShape::createArrow(gVec3_2_OsgVec(gMat_default_chair.trn()), gVec3_2_OsgVec(gMat_default_chair.rotY()), gMat_default_chair.rotY().magnitude()*30.0, 1.0));
	gOSGShape::setColor(osg::Vec4(0.0, .0, 1.0, 1.0));
	debugGroup3->addChild(gOSGShape::createArrow(gVec3_2_OsgVec(gMat_default_chair.trn()), gVec3_2_OsgVec(gMat_default_chair.rotZ()), gMat_default_chair.rotZ().magnitude()*10.0, 1.0));
*/

	
	int iter = 0;
	double simulationTime = 0;
	while (!viewer->done())
	{
		viewer->frame(simulationTime);

		if (iter < 0)
			iter = 0;
		if (iter >= motion->nMotion)
			iter = 0;



		avatar->setFromCompactCoordArray(refCoord.col(15));
		avatar->updateKinematicsUptoPos();
		avatar->updateKinematicBodiesOfCharacterSim();
		AvatarWorld::getInstance()->visSys.update();

		gXMat T = avatar->baseLink()->frame();
		//std::cout << T << std::endl;
		//writeCSVFile_coords(nFnt, motion->nMotion);
		//writeCSVFile_motioncurve(nFnt, motion->nMotion);
		//nFnt++;

		//skin.updateSkin();


		debugGroup->removeChildren(0, debugGroup->getNumChildren());

		//drawBone(skeleton->boneRoot, motion->motions[5], debugGroup);


		debugGroup2->removeChildren(0, debugGroup2->getNumChildren());

		

		//Input
		// Task plane
		// position center position of hands / character height
		// object normal vector
		// hand direction relative with object normal vector
		gLink* gLink_rHand = avatar->findLink("RHand");
		gLink* gLink_lHand = avatar->findLink("LHand");
		gVec3 rHand = gLink_rHand->frame().multVec4(gLink_rHand->inertia().comInVec3());
		gVec3 lHand = gLink_lHand->frame().multVec4(gLink_lHand->inertia().comInVec3());

		/*debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(rHand.x(), rHand.y(), rHand.z()), 5.0));
		debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(lHand.x(), lHand.y(), lHand.z()), 5.0));*/

		gVec3 dirR2L = lHand - rHand; double mag = dirR2L.magnitude();
		mag = mag / 2;	dirR2L.normalize();
		gVec3 midHandPos = rHand + (dirR2L * mag);
		
		gOSGShape::setColor(osg::Vec4(0, 0, 1, 1));
		debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(midHandPos.x(), midHandPos.y(), midHandPos.z()), 10.0));
		
		//손의 벡터
		gVec3 rHandMid = avatar->findLink("RHand")->frame().trn();
		gVec3 dirR2Tip = avatar->findLink("RHand")->frame().rotX();
		dirR2Tip.normalize(); dirR2Tip = -1 * dirR2Tip;
		gVec3 pos2 = rHandMid + dirR2Tip * 10.0;
		debugGroup2->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(rHandMid), gVec3_2_OsgVec(pos2), 10.0));

		//손의 벡터
		gVec3 lHandMid = avatar->findLink("LHand")->frame().trn();
		gVec3 dirL2Tip = avatar->findLink("LHand")->frame().rotX();
		dirL2Tip.normalize(); dirL2Tip = dirL2Tip;
		gVec3 pos1 = lHandMid + dirL2Tip * 10.0;
		debugGroup2->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(lHandMid), gVec3_2_OsgVec(pos1), 10.0));


		//Task Plane 의 normal 벡터
		gVec3 Normal = gVec3(0, 1, 0);

		gVec3 dirPlaneZ = Normal % dirR2L;
		double scale = 9.0;
		gVec3 NormalPos = midHandPos + Normal*scale;
		gVec3 TipPos = midHandPos + -1 * dirPlaneZ*scale;
		gVec3 mid2Pos = midHandPos + dirR2L*scale;



		gOSGShape::setColor(osg::Vec4(1, 0, 0, 1));
		debugGroup2->addChild(gOSGShape::createLineShape(osg::Vec3(midHandPos.x(), midHandPos.y(), midHandPos.z()), osg::Vec3(midHandPos.x(), midHandPos.y(), midHandPos.z()), 1.0));
		gOSGShape::setColor(osg::Vec4(0, 0, 1, 1));
		debugGroup2->addChild(gOSGShape::createLineShape(osg::Vec3(midHandPos.x(), midHandPos.y(), midHandPos.z()), osg::Vec3(NormalPos.x(), NormalPos.y(), NormalPos.z()), 5.0));
		gOSGShape::setColor(osg::Vec4(0, 0, 1, 1));
		debugGroup2->addChild(gOSGShape::createLineShape(osg::Vec3(midHandPos.x(), midHandPos.y(), midHandPos.z()), osg::Vec3(TipPos.x(), TipPos.y(), TipPos.z()), 1.0));


		//Output
		// Foot position (seen from base) / character height
		// Base position (relative position with task position) / character height
		// Spine direction : calculate direction relative with up-vector(world frame normal)
		
		// Foot position
		gLink* FootR = avatar->findLink("RFoot"); 
		gLink* FootL = avatar->findLink("LFoot");
		gVec3 rFoot = FootR->frame().multVec4(FootR->inertia().comInVec3());
		gVec3 lFoot = FootL->frame().multVec4(FootL->inertia().comInVec3());

		
		/*debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(rFoot.x(), rFoot.y(), rFoot.z()), 5.0));
		debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(lFoot.x(), lFoot.y(), lFoot.z()), 5.0));*/

		dirR2L = lFoot - rFoot; mag = dirR2L.magnitude();
		mag = mag / 2;	dirR2L.normalize();
		gVec3 midFootPos = rFoot + (dirR2L * mag);
		gOSGShape::setColor(osg::Vec4(1, 0, 1, 1));
		debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(midFootPos.x(), midFootPos.y(), midFootPos.z()), 10.0));

		// base position
		gVec3 BasePos = avatar->baseLink()->frame().trn() - midHandPos;
		//
		BasePos = midHandPos + BasePos;
		//
		//gOSGShape::setColor(osg::Vec4(1, 0, 1, 1));
		debugGroup2->addChild(gOSGShape::createPoint(osg::Vec3(BasePos.x(), BasePos.y(), BasePos.z()), 10.0));

		// Spine direction
		int nSize = avatar->m_trunkIdx.size();
		for (int p = 0; p < nSize; p++)
		{
			int idx_j = avatar->m_trunkIdx[p];

			gLink* gLink_joint = avatar->link(idx_j);
			gVec3 dir_j = gLink_joint->frame().rotY(); // world direction of joint

			//gOSGShape::setColor(osg::Vec4(0, 1, 0, 1));
			gVec3 normal_pos1 = gLink_joint->frame().trn() + dir_j * 10;
			debugGroup2->addChild(gOSGShape::createLineShape(gVec3_2_OsgVec(gLink_joint->frame().trn()), osg::Vec3(normal_pos1.x(), normal_pos1.y(), normal_pos1.z()), 10.0));

			/*int j_idx;int j_idx_2;
			gLink* gLink_joint;	gLink* gLink_joint2;
			gVec3 dir_link;
			if (p == nSize - 1)
			{
				j_idx = avatar->m_trunkIdx[p];
				gLink_joint = avatar->link(j_idx);
				dir_link = gLink_joint->frame().multVec4(gLink_joint->inertia().comInVec3()) - gLink_joint->frame().trn();
				double mag = dir_link.magnitude();
				dir_link.normalize();

			}
			else
			{
				j_idx = avatar->m_trunkIdx[p];	j_idx_2 = avatar->m_trunkIdx[p + 1];
				gLink_joint = avatar->link(j_idx);	gLink_joint2 = avatar->link(j_idx_2);
				dir_link = gLink_joint2->frame().trn() - gLink_joint->frame().trn();
				double mag = dir_link.magnitude();
				dir_link.normalize();

			}

			gVec3 link_pos = gLink_joint->frame().trn(); gVec3 normal_pos = link_pos + dir_link * mag;
			gOSGShape::setColor(osg::Vec4(1, 0, 1, 1));
			debugGroup2->addChild(gOSGShape::createLineShape(osg::Vec3(link_pos.x(), link_pos.y(), link_pos.z()), osg::Vec3(normal_pos.x(), normal_pos.y(), normal_pos.z()), 10.0));
*/
		}
		
		
		
		
		simulationTime += 1. / 30.;
		iter++;

	}
	return 0;


}