#include "mw_tiUtil_IK.h"



bool mw_tiUtil_CHAIN::checkReachable(intArray ia_chain, int startIdx, int endIdx, gVec3 tarpos)
{
	//the distance between root and target
	int rootIdx = ia_chain[1];
	gVec3 dist = tarpos - character->link(rootIdx)->frame().trn();
	double d_dist = dist.magnitude();

	//find bodylength
	double d_bodylength = getChainBodyLength(ia_chain, startIdx, endIdx);

	//check inside or not
	if (d_dist > d_bodylength)
		return false;
	else
		return true;
}

bool mw_tiUtil_CHAIN::checkReachable(sBodyChain s_chainBox, int startIdx, int endIdx, gVec3 tarpos)
{
	if (s_chainBox.size == 0) {
		printf("checking your chain box (Empty)");
		return false;
	}
	else {
		//the distance between root and target
		s_chainBox.AttachedPoints[startIdx].updateKinematicsUptoPos();
		gVec3 dist = tarpos - s_chainBox.AttachedPoints[startIdx].posWorld();
		double d_dist = dist.magnitude();

		//find bodylength
		std::cout << " dist " << d_dist <<" dd " << s_chainBox.totalchainLength << std::endl;
		//check inside or not
		if (d_dist > s_chainBox.totalchainLength)
			return false;
		else
			return true;
	}
}


arma::vec mw_tiUtil_CHAIN::getChainCompactCoord(arma::vec v_FullVec, intArray ia_chain)
{
	//
	arma::vec chainVec;
	chainVec.resize((ia_chain.size()) * 3); //chain joint angles

	int cnt = 0; // for loop
	for (int i = 0; i < ia_chain.size(); i++) {
		int jointIdx = ia_chain[i];
		int vectorIdx = 3 * jointIdx + 3;

		chainVec[cnt++] = v_FullVec[vectorIdx];
		chainVec[cnt++] = v_FullVec[vectorIdx + 1];
		chainVec[cnt++] = v_FullVec[vectorIdx + 2];

	}

	if (cnt != chainVec.size()) {
		std::cout << " Something Wrong: non-equality between jointVec size and chainVec size" << std::endl;
		arma::vec error; error.resize(ia_chain.size() * 3);
		return error.zeros();
	}
	else
		return chainVec;
}

arma::vec mw_tiUtil_CHAIN::getTrunkChainCompactCoord(arma::vec v_FullVec, intArray ia_chain)
{
	//
	arma::vec chainVec;
	chainVec.resize((ia_chain.size()) * 3 + 3); //chain joint angles +position

	chainVec[0] = v_FullVec[0];
	chainVec[1] = v_FullVec[1];
	chainVec[2] = v_FullVec[2];
	chainVec[3] = v_FullVec[3];
	chainVec[4] = v_FullVec[4];
	chainVec[5] = v_FullVec[5];

	int cnt = 6; // for loop
	for (int i = 1; i < ia_chain.size(); i++) {
		int jointIdx = ia_chain[i];
		int vectorIdx = 3 * jointIdx + 3;

		chainVec[cnt++] = v_FullVec[vectorIdx];
		chainVec[cnt++] = v_FullVec[vectorIdx + 1];
		chainVec[cnt++] = v_FullVec[vectorIdx + 2];

	}

	if (cnt != chainVec.size()) {
		std::cout << " Something Wrong: non-equality between jointVec size and chainVec size" << std::endl;
		arma::vec error; error.resize(ia_chain.size() * 3);
		return error.zeros();
	}
	else
		return chainVec;
}

void mw_tiUtil_CHAIN::updateChain()
{
	for (int p = 0; p < sR_ArmChain.AttachedPoints.size(); p++) { // update attached points (joint position + endeffector position) 
		sR_ArmChain.AttachedPoints[p].updateKinematicsUptoPos();
		sL_ArmChain.AttachedPoints[p].updateKinematicsUptoPos();
	}
	for (int p = 0; p < sT_TrunkChain.AttachedPoints.size(); p++) { // update attached points (joint position + endeffector position) 
		sT_TrunkChain.AttachedPoints[p].updateKinematicsUptoPos();
	}
	for (int p = 0; p < sR_LegChain.AttachedPoints.size(); p++) { // update attached points (joint position + endeffector position) 
		sR_LegChain.AttachedPoints[p].updateKinematicsUptoPos();
		sL_LegChain.AttachedPoints[p].updateKinematicsUptoPos();
	}
}

void objFuncRightLimIK(double *p, double *hx, int m, int n, void *adata) {
	mw_tiUtil_CHAIN* pt =  (mw_tiUtil_CHAIN*) adata;
	int cnt = 0;

	// p -> set joint matrix

	pt->setLimCompactCoordArray(&pt->sR_LegChain, pt->character, p);
	pt->character->updateKinematicsUptoPos();
	pt->character->updateKinematicBodiesOfCharacterSim();
	//cout << " " << p[0] << " " << p[1] << " " << p[2] << " " << pt->source_jointpose[ankle].GetPosition() << " " << pt->joint_world_matrix[0].get_row3(3) << endl;


	for (int i = 0; i < m; i++) {
		hx[cnt++] = (p[i] - pt->sR_LegChain.lca_srccoord[i]) * pt->sR_LegDesiredEE.gV_weight[2];
	}
	for (int des_i = 0; des_i < pt->sR_LegDesiredEE.gV_EEs.size(); des_i++) {
		int j_idx = pt->sR_LegDesiredEE.gV_EEints[des_i];
		gXMat j_frame = pt->character->link(j_idx)->frame();

		gVec3 del = pt->sR_LegDesiredEE.gV_EEs[des_i] - j_frame.trn();
		hx[cnt++] = (del.x()) * pt->sR_LegDesiredEE.gV_weight[0];
		hx[cnt++] = (del.y()) * pt->sR_LegDesiredEE.gV_weight[0];
		hx[cnt++] = (del.z()) * pt->sR_LegDesiredEE.gV_weight[0];

		gVec3 del_rX = pt->sR_LegDesiredEE.gV_EEtarNs[0] - j_frame.rotZ();
		hx[cnt++] = (del_rX.x()) * pt->sR_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.y()) * pt->sR_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.z()) * pt->sR_LegDesiredEE.gV_weight[1];

		gVec3 del_rY = pt->sR_LegDesiredEE.gV_EEtarNs[1] - j_frame.rotY();
		hx[cnt++] = (del_rY.x()) * pt->sR_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.y()) * pt->sR_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.z()) * pt->sR_LegDesiredEE.gV_weight[1];
	}

	if (cnt != n)
		std::cout << " error measurement vector size " << std::endl;
	return;
}
void objFuncLeftLimIK(double *p, double *hx, int m, int n, void *adata) {
	mw_tiUtil_CHAIN* pt = (mw_tiUtil_CHAIN*)adata;
	int cnt = 0;

	// p -> set joint matrix

	pt->setLimCompactCoordArray(&pt->sL_LegChain, pt->character, p);
	pt->character->updateKinematicsUptoPos();
	pt->character->updateKinematicBodiesOfCharacterSim();
	//cout << " " << p[0] << " " << p[1] << " " << p[2] << " " << pt->source_jointpose[ankle].GetPosition() << " " << pt->joint_world_matrix[0].get_row3(3) << endl;


	for (int i = 0; i < m; i++) {
		hx[cnt++] = (p[i] - pt->sL_LegChain.lca_srccoord[i]) * pt->sL_LegDesiredEE.gV_weight[2];
	}
	for (int des_i = 0; des_i < pt->sL_LegDesiredEE.gV_EEs.size(); des_i++) {
		int j_idx = pt->sL_LegDesiredEE.gV_EEints[des_i];
		gXMat j_frame = pt->character->link(j_idx)->frame();

		gVec3 del = pt->sL_LegDesiredEE.gV_EEs[des_i] - j_frame.trn();
		hx[cnt++] = (del.x()) * pt->sL_LegDesiredEE.gV_weight[0];
		hx[cnt++] = (del.y()) * pt->sL_LegDesiredEE.gV_weight[0];
		hx[cnt++] = (del.z()) * pt->sL_LegDesiredEE.gV_weight[0];

		gVec3 del_rX = pt->sL_LegDesiredEE.gV_EEtarNs[0] - j_frame.rotZ();
		hx[cnt++] = (del_rX.x()) * pt->sL_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.y()) * pt->sL_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.z()) * pt->sL_LegDesiredEE.gV_weight[1];

		gVec3 del_rY = pt->sL_LegDesiredEE.gV_EEtarNs[1] - j_frame.rotY();
		hx[cnt++] = (del_rY.x()) * pt->sL_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.y()) * pt->sL_LegDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.z()) * pt->sL_LegDesiredEE.gV_weight[1];

	}

	if (cnt != n)
		std::cout << " error measurement vector size " << std::endl;
	return;
}
void objFuncRightArmIK(double *p, double *hx, int m, int n, void *adata) {
	mw_tiUtil_CHAIN* pt = (mw_tiUtil_CHAIN*)adata;
	int cnt = 0;

	// p -> set joint matrix

	pt->setLimCompactCoordArray(&pt->sR_ArmChain, pt->character, p);
	pt->character->updateKinematicsUptoPos();
	pt->character->updateKinematicBodiesOfCharacterSim();
	//cout << " " << p[0] << " " << p[1] << " " << p[2] << " " << pt->source_jointpose[ankle].GetPosition() << " " << pt->joint_world_matrix[0].get_row3(3) << endl;


	for (int i = 0; i < m; i++) {
		hx[cnt++] = (p[i] - pt->sR_ArmChain.lca_srccoord[i]) * pt->sR_ArmDesiredEE.gV_weight[2];
	}
	for (int des_i = 0; des_i < pt->sR_ArmDesiredEE.gV_EEs.size(); des_i++) {
		int j_idx = pt->sR_ArmDesiredEE.gV_EEints[des_i];
		gXMat j_frame = pt->character->link(j_idx)->frame();

		gVec3 del = pt->sR_ArmDesiredEE.gV_EEs[des_i] - pt->character->link(j_idx)->frame().trn();
		hx[cnt++] = (del.x()) * pt->sR_ArmDesiredEE.gV_weight[0];
		hx[cnt++] = (del.y()) * pt->sR_ArmDesiredEE.gV_weight[0];
		hx[cnt++] = (del.z()) * pt->sR_ArmDesiredEE.gV_weight[0];

		gVec3 del_rX = pt->sR_ArmDesiredEE.gV_EEtarNs[0] - (-1*j_frame.rotX());
		hx[cnt++] = (del_rX.x()) * pt->sR_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.y()) * pt->sR_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.z()) * pt->sR_ArmDesiredEE.gV_weight[1];

		gVec3 del_rY = pt->sR_ArmDesiredEE.gV_EEtarNs[1] - j_frame.rotY();
		hx[cnt++] = (del_rY.x()) * pt->sR_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.y()) * pt->sR_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.z()) * pt->sR_ArmDesiredEE.gV_weight[1];

	}

	if (cnt != n)
		std::cout << " error measurement vector size " << std::endl;
	return;
}
void objFuncLeftArmIK(double *p, double *hx, int m, int n, void *adata) {
	mw_tiUtil_CHAIN* pt = (mw_tiUtil_CHAIN*)adata;
	int cnt = 0;

	// p -> set joint matrix

	pt->setLimCompactCoordArray(&pt->sL_ArmChain, pt->character, p);
	pt->character->updateKinematicsUptoPos();
	pt->character->updateKinematicBodiesOfCharacterSim();
	//cout << " " << p[0] << " " << p[1] << " " << p[2] << " " << pt->source_jointpose[ankle].GetPosition() << " " << pt->joint_world_matrix[0].get_row3(3) << endl;


	for (int i = 0; i < m; i++) {
		hx[cnt++] = (p[i] - pt->sL_ArmChain.lca_srccoord[i]) * pt->sL_ArmDesiredEE.gV_weight[2];
	}
	for (int des_i = 0; des_i < pt->sL_ArmDesiredEE.gV_EEs.size(); des_i++) {
		int j_idx = pt->sL_ArmDesiredEE.gV_EEints[des_i];
		gXMat j_frame = pt->character->link(j_idx)->frame();

		gVec3 del = pt->sL_ArmDesiredEE.gV_EEs[des_i] - pt->character->link(j_idx)->frame().trn();
		hx[cnt++] = (del.x()) * pt->sL_ArmDesiredEE.gV_weight[0];
		hx[cnt++] = (del.y()) * pt->sL_ArmDesiredEE.gV_weight[0];
		hx[cnt++] = (del.z()) * pt->sL_ArmDesiredEE.gV_weight[0];

		gVec3 del_rX = pt->sL_ArmDesiredEE.gV_EEtarNs[0] - (j_frame.rotX());
		hx[cnt++] = (del_rX.x()) * pt->sL_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.y()) * pt->sL_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.z()) * pt->sL_ArmDesiredEE.gV_weight[1];

		gVec3 del_rY = pt->sL_ArmDesiredEE.gV_EEtarNs[1] - j_frame.rotY();
		hx[cnt++] = (del_rY.x()) * pt->sL_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.y()) * pt->sL_ArmDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.z()) * pt->sL_ArmDesiredEE.gV_weight[1];

	}

	if (cnt != n)
		std::cout << " error measurement vector size " << std::endl;
	return;
}

void objFuncTrunkLimIK_Furniture(double *p, double *hx, int m, int n, void *adata) {
	mw_tiUtil_CHAIN* pt = (mw_tiUtil_CHAIN*)adata;
	int cnt = 0;

	// p -> set joint matrix

	pt->setTrunkLimCompactCoordArray(&pt->sT_TrunkChain, pt->character, p);
	pt->character->updateKinematicsUptoPos();
	pt->character->updateKinematicBodiesOfCharacterSim();
	//cout << " " << p[0] << " " << p[1] << " " << p[2] << " " << pt->source_jointpose[ankle].GetPosition() << " " << pt->joint_world_matrix[0].get_row3(3) << endl;

	for (int i = 0; i < m; i++) {
		hx[cnt++] = (p[i] - pt->sT_TrunkChain.lca_srccoord[i]) * pt->sT_TrkDesiredEE.gV_weight[2];
	}
	for (int des_i = 0; des_i < pt->sT_TrkDesiredEE.gV_EEs.size(); des_i++) {
		int j_idx = pt->sT_TrkDesiredEE.gV_EEints[des_i];
		gXMat j_frame = pt->character->link(j_idx)->frame();
		
		//position
		gVec3 del = pt->sT_TrkDesiredEE.gV_EEs[des_i] - j_frame.trn();

		hx[cnt++] = (del.x()) * pt->sT_TrkDesiredEE.gV_weight[0];
		hx[cnt++] = (del.y()) * pt->sT_TrkDesiredEE.gV_weight[0];
		hx[cnt++] = (del.z()) * pt->sT_TrkDesiredEE.gV_weight[0];

		gVec3 del_rX = pt->sT_TrkDesiredEE.gV_EEtarNs[0] - j_frame.rotZ();
		hx[cnt++] = (del_rX.x()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.y()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.z()) * pt->sT_TrkDesiredEE.gV_weight[1];

		gVec3 del_rY = pt->sT_TrkDesiredEE.gV_EEtarNs[1] - j_frame.rotY();
		hx[cnt++] = (del_rY.x()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.y()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.z()) * pt->sT_TrkDesiredEE.gV_weight[1];

	}
	

	if (cnt != n)
		std::cout << " error measurement vector size " << std::endl;
	return;
}
void objFuncTrunkLimIK_Furniture_withDir(double *p, double *hx, int m, int n, void *adata) {
	mw_tiUtil_CHAIN* pt = (mw_tiUtil_CHAIN*)adata;
	int cnt = 0;

	// p -> set joint matrix

	pt->setTrunkLimCompactCoordArray(&pt->sT_TrunkChain, pt->character, p);
	pt->character->updateKinematicsUptoPos();
	pt->character->updateKinematicBodiesOfCharacterSim();
	//cout << " " << p[0] << " " << p[1] << " " << p[2] << " " << pt->source_jointpose[ankle].GetPosition() << " " << pt->joint_world_matrix[0].get_row3(3) << endl;

	for (int i = 0; i < m; i++) {
		hx[cnt++] = (p[i] - pt->sT_TrunkChain.lca_srccoord[i]) * pt->sT_TrkDesiredEE.gV_weight[2];
	}
	for (int des_i = 0; des_i < pt->sT_TrkDesiredEE.gV_EEs.size(); des_i++) {
		int j_idx = pt->sT_TrkDesiredEE.gV_EEints[des_i];
		gXMat j_frame = pt->character->link(j_idx)->frame();

		//position
		gVec3 del = pt->sT_TrkDesiredEE.gV_EEs[des_i] - j_frame.trn();

		hx[cnt++] = (del.x()) * pt->sT_TrkDesiredEE.gV_weight[0];
		hx[cnt++] = (del.y()) * pt->sT_TrkDesiredEE.gV_weight[0];
		hx[cnt++] = (del.z()) * pt->sT_TrkDesiredEE.gV_weight[0];

		gVec3 del_rX = pt->sT_TrkDesiredEE.gV_EEtarNs[0] - j_frame.rotZ();
		hx[cnt++] = (del_rX.x()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.y()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rX.z()) * pt->sT_TrkDesiredEE.gV_weight[1];

		gVec3 del_rY = pt->sT_TrkDesiredEE.gV_EEtarNs[1] - j_frame.rotY();
		hx[cnt++] = (del_rY.x()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.y()) * pt->sT_TrkDesiredEE.gV_weight[1];
		hx[cnt++] = (del_rY.z()) * pt->sT_TrkDesiredEE.gV_weight[1];

	}

	for (int des_i = 0; des_i < pt->sT_TrkDesiredEE.gV_EEtarDirs.size(); des_i++) {
		int joint_idx = pt->sT_TrkDesiredEE.gV_EEints_Dirs[des_i];
		gVec3 dir_joint = pt->character->link(joint_idx)->frame().rotY();
		gVec3 dir_Desired = pt->sT_TrkDesiredEE.gV_EEtarDirs[des_i];

		gVec3 del_dir = dir_Desired - dir_joint;
		hx[cnt++] = (del_dir.x()) * pt->sT_TrkDesiredEE.gV_weight[2];
		hx[cnt++] = (del_dir.y()) * pt->sT_TrkDesiredEE.gV_weight[2];
		hx[cnt++] = (del_dir.z()) * pt->sT_TrkDesiredEE.gV_weight[2];

	}

	if (cnt != n)
		std::cout << " error measurement vector size " << std::endl;
	return;
}
void mw_tiUtil_CHAIN::rightLegIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src)
{
	int m = chain->size - 1; // joint number
	int dof = m * 3;
	//double* lca_r_srccoord = new double[dof];
	//chain->lca_srccoord = getLimCompactCoordArray(chain);
	
	if (chain->b_first == true) {
		chain->lca_precoord = chain->lca_srccoord; // 미리 할당되어져야한다.
		chain->b_first = false; // 미리 할당되어져야한다.
	}
	////ik function
	int sz = dof + EEBox->gV_EEs.size() * 3 +EEBox->gV_EEtarNs.size() * 3; // dof:source pose, EEposition
	double* lb = new double[dof]; double* ub = new double[dof];
	lb[0] = -degree2radian(105);   lb[1] = -degree2radian(90);   lb[2] = -degree2radian(90);
	lb[3] = -degree2radian(1e-3); lb[4] = -degree2radian(3); lb[5] = -degree2radian(10);
	lb[6] = -degree2radian(30);   lb[7] = -degree2radian(1e-3);   lb[8] = -degree2radian(10);

	ub[0] = degree2radian(105);    ub[1] = degree2radian(90);   ub[2] = degree2radian(45);
	ub[3] = degree2radian(100);    ub[4] = degree2radian(3); ub[5] = degree2radian(10);
	ub[6] = degree2radian(30);	  ub[7] = degree2radian(5);   ub[8] = degree2radian(10);
	dlevmar_bc_dif(objFuncRightLimIK, chain->lca_precoord, NULL, dof, sz,lb,ub,NULL, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	printf("nIter=%d, reason=%d, e=%g to %g\n", int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0], LevMarInfo[1]);

	setLimCompactCoordArray(&sR_LegChain, character, chain->lca_precoord);
		
	character->updateKinematicsUptoPos();
	character->updateKinematicBodiesOfCharacterSim();
}

void mw_tiUtil_CHAIN::leftLegIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src)
{
	int m = chain->size - 1; // joint number
	int dof = m * 3;
	//chain->lca_srccoord = getLimCompactCoordArray(chain);

	if (chain->b_first == true) {
		chain->lca_precoord = chain->lca_srccoord; // 미리 할당되어져야한다.
		chain->b_first = false; // 미리 할당되어져야한다.
	}
	////ik function
	int sz = dof + EEBox->gV_EEs.size() * 3 + EEBox->gV_EEtarNs.size() * 3; // dof:source pose, EEposition
	double* lb = new double[dof]; double* ub = new double[dof];
	lb[0] = -degree2radian(105);   lb[1] = -degree2radian(90);   lb[2] = -degree2radian(45);
	lb[3] = -degree2radian(1e-3); lb[4] = -degree2radian(3); lb[5] = -degree2radian(10);
	lb[6] = -degree2radian(30);   lb[7] = -degree2radian(1);   lb[8] = -degree2radian(10);

	ub[0] = degree2radian(105);    ub[1] = degree2radian(90);   ub[2] = degree2radian(90);
	ub[3] = degree2radian(100);    ub[4] = degree2radian(3); ub[5] = degree2radian(10);
	ub[6] = degree2radian(30);	  ub[7] = degree2radian(5);   ub[8] = degree2radian(10);
	dlevmar_bc_dif(objFuncLeftLimIK, chain->lca_precoord, NULL, dof, sz,lb,ub, NULL, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	//dlevmar_dif(objFuncLeftLimIK, chain->lca_precoord, NULL, dof, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	printf("nIter=%d, reason=%d, e=%g to %g\n", int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0], LevMarInfo[1]);

	setLimCompactCoordArray(&sL_LegChain, character, chain->lca_precoord);

	character->updateKinematicsUptoPos();
	character->updateKinematicBodiesOfCharacterSim();
}

void mw_tiUtil_CHAIN::rightArmIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src)
{
	int m = chain->size - 1; // joint number
	int dof = m * 3;
	//chain->lca_srccoord = getLimCompactCoordArray(chain);
	
	if (chain->b_first == true) {
		chain->lca_precoord = chain->lca_srccoord; // 미리 할당되어져야한다.
		chain->b_first = false; // 미리 할당되어져야한다.
	}
	
	////ik function
	int sz = dof + EEBox->gV_EEs.size() * 3 + (EEBox->gV_EEtarNs.size()  * 3); // dof:source pose, EEposition EEBox->gV_EEtarNs.size()
	//dlevmar_dif(objFuncRightArmIK, chain->lca_precoord, NULL, dof, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	double* lb = new double[dof]; double* ub = new double[dof];
	lb[0] = -degree2radian(13); lb[1] = -degree2radian(1e-3); lb[2] = -degree2radian(70);
	lb[3] = -degree2radian(1); lb[4] = -degree2radian(1e-3); lb[5] = -degree2radian(5);
	lb[6] = -degree2radian(90);   lb[7] = -degree2radian(30);   lb[8] = -degree2radian(90);

	ub[0] = degree2radian(13);  ub[1] = degree2radian(90); ub[2] = degree2radian(60);
	ub[3] = degree2radian(1);  ub[4] = degree2radian(90); ub[5] = degree2radian(5);
	ub[6] = degree2radian(90);	  ub[7] = degree2radian(30); ub[8] = degree2radian(90);

	dlevmar_bc_dif(objFuncRightArmIK, chain->lca_precoord, NULL, dof, sz,lb,ub, NULL, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	printf("nIter=%d, reason=%d, e=%g to %g\n", int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0], LevMarInfo[1]);

	setLimCompactCoordArray(&sR_ArmChain, character, chain->lca_precoord);

	character->updateKinematicsUptoPos();
	character->updateKinematicBodiesOfCharacterSim();
}

void mw_tiUtil_CHAIN::leftArmIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src)
{
	int m = chain->size - 1; // joint number
	int dof = m * 3;
	//chain->lca_srccoord = getLimCompactCoordArray(chain);

	if (chain->b_first == true) {
		chain->lca_precoord = chain->lca_srccoord; // 미리 할당되어져야한다.
		chain->b_first = false; // 미리 할당되어져야한다.
	}
	////ik function
	int sz = dof + EEBox->gV_EEs.size() * 3 + (EEBox->gV_EEtarNs.size() * 3); // dof:source pose, EEposition EEBox->gV_EEtarNs.size()
	double* lb = new double[dof]; double* ub = new double[dof];
	lb[0] = -degree2radian(31); lb[1] = -degree2radian(90);   lb[2] = -degree2radian(45);
	lb[3] = -degree2radian(1); lb[4] = -degree2radian(90); lb[5] = -degree2radian(5);
	lb[6] = -degree2radian(90);   lb[7] = -degree2radian(30);   lb[8] = -degree2radian(90);

	ub[0] = degree2radian(31);  ub[1] = degree2radian(1e-3);   ub[2] = degree2radian(70);
	ub[3] = degree2radian(1);  ub[4] = degree2radian(1e-3); ub[5] = degree2radian(5);
	ub[6] = degree2radian(90);	  ub[7] = degree2radian(30);   ub[8] = degree2radian(90);

	//dlevmar_dif(objFuncLeftArmIK, chain->lca_precoord, NULL, dof, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	dlevmar_bc_dif(objFuncLeftArmIK, chain->lca_precoord, NULL, dof, sz, lb, ub, NULL, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	printf("nIter=%d, reason=%d, e=%g to %g\n", int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0], LevMarInfo[1]);

	setLimCompactCoordArray(&sL_ArmChain, character, chain->lca_precoord);

	character->updateKinematicsUptoPos();
	character->updateKinematicBodiesOfCharacterSim();
}

void mw_tiUtil_CHAIN::doChainIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * desEE, float refPosWeight,
	gVec3 desPos, float posWeight, gVec3 desRotX, gVec3 desRotY, float dirWeight)
{
	//pos
	std::vector<gVec3> points; std::vector<gVec3> normls; std::vector<int> joint_indices; std::vector<float> weights;
	int ee_jointidx = chain->JointIndexs.size() - 1;
	ee_jointidx = chain->JointIndexs[ee_jointidx];
	points.push_back(desPos); joint_indices.push_back(ee_jointidx); weights.push_back(posWeight);

	//dir
	gVec3 normX = desRotX;  normls.push_back(normX); gVec3 normY = desRotY;  normls.push_back(normY);
	weights.push_back(dirWeight);
	
	//reference
	weights.push_back(refPosWeight);

	//update EEBox
	desEE->gV_EEs = points; desEE->gV_EEtarNs = normls;
	desEE->gV_EEints = joint_indices; desEE->gV_weight = weights;

	if(chain->chain_name_idx == 0)
		TrunkIK(chain, desEE, character);
	if (chain->chain_name_idx == 1)
		rightArmIK(chain, desEE, character);
	if (chain->chain_name_idx == 2)
		leftArmIK(chain, desEE, character);
	if (chain->chain_name_idx == 3)
		rightLegIK(chain, desEE, character);
	if (chain->chain_name_idx == 4)
		leftLegIK(chain, desEE, character);
}

void mw_tiUtil_CHAIN::doChainIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * desEE, float refPosWeight, 
	gVec3 desPos, float posWeight, 
	gVec3 desRotX, gVec3 desRotY, float dirWeight, 
	std::vector<gVec3> desDirs, std::vector<int> desDirs_indices, float linkDirWeight)
{
	//pos
	std::vector<gVec3> points; std::vector<gVec3> normls; std::vector<int> joint_indices; std::vector<float> weights;
	int ee_jointidx = chain->JointIndexs.size() - 1;
	ee_jointidx = chain->JointIndexs[ee_jointidx];
	points.push_back(desPos); joint_indices.push_back(ee_jointidx); weights.push_back(posWeight);

	//dir
	gVec3 normX = desRotX;  normls.push_back(normX); gVec3 normY = desRotY;  normls.push_back(normY);
	weights.push_back(dirWeight);

	//reference
	weights.push_back(refPosWeight);

	//update EEBox
	desEE->gV_EEs = points; desEE->gV_EEtarNs = normls;
	desEE->gV_EEints = joint_indices; desEE->gV_weight = weights;

	//link dir
	weights.push_back(linkDirWeight);
	desEE->gV_EEints_Dirs = desDirs_indices; desEE->gV_EEtarDirs = desDirs;

	if (chain->chain_name_idx == 0)
		TrunkIK_withDir(chain, desEE, character);
	if (chain->chain_name_idx == 1)
		rightArmIK(chain, desEE, character);
	if (chain->chain_name_idx == 2)
		leftArmIK(chain, desEE, character);
	if (chain->chain_name_idx == 3)
		rightLegIK(chain, desEE, character);
	if (chain->chain_name_idx == 4)
		leftLegIK(chain, desEE, character);
}

void mw_tiUtil_CHAIN::TrunkIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src)
{
	int m = chain->size - 2; // joint number - base orientation
	int dof = m * 3;
	//chain->lca_srccoord = getTrunkLimCompactCoordArray();
	
	//for(int i =0; i < dof; i++)
	//std::cout << " trunk coord " << lca_tr_srccoord[i] << std::endl;
	//lca_tr_base = character->link(0)->frame().trn();
	
	if (chain->b_first == true) {
		chain->lca_precoord = chain->lca_srccoord; // 미리 할당되어져야한다.
		chain->b_first = false; // 미리 할당되어져야한다.
	}
	////////ik function
	int sz = dof + EEBox->gV_EEs.size() * 3 + EEBox->gV_EEtarNs.size() * 3;// _iter_ViewFieldInfo->p_ends.size(); // dof:source pose, EEposition EEBox->gV_EEs.size() * 3
	dlevmar_dif(objFuncTrunkLimIK_Furniture, chain->lca_precoord, NULL, dof, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	printf("nIter=%d, reason=%d, e=%g to %g\n", int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0], LevMarInfo[1]);

	setTrunkLimCompactCoordArray(&sT_TrunkChain, character, chain->lca_precoord);

	character->updateKinematicsUptoPos();
	character->updateKinematicBodiesOfCharacterSim();
}

void mw_tiUtil_CHAIN::TrunkIK_withDir(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src)
{
	int m = chain->size - 2; // joint number - base orientation
	int dof = m * 3;
	//chain->lca_srccoord = getTrunkLimCompactCoordArray();

	//for(int i =0; i < dof; i++)
	//std::cout << " trunk coord " << lca_tr_srccoord[i] << std::endl;
	//lca_tr_base = character->link(0)->frame().trn();

	if (chain->b_first == true) {
		chain->lca_precoord = chain->lca_srccoord; // 미리 할당되어져야한다.
		chain->b_first = false; // 미리 할당되어져야한다.
	}
	////////ik function
	int sz = dof + EEBox->gV_EEs.size() * 3 + EEBox->gV_EEtarNs.size() * 3 + EEBox->gV_EEtarDirs.size() * 3;// _iter_ViewFieldInfo->p_ends.size(); // dof:source pose, EEposition EEBox->gV_EEs.size() * 3
	dlevmar_dif(objFuncTrunkLimIK_Furniture_withDir, chain->lca_precoord, NULL, dof, sz, 100, LevMarOpts, LevMarInfo, NULL, NULL, this);
	printf("nIter=%d, reason=%d, e=%g to %g\n", int(LevMarInfo[5]), int(LevMarInfo[6]), LevMarInfo[0], LevMarInfo[1]);

	setTrunkLimCompactCoordArray(&sT_TrunkChain, character, chain->lca_precoord);

	character->updateKinematicsUptoPos();
	character->updateKinematicBodiesOfCharacterSim();
}




mw_tiUtil_CHAIN::mw_tiUtil_CHAIN(bCharacter * cha)
{
	double stopThresh = 1e-2;
	LevMarOpts[0] = 1e-2;
	LevMarOpts[1] = stopThresh;
	LevMarOpts[2] = stopThresh;
	LevMarOpts[3] = stopThresh;
	LevMarOpts[4] = LM_DIFF_DELTA;

	//setting the bCharacter which doing IK
	character = cha;
	_iter_ViewFieldInfo = new iterViewInfo();
	_iter_BodsInfo = new iterBodsInfo();

	iterJntVec.resize(character->dof()); // for 
	defaultPose.resize(character->dof()); // for default pose

										  //initial value for chain inverse kinematics
	character->getCompactCoordArray(defaultPose); //get default pose
}

mw_tiUtil_CHAIN::~mw_tiUtil_CHAIN()
{
}
