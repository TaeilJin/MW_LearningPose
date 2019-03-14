#ifndef _H_mw_CHAIN_
#define _H_mw_CHAIN_

/*
*	[ mw_tiUtil_IK ]
*
*	TYPE		: IMPLEMENTATION
*	AUTHOR		: Taeil Jin
*	LAST UPDATE	: 2017 / 11 / 17
*	DESCRIPTION	:
*	this IK solver generate full body pose which designed to MBS created by S.H. Lee.
*
*/
using namespace std;
#include "Character/bCharacter.h"
#include "Character/bCharacterSolver.h"
#include "mbs/gArmaUtil.h"
#include "levmar.h"

#pragma once
class mw_tiUtil_CHAIN
{
public:
	//* Variables *//

	bCharacter* character; //character which applied Inverse Kinematics

	arma::vec defaultPose; //desired pose used to drive IK to natural human pose

	arma::vec iterJntVec; //get current joint vector ( 6 (base_angle+position) + N(angle))
	
	//end-effector set
	struct sEEBox {
		std::vector<int> gV_EEints; //target positions std vector
		std::vector<float> gV_weight; // target weight
		std::vector<gVec3> gV_EEs; //target positions std vector
		std::vector<gVec3> gV_EEtarNs; //target normals std vector

		std::vector<int> gV_EEints_Dirs; // target link indices
		std::vector<gVec3> gV_EEtarDirs; // target link direction std vector
	};
	sEEBox sR_ArmDesiredEE; sEEBox sL_ArmDesiredEE;
	sEEBox sR_LegDesiredEE; sEEBox sL_LegDesiredEE;
	sEEBox sT_TrkDesiredEE;
	//* Function *//
	//get current character output
	bCharacter* getCha() {
		return character;
	};

	//get character body length
	double getChainBodyLength(intArray chain, int startIdx,int endIdx) {
		//calculating body length
		double d_leng = 0.0;
		for (int p = startIdx; p < endIdx-1; p++) {
			int jointIdx = chain[p];

			//std::cout << " name 1 " << character->link(jointIdx + 1)->name() << " name 2 " << character->link(jointIdx)->name() << std::endl;
			gVec3 leng = (character->link(jointIdx + 1)->frame().trn() - character->link(jointIdx)->frame().trn());
			d_leng += leng.magnitude();
		}

		return d_leng;
	};
	
	struct sBodyChain {
		int size; // body chain joint size;
		int chain_name_idx; // allocated body chain name idx;
		std::vector<gAttachedPoint> AttachedPoints;//points in chain joints
		std::vector<gAttachedPoint> AttachedPointsForObjFunc;// points for optimization method
		std::vector<int> JointIndexs; // joint index of total joint coordinates

		intArray chain_idx; // chain indice in MBS character
		double totalchainLength;// total bone length ( it may calculate in the T-pose);

		std::vector<gAttachedPoint> tarCollid; // chain tarcollid for finding obstacle avoidance
		std::vector<bool> isContact; // bool whether point is contact or not
		//std::vector<double> boneLength; //bone length
		//std::vector<gVec3> boneVector; // bone vector

		double* lca_precoord; 
		double* lca_srccoord;
		bool b_first;

	};
	sBodyChain sR_ArmChain; //sR_ArmChain; 
	sBodyChain sL_ArmChain; //sL_ArmChain;
	
	sBodyChain sR_LegChain; //sR_LegChain;
	sBodyChain sL_LegChain; //sL_LegChain;

	sBodyChain sT_TrunkChain; //sTrunkChain;

	void initChainInfo(sBodyChain& sChain, int name_idx, intArray ia_chain, gVec3 EEVec, int EEidx, bool b_hasEEpos) {
		sChain.size = ia_chain.size() + 1;// root joint 부터의 bone 은 사용하지 않지만(-1) EE 까지의 bone을 만들기 때문에 (+1) MBS 의 chain 갯수와 같다.
		sChain.chain_idx = ia_chain;
		sChain.isContact.resize(sChain.size);
		sChain.chain_name_idx = name_idx;
		//updating the chain information
		for (int p = 0; p < ia_chain.size(); p++) { // EE 에 대한 information 까지 포함하여만든다.  root에서부터는 빼고
			sChain.isContact[p] = false;
			if (p == ia_chain.size() - 1) //last joint we make end effector 
			{
				//last joint attached point
				int jntIdx = ia_chain[p];
				sChain.JointIndexs.push_back(jntIdx); //get joint index information
				gLink* joint = character->link(jntIdx); // get joint information
				gAttachedPoint ps(joint, gVec3(0, 0, 0)); // Attached points into joint
				sChain.AttachedPoints.push_back(ps); // get Attached points
				sChain.AttachedPointsForObjFunc.push_back(ps); // get Attached points for optimzation process

				//ee attached point
				if (b_hasEEpos == true){
					//last joint attached point
					sChain.JointIndexs.push_back(EEidx); //get joint index information
					gLink* joint = character->link(EEidx); // get joint information
					gAttachedPoint ps(joint, gVec3(0, 0, 0)); // Attached points into joint
					sChain.AttachedPoints.push_back(ps); // get Attached points
					sChain.AttachedPointsForObjFunc.push_back(ps); // get Attached points for optimzation process
				}
				else {
					int eeJoint = ia_chain[p];
					sChain.JointIndexs.push_back(eeJoint);
					joint = character->link(eeJoint);
					EEVec = joint->frame().multVec3(EEVec);
					gAttachedPoint EEs(joint, EEVec);
					sChain.AttachedPoints.push_back(EEs); // put a point into the hand joint
					sChain.AttachedPointsForObjFunc.push_back(EEs); // put a point into the joints this points are used for optimzation process

					//sChain.isContact[p + 1] = false;
				}
			}
			else {
				int jntIdx = ia_chain[p];
				sChain.JointIndexs.push_back(jntIdx); //get joint index information
				gLink* joint = character->link(jntIdx); // get joint information
				gAttachedPoint ps(joint, gVec3(0, 0, 0)); // Attached points into joint
				sChain.AttachedPoints.push_back(ps); // get Attached points
				sChain.AttachedPointsForObjFunc.push_back(ps); // get Attached points for optimzation process
			}
			
		}

		//updating the IK  chain information
		int nSize = (sChain.size - 1) * 3;  sChain.lca_precoord = new double[nSize]; sChain.lca_srccoord = new double[nSize]; sChain.b_first = true;
	};
	
	

	//get compact coordinate vector for chain
	arma::vec getChainCompactCoord(arma::vec v_FullVec, intArray ia_chain); 
	arma::vec getTrunkChainCompactCoord(arma::vec v_FullVec, intArray ia_chain);

	//set the new coordinates into full joint coordinates
	void setChainJointFromCompactArray(arma::vec& v_FullVec, intArray ia_chain, arma::vec chainVec) {
		int cnt = 0;
		for (int tem = 0; tem < ia_chain.size(); tem++) {
			int jointIdx = ia_chain[tem];
			int vectorIdx = 3 * jointIdx + 3;
			
			v_FullVec[vectorIdx] = chainVec[cnt++];
			v_FullVec[vectorIdx + 1] = chainVec[cnt++];
			v_FullVec[vectorIdx + 2] = chainVec[cnt++];
		}
		character->setFromCompactCoordArray(v_FullVec);
	};
	
	void setTrunkChainJointFromCompactArray(arma::vec& v_FullVec, intArray ia_chain, arma::vec chainVec) {
		
		v_FullVec[0] = chainVec[0];
		v_FullVec[1] = chainVec[1];
		v_FullVec[2] = chainVec[2];
		v_FullVec[3] = chainVec[3];
		v_FullVec[4] = chainVec[4];
		v_FullVec[5] = chainVec[5];

		int cnt = 6;
		for (int tem = 1; tem < ia_chain.size(); tem++) {
			int jointIdx = ia_chain[tem];
			int vectorIdx = 3 * jointIdx + 3;

			v_FullVec[vectorIdx] = chainVec[cnt++];
			v_FullVec[vectorIdx + 1] = chainVec[cnt++];
			v_FullVec[vectorIdx + 2] = chainVec[cnt++];
		}
		character->setFromCompactCoordArray(v_FullVec);
	};
	//get chain length (root to EndEffector)
	double getChainBodyLength(sBodyChain chain, int startIdx, int endIdx) {
		//calculating body length
		double d_leng = 0.0;
		for (int p = startIdx; p < endIdx; p++) {
			//std::cout << " name 1 " << character->link(jointIdx + 1)->name() << " name 2 " << character->link(jointIdx)->name() << std::endl;
			gVec3 leng = (chain.AttachedPoints[p + 1].posWorld() - chain.AttachedPoints[p].posWorld());
			d_leng += leng.magnitude();
		}

		return d_leng;
	};

	
	//degree to radian
	double degree2radian(double degree) {
		double radian = degree * M_PI / 180;
		return radian;
	};
	//degree to radian
	double radian2degree(double radian) {
		double degree = radian * 180 / M_PI;
		return degree;
	};
	//set the desired normal vector for endeffector
	void setChainDesiredNormalVector(sBodyChain chain, gVec3& gV_BoneVec, gVec3& gV_NormalVec) {
		int nChain = chain.size;
		gV_BoneVec = chain.AttachedPoints[nChain - 1].posWorld() - chain.AttachedPoints[nChain - 2].posWorld();
		gV_BoneVec.normalize();
		
		gV_NormalVec = chain.AttachedPoints[nChain - 1].body()->frame().multVec3(gVec3(0, 1.0, 0));
		gV_NormalVec.normalize();
	};

	/*
	*
	*Contact Point Descriptor
	*
	*/
	class cpinf
	{
	public:
		cpinf() {
			_contactForce.setZero();
		}

		cpinf(gVec3 pos, gVec3 nor, int index)
		{
			setPos(pos);
			setNormal(nor);
			setPartIndex(index);
			_contactForce.setZero();
			_contact = false;
		}

		cpinf(gVec3 pos) { setPos(pos); }

		void	setPos(gVec3 pos) { _pos = pos; }
		gVec3	getPos() { return _pos; }

		gVec3	getNormal() { return _normal; }
		void	setNormal(gVec3 nor) { nor.normalize(); _normal = nor; }

		gVec3	getContactForce() { return _contactForce; }
		void	setContactForce(gVec3 force) { _contactForce = force; }

		void	setPartIndex(int i) { _partindex = i; }
		int		getPartIndex() { return _partindex; }

		void	setCPIndex(int i) { _cpindex = i; }
		int		getCPIndex() { return _cpindex; }

		void	setSupport_eps(double eps) { _eps = eps; }
		double  getSupport_eps() { return _eps; }

		bool	isContact() { return _contact; }
		void	setContact(bool con) { _contact = con; }

	private:
		bool	_contact;

		gVec3	_contactForce;  //Added by SHL (2013-4-24)
		gVec3	_pos;
		gVec3	_normal;
		int		_partindex;
		int		_cpindex;

		double  _eps; //Added by Taeil (2018.04.22) 
	};

	std::vector<cpinf*> sContactDescript;

	
	
	//do Inverse Kinematics
	/* LevMarOpts: opts[0-4] = minim. options [\tau, \epsilon1, \epsilon2, \epsilon3, \delta]. Respectively the
	//                    * scale factor for initial \mu, stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2 and the
	//                    * step used in difference approximation to the Jacobian. If \delta<0, the Jacobian is approximated
	//                    * with central differences which are more accurate (but slower!) compared to the forward differences
	//                    * employed by default. Set to NULL for defaults to be used.
	//                    */
	double LevMarOpts[5]; //option for levmar


						  //                    /* O: information regarding the minimization. Set to NULL if don't care
						  //                     * info[0]= ||e||_2 at initial p.
						  //                     * info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, \mu/max[J^T J]_ii ], all computed at estimated p.
						  //                     * info[5]= # iterations,
						  //                     * info[6]=reason for terminating: 1 - stopped by small gradient J^T e
						  //                     *                                 2 - stopped by small Dp
						  //                     *                                 3 - stopped by itmax
						  //                     *                                 4 - singular matrix. Restart from current p with increased \mu
						  //                     *                                 5 - no further error reduction is possible. Restart with increased mu
						  //                     *                                 6 - stopped by small ||e||_2
						  //                     *                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values; a user error
						  //                     * info[7]= # function evaluations
						  //                     * info[8]= # Jacobian evaluations
						  //                     * info[9]= # linear systems solved, i.e. # attempts for reducing error
						  //                     */
	double LevMarInfo[LM_INFO_SZ]; //performance report from levmar is stored here.

	bool checkReachable(intArray ia_chain, int startIdx, int endIdx, gVec3 tarpos);
	bool checkReachable(sBodyChain s_chainBox, int startIdx, int endIdx, gVec3 tarpos);

	void updateChain();
	double* getLimCompactCoordArray(mw_tiUtil_CHAIN::sBodyChain* chain) {
		
		int jointsize = chain->size - 1; //except end effector
		double* lca = new double[jointsize * 3]; // dof;
		for (int p = 0; p < jointsize; p++) {
			
			double* joint_lca = new double[3];
			character->link(chain->JointIndexs[p])->getCompactCoordArray(joint_lca);
			lca[3 * p + 0] = joint_lca[0]; lca[3 * p + 1] = joint_lca[1]; lca[3 * p + 2] = joint_lca[2];
		}
		return lca;
	}
	double* getTrunkLimCompactCoordArray() {

		cout << " trunk size " << sT_TrunkChain.size << endl;
		int jointsize = sT_TrunkChain.size - 1; //except end effector
		cout << " joint size " << jointsize << endl;

		double* lca = new double[jointsize * 3]; // dof;
		for (int p = 1; p < jointsize; p++) {

			double* joint_lca = new double[3];
			if (sT_TrunkChain.JointIndexs[p] == 0){
				joint_lca = new double[6];
				character->link(sT_TrunkChain.JointIndexs[p])->getCompactCoordArray(joint_lca);

				lca[3 * p + 0] = joint_lca[0]; lca[3 * p + 1] = joint_lca[1]; lca[3 * p + 2] = joint_lca[2];
				lca[3 * p + 3] = joint_lca[3]; lca[3 * p + 4] = joint_lca[4]; lca[3 * p + 5] = joint_lca[5];
			}
			else {
				character->link(sT_TrunkChain.JointIndexs[p])->getCompactCoordArray(joint_lca);
				lca[3 * (p - 1) + 0] = joint_lca[0]; lca[3 * (p - 1) + 1] = joint_lca[1]; lca[3 * (p - 1) + 2] = joint_lca[2];
			}
		}
		return lca;
	}
	void setSrcCoordFromRefCoord(mw_tiUtil_CHAIN::sBodyChain* chain, arma::vec lca) {

		int jointsize = chain->size - 1; //except end effector
		for (int p = 0; p < jointsize; p++) {
			
			int id_j = chain->JointIndexs[p];
			chain->lca_srccoord[3 * p + 0] = lca((3 * id_j + 3) + 0);
			chain->lca_srccoord[3 * p + 1] = lca((3 * id_j + 3) + 1);
			chain->lca_srccoord[3 * p + 2] = lca((3 * id_j + 3) + 2);
			
		}
	}
	void setLimCompactCoordArray(mw_tiUtil_CHAIN::sBodyChain* chain, bCharacter* tar, double* lca) {
		
		int jointsize = chain->size - 1; //except end effector
		for (int p = 0; p < jointsize; p++) {
			
			double* joint_lca = new double[3];
			joint_lca[0] = lca[3 * p + 0]; joint_lca[1] = lca[3 * p + 1]; joint_lca[2] = lca[3 * p + 2];
			character->link(chain->JointIndexs[p])->setFromCompactCoordArray(joint_lca);
		}
	}
	void setTrunkSrcCoordFromRefCoord(mw_tiUtil_CHAIN::sBodyChain* chain, arma::vec lca) {

		int jointsize = chain->size - 1; //except end effector
		for (int p = 1; p < jointsize; p++) {
			double* joint_lca = new double[3];
			chain->lca_srccoord[3 * (p - 1) + 0] = lca[3 * (sT_TrunkChain.JointIndexs[p] + 3) + 0];
			chain->lca_srccoord[3 * (p - 1) + 1] = lca[3 * (sT_TrunkChain.JointIndexs[p] + 3) + 1];
			chain->lca_srccoord[3 * (p - 1) + 2] = lca[3 * (sT_TrunkChain.JointIndexs[p] + 3) + 2];
			
		}
	}
	void setTrunkLimCompactCoordArray(mw_tiUtil_CHAIN::sBodyChain* chain, bCharacter* tar, double* lca) {

		int jointsize = chain->size - 1; //except end effector
		for (int p = 1; p < jointsize; p++) {
				double* joint_lca = new double[3];
				joint_lca[0] = lca[3 * (p-1) + 0]; joint_lca[1] = lca[3 * (p - 1) + 1]; joint_lca[2] = lca[3 * (p - 1) + 2];
				character->link(sT_TrunkChain.JointIndexs[p])->setFromCompactCoordArray(joint_lca);
		}
	}

	void TrunkIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src);
	void TrunkIK_withDir(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * EEBox, bCharacter * src);
	void rightLegIK(mw_tiUtil_CHAIN::sBodyChain* chain, mw_tiUtil_CHAIN::sEEBox* EEBox, bCharacter * src);
	void leftLegIK(mw_tiUtil_CHAIN::sBodyChain* chain, mw_tiUtil_CHAIN::sEEBox* EEBox, bCharacter * src);
	void rightArmIK(mw_tiUtil_CHAIN::sBodyChain* chain, mw_tiUtil_CHAIN::sEEBox* EEBox, bCharacter * src);
	void leftArmIK(mw_tiUtil_CHAIN::sBodyChain* chain, mw_tiUtil_CHAIN::sEEBox* EEBox, bCharacter * src);

	void doChainIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * desEE, float refPosWeight,
		gVec3 desPos, float posWeight, gVec3 desRotX, gVec3 desRotY, float dirWeight);
	void doChainIK(mw_tiUtil_CHAIN::sBodyChain * chain, mw_tiUtil_CHAIN::sEEBox * desEE, 
		float refPosWeight, gVec3 desPos, float posWeight, gVec3 desRotX, gVec3 desRotY, float dirWeight,
		std::vector<gVec3> desDirs, std::vector<int> desDirs_indices, float linkDirWeight);


	struct iterViewInfo{
		int tip_ji;
		gVec3 p_tip; // written by "Head" joint
		std::vector<gVec3> p_ends; // written by "Head" joint
		std::vector<gVec3> dir_tip2ends; // written by "Head" joint

		int nCastedNum;
		double desired_dist_mean;


		std::vector<double> desired_dist;
		std::vector<double> weight_Dist;
	};
	iterViewInfo* _iter_ViewFieldInfo;
	struct iterBodsInfo {
		std::vector<int> joint_id; // joint index
		std::vector<gVec3> p_ends; // written by "that joint"
		std::vector<gVec3> dir_tip2ends; // written by "Head" joint
		std::vector<gVec3> src_pJoint; // joint position
		std::vector<double> desired_dist;
	};
	iterBodsInfo* _iter_BodsInfo;
	




	void initColCheck(int dividedNum, sBodyChain& BodyChain)
	{
		using namespace std;

		for (int i = 0; i < BodyChain.chain_idx.size(); i++) {

			// getting link vector
			int chainIdx = BodyChain.chain_idx[i];
			gXMat shapeXform = character->getOrCreateVisShapeSet()->getShapeXform(character->link(chainIdx)->name());
			gVec3 p_shapePosWorld = character->link(chainIdx)->frame().multVec4(shapeXform.trn());
			gVec3 p_shapeJointWorld = character->link(chainIdx)->frame().trn();

			//get unit Vector, link length
			gVec3 v_link = p_shapePosWorld - p_shapeJointWorld;  //tar->link(k)->frame().trn() - tar->link(k - 1)->frame().trn();
			double length = v_link.magnitude();
			gVec3 v_unitLink; v_unitLink.setX(v_link.x() / length); v_unitLink.setY(v_link.y() / length); v_unitLink.setZ(v_link.z() / length);
			length = length * 2.0;


			// divided length (dl)
			double d_l = length / (dividedNum);

			// making sampled collision check points
			gVec3 p_divide;
			if (chainIdx == 0) {
				//-- hip capsule
				for (int k = 1; k < 5; k++) {

					if (k < 3) {
						v_unitLink.set(1, 0, 0);

						p_divide.setX(p_shapePosWorld.x() + v_unitLink.x() * d_l * k);
						p_divide.setY(p_shapePosWorld.y() + v_unitLink.y() * d_l * k);
						p_divide.setZ(p_shapePosWorld.z() + v_unitLink.z() * d_l * k);
					}
					else {
						v_unitLink.set(-1, 0, 0);

						p_divide.setX(p_shapePosWorld.x() + v_unitLink.x() * d_l * (k - 2));
						p_divide.setY(p_shapePosWorld.y() + v_unitLink.y() * d_l * (k - 2));
						p_divide.setZ(p_shapePosWorld.z() + v_unitLink.z() * d_l * (k - 2));
					}

					p_divide = character->link(chainIdx)->frame().invMultVec4(p_divide);
					gAttachedPoint psA(&*character->link(chainIdx), p_divide);
					BodyChain.tarCollid.push_back(psA);

				}
				//---
			}
			else {
				//-- another case
				for (int t = 0; t < dividedNum; t++) {

					p_divide.setX(p_shapeJointWorld.x() + v_unitLink.x() * d_l*t);
					p_divide.setY(p_shapeJointWorld.y() + v_unitLink.y() * d_l*t);
					p_divide.setZ(p_shapeJointWorld.z() + v_unitLink.z() * d_l*t);

					p_divide = character->link(chainIdx)->frame().invMultVec4(p_divide);
					gAttachedPoint psD(&*character->link(chainIdx), p_divide);
					BodyChain.tarCollid.push_back(psD);

				}
				//---
			}


		}


	};
	

	mw_tiUtil_CHAIN(bCharacter * cha);
	~mw_tiUtil_CHAIN();

};


#endif // !_H_mw_TIUTIL_IK_


