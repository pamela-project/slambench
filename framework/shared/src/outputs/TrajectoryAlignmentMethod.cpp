/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "outputs/TrajectoryAlignmentMethod.h"
#include "outputs/Output.h"

#include <vector>
#include <boost/optional.hpp>

#include <iostream>

using namespace slambench::outputs;

// TODO: this is duplicated in ATEMetric.cpp
static boost::optional<slambench::TimeStamp> select_closest_before (const TrajectoryAlignmentMethod::trajectory_t & GT , slambench::TimeStamp start_from , const slambench::TimeStamp& LO_TS) {
	
	if(GT.size() == 0) {
		std::cerr << "**** Error: Empty GT." << std::endl;
		return  boost::none;
	}

	auto gt_iterator = GT.begin();
	while(gt_iterator->first < start_from) {
		gt_iterator++;
	}
	
	if(gt_iterator == GT.end()) {
		std::cerr << "**** Error: No more GT to compare with." << std::endl;
		return boost::none;
	}
	
	/*
	 *  We iterate over GT.
	 *  If GT_TS happens after LO_TS we stop the loop.
	 *  It can be that the first GT_TS we found happens after, this means we should not compare with this one (skip long gap of empty GT).
	 *  Then closest is the first found, or the closest.
	 * */

	slambench::TimeStamp closest = gt_iterator->first;
	for (; gt_iterator != GT.end(); ++gt_iterator) {
		const slambench::TimeStamp & GT_TS  = gt_iterator->first;
		if ( GT_TS > LO_TS ) {
			break;
		}

		slambench::TimeStamp CL_TS  = gt_iterator->first;
		if (LO_TS - CL_TS > LO_TS - GT_TS) {
			closest = GT_TS;
		}
	}
	
	return closest;
}

Eigen::Matrix4f align_trajectories_original (const TrajectoryAlignmentMethod::trajectory_t & gt , const TrajectoryAlignmentMethod::trajectory_t & t) {

	Eigen::Matrix4f res = Eigen::Matrix4f::Identity();

	if (gt.size() != 0 and t.size() != 0){

		for (auto past_point : t) {
			auto closest = select_closest_before(gt,{0,0},past_point.first);
			if (closest) {

				res = ((gt.at(closest.get()))).GetValue() * past_point.second.GetValue().inverse();
				//std::cout << res << std::endl;
				return res ;
			}
		}

	}



	return res ;
}


bool associate(const TrajectoryAlignmentMethod::trajectory_t & gt,
          const TrajectoryAlignmentMethod::trajectory_t & t,
          std::vector<Eigen::Matrix4f>& vGroundTruth,
          std::vector<Eigen::Matrix4f>& vEstimate)
{

    for(unsigned int index = 0; index < t.size(); index++)  {

        vEstimate.push_back(t.at(index).second.GetValue());

        double time = t.at(index).first.S + (t.at(index).first.Ns)*std::pow(10,-9);
        //std::cout << t[index].second.S + (t[index].second.Ns)*std::pow(10,-9)  << std::endl;

        unsigned int    closest_gt_Index = index;

        double bestTimeDiff = 100000000000.0;
        for(unsigned int index2 = index; index2 < gt.size(); index2++)
        {
            double gt_time = gt.at(index2).first.ToS();
            double timeDiff =  gt_time - time;
            if(fabs(timeDiff) < bestTimeDiff)
            {
                closest_gt_Index = index2;
                bestTimeDiff = fabs(timeDiff);
            }
        }

        //std::cout <<  index << " " << closest_gt_Index <<  " " << bestTimeDiff << std::endl;
        //std::cout <<  t[index].second.S << " " << (t[index].second.Ns) << " " << gt[closest_gt_Index].second.S << " " << (gt[closest_gt_Index].second.Ns) << std::endl;

        Eigen::Matrix4f gtPose;
        gtPose = gt.at(closest_gt_Index).second.GetValue(); //<block-size>(start-i, start-j)
        vGroundTruth.push_back(gtPose);
    }
   return (bool)(vGroundTruth.size() * vEstimate.size());
}


Eigen::MatrixXd ATERotation(Eigen::MatrixXd model, Eigen::MatrixXd data)
{
    Eigen::MatrixXd w;
    w = Eigen::MatrixXd::Identity(3,3);

    int cols = model.cols();

    //Model = [M0, M1, ...], Data = [D0, D1, ...]
    // W = M0*D0' + M1*D1' + ...  = \sum{Mi*Di}
    for (int i = 0; i < cols; i++)
        w = w + model.col(i) * data.col(i).transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(w.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    float detV = V.determinant();
    float detU = U.determinant();

    Eigen::MatrixXd S;
    S = Eigen::MatrixXd::Identity(3,3);

    if(detU * detV < 0)
        S(2,2) = -1;

    Eigen::MatrixXd rot;
    rot = U * S * V.transpose();

    return rot;
}



double ATEScale(Eigen::MatrixXd model, Eigen::MatrixXd data, Eigen::MatrixXd rotation)
{
    int cols = model.cols();
    Eigen::MatrixXd rotatedModel;
    rotatedModel = rotation * model;

    double dots = 0.0;
    double norms = 0.0;
    double normi = 0.0;

    //Model = [M0, M1, ...], Rotated Data = [R0, R1, ...]
    // W = M0.D0' + M1.D1' + ...  = \sum{Mi.Di}
    for (int i = 0; i < cols; i++)
    {
    	Eigen::Vector3d v1 = data.col(i);
    	Eigen::Vector3d v2 = rotatedModel.col(i);
    	Eigen::Vector3d v3 = model.col(i);

       dots = dots + v1.transpose()*v2;
       normi = v3.norm();
       norms = norms + normi*normi;
    }

    // scale
    return  (dots/norms);
    //return 1;
}


Eigen::Vector3d ATETranslation(Eigen::MatrixXd model, Eigen::MatrixXd data, double scale, Eigen::MatrixXd rotation, float& ate)
{
    int N = model.cols();
    Eigen::Vector3d translation = data.rowwise().mean() - (scale*rotation)*(model.rowwise().mean());
    Eigen::MatrixXd rotatedModel(3,N);
    rotatedModel = (scale*rotation)*model;

    // error matrix E = [E1, E2, ...]
    Eigen::MatrixXd errorMat(3, N);
    errorMat = (rotatedModel.colwise() + translation) - data;

    // Absoute Trajectory Error (ATE) = |||E1|| + ||E2||+ ... = \sum(||Ei||)
    for (int i = 0; i < N; i++)
        ate = ate + errorMat.col(i).norm();

    ate = ate/N;
    return   translation;

}


Eigen::Matrix4d calculateATE(std::vector<Eigen::Matrix4f> gt, std::vector<Eigen::Matrix4f> es, float & ate)
{
    // convert pose vectors to Eigen matrices
    int N = gt.size();
    Eigen::MatrixXd gtMat(3,N);
    for (int i = 0; i < N; i++)
    {
        gtMat(0,i) = gt.at(i).block<3,1>(0,3)(0);
        gtMat(1,i) = gt.at(i).block<3,1>(0,3)(1);
        gtMat(2,i) = gt.at(i).block<3,1>(0,3)(2);
    }


    int M = gt.size();
    Eigen::MatrixXd esMat(3,N);
    for (int i = 0; i < M; i++)
    {
        esMat(0,i) = es.at(i).block<3,1>(0,3)(0);
        esMat(1,i) = es.at(i).block<3,1>(0,3)(1);
        esMat(2,i) = es.at(i).block<3,1>(0,3)(2);
    }


    // calculate the mean pose to zero-shift the poses
    Eigen::Vector3d gtMean = gtMat.rowwise().mean();
    Eigen::Vector3d esMean = esMat.rowwise().mean();

    Eigen::MatrixXd gtZeroMat(3,N);
    Eigen::MatrixXd esZeroMat(3,N);

    gtZeroMat = gtMat.colwise() - gtMean;
    esZeroMat = esMat.colwise() - esMean;

    // absoulte trajector error (ate)
    float error = 0;
    //bool bOk             = prepareData(gt,  es, gtZeroMat, esZeroMat);
    Eigen::Matrix3d rotation    = ATERotation(gtZeroMat, esZeroMat);
    double   scale       = ATEScale(gtZeroMat, esZeroMat, rotation);
    Eigen::Vector3d translation = ATETranslation(gtMat, esMat, scale, rotation, error);

    ate = error;

    Eigen::Matrix4d Mat;
    Mat = Eigen::Matrix4d::Identity();

    Mat.block<3,3>(0,0) = scale*rotation;
    Mat.block<3,1>(0,3) = translation;
    Mat.block<1,4>(3,0) << 0,0,0,1;

    return Mat;
}


Eigen::Matrix4f align_trajectories_new (const TrajectoryAlignmentMethod::trajectory_t & gt , const TrajectoryAlignmentMethod::trajectory_t & t) {

	if (t.size() <= 100){ // do not align for the first 100 frames, because it will not be accurate
		return align_trajectories_original(gt ,  t) ;
	}






    if (t.size() > 100) // do not align for the first 100 frames, because it will not be accurate
    {
        std::vector<Eigen::Matrix4f> vGroundTruth;
        std::vector<Eigen::Matrix4f> vEstimate;

        //if(associate(gt, t, vGroundTruth, vEstimate))
        //{
        associate(gt, t, vGroundTruth, vEstimate);
		
        float error;
        Eigen::Matrix4d Mat = calculateATE(vGroundTruth, vEstimate, error);

        //std::cout << " ------ " << error << std::endl;
        //if (error < 0.1)
        //{
            return Mat.cast<float>().inverse();
            //return Mat.cast<float>();
        //}
    }
    else
    {
    	return align_trajectories_original(gt ,  t) ;
   }
}

Eigen::Matrix4f NewTrajectoryAlignmentMethod::operator()(const TrajectoryAlignmentMethod::trajectory_t & ground_truth, const TrajectoryAlignmentMethod::trajectory_t & trajectory)
{
	return align_trajectories_new(ground_truth, trajectory);
}

Eigen::Matrix4f OriginalTrajectoryAlignmentMethod::operator()(const TrajectoryAlignmentMethod::trajectory_t & ground_truth, const TrajectoryAlignmentMethod::trajectory_t & trajectory)
{
	return align_trajectories_original(ground_truth, trajectory);
}

