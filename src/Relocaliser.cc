#include "Relocaliser.h"

#include"ORBmatcher.h"
#include"Optimizer.h"
#include"PnPsolver.h"

namespace ORB_SLAM2
{

//class Relocaliser;

vector<BriefData*>& Relocaliser::mapPoint2BriefData(const vector<MapPoint*>& mapMPs, vector<BriefData*>& mapBDs)
{
  mapBDs.reserve(mapMPs.size());
  for(uint32_t i=0; i< mapMPs.size(); ++i)
    mapBDs.push_back(mapMPs[i]->pBData);
  return mapBDs;
}

vector<BriefData*> Relocaliser::mapPoint2BriefData(const vector<MapPoint*>& mapMPs)
{
  vector<BriefData*> mapBDs;
  mapPoint2BriefData(mapMPs, mapBDs);
  return mapBDs;
}

void Relocaliser::AttemptRecovery(Frame *mCurrentFrame)
{

    //    vector<MapPoint*> queryMPs;

    //    extractForFastCorners(mCurrentFrame,queryMPs);

    // find the set of mappoints which were observed in keyframes that are in
    // a circle of maxDist m around the current pose.
    mMapBDs.clear(); mMapBDs.reserve(mpMap->GetAllMapPoints().size()*2);
    map<KeyFrame*,bool> relevantKFs;
    int nKFs = 0;
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    const vector<MapPoint*> vpPoints = mpMap->GetAllMapPoints();

    Eigen::Matrix<double,3,1> curPos=Converter::toVector3d(mCurrentFrame->GetCameraCenter());

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        Eigen::Matrix<double,3,1> pos = Converter::toVector3d(pKF->GetCameraCenter());

        double dpos[] = {curPos(1)-pos(1),curPos(2)-pos(2),curPos(3)-pos(3)};
        double normpos = dpos[0]*dpos[0]+dpos[1]*dpos[1]+dpos[2]*dpos[2];

        if(sqrt(normpos) < RELEVANCE_RADIUS)
        {
            relevantKFs[pKF] = true;
            nKFs++;
        }
    }

    map<KeyFrame*,size_t>::iterator  iter;
//    vector<cv::Mat> Descriptor;

    for(size_t i=0; i<vpPoints.size(); ++i)
    {
        for(iter = vpPoints[i]->GetObservations().begin(); iter != vpPoints[i]->GetObservations().end(); iter++)
        {
            if(relevantKFs.find(iter->first) != relevantKFs.end())
//              Descriptor.push_back(vpPoints[i]->GetDescriptor());
                mMapBDs.push_back(vpPoints[i]->pBData);
        }
    }


    //test for I2Mrelocalise 20170207

//    ORBmatcher matcher(0.75,true);

//    vector<PnPsolver*> vpPnPsolvers;
//    vpPnPsolvers.resize(nKFs);



    // exchange the classifier with a version that has updated mapPoints!
//    LshParams lshParams = mpClassifier->getParams();
//    delete mpClassifier;
//    mpClassifier= new LSH<BriefData,uint32_t>(mMapBDs,lshParams);

//    vector<BriefData*> queryBDs;

//    mapPoint2BriefData(mCurrentFrame->mvpMapPoints, queryBDs);

//    vector<Assoc<BriefData,uint32_t> > pairing;

//    float tPrep = mpClassifier->prepare();

//    float tPair = dynamic_cast<Classifier<BriefData,uint32_t>* >(mpClassifier)->pair(queryBDs, pairing);
//    cout<<"Paired "<<queryBDs.size()<<" queries"<<endl;
//    uint32_t querySize = queryBDs.size();
//    uint32_t mapSize = mpClassifier->getMapDs().size();

}

//void Relocaliser::extractForFastCorners(Frame &kF, vector<MapPoint*>& mps)
//{
//    for(i=0; i<kF.N; i++)
//    {
//        BriefDataS* bData=new BriefDataS(mps[i]);
//        bData->mpKF = &kF;
//        bData->descriptorFromCvMatRow(kF.mDescriptors.data[i*kF.mDescriptors.cols]);
//        Vector<2> v2p;
//        v2p[0] = kF.mvKeys.data()->pt.x; v2p[1] = kF.mvKeys.data()->pt.y;
//        bData->v2p = v2p;
//        mps[i]->pBData = bData;
//    }
//}



}
