#ifndef __RELOCALISER_H
#define __RELOCALISER_H

#include "Map.h"
#include "MapPoint.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Converter.h"
#include <lsh.hpp>
#include <TooN/wls.h>
#include <TooN/se2.h>


namespace ORB_SLAM2
{

//class Map;
//class MapPoint;
//class Frame;
//class KeyFrame;
//class BriefData;

class LSHRelocaliser
{
public:
    LSHRelocaliser(LSH<BriefData,uint32_t>* classifier)
    :mpClassifier(classifier)
    {}

    ~LSHRelocaliser()
    {
        delete mpClassifier;
    }


//private:
    LSH<BriefData,uint32_t>* mpClassifier; // classifier to use for search

};

class Relocaliser: public LSHRelocaliser
{
public:
  Relocaliser(Map* map,const LshParams& lshParams)
    : LSHRelocaliser(new LSH<BriefData,uint32_t>(mapPoint2BriefData(map->GetAllMapPoints(), mMapBDs),lshParams)),
      mpMap(map)
  {}

  ~Relocaliser()
  {}

  void AttemptRecovery(Frame *mCurrentFrame);
  void extractForFastCorners(Frame &kF, vector<MapPoint*>& mps);

  vector<BriefData*>& mapPoint2BriefData(const vector<MapPoint*>& mapMPs, vector<BriefData*>& mapBDs);
  vector<BriefData*> mapPoint2BriefData(const vector<MapPoint*>& mapMPs);

private:
  Map* mpMap;
  vector<BriefData*> mMapBDs;
//  Feature_t mFeatureType;
//  LSH<BriefData,uint32_t>* mpClassifier; // classifier to use for search


  //[m] within this radius keyframes are taken into consideration for building the list of points to match against
  constexpr static const double RELEVANCE_RADIUS=5.0;

};

}
#endif
