#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter_g2o.h"


class dc_optimizer{
	
public:
	dc_optimizer(vill::Map* pMap_):pMap(pMap_){}
	
	double optimize_BA_with_dc_error();
	
	vill::Map* pMap;
};