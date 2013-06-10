#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class RegionGrowthAlg
{
#define REG_NO_ACCESS -1
#define REG_CAN_NOT_ACCESS -2
#define REG_MAX_DST 100000

public:
	struct Node
	{
		public:
		int id;
		int dst[4];
		int minDst;
		int meanDst;
		int dstNum;
		bool isInCandidates;
		Node()
		{}
		Node(int regionId, int regionNumX, int regionNumY)
		{
			int posX = regionId % regionNumX;
			int posY = regionId / regionNumX;
			id = regionId;
			minDst = REG_MAX_DST;
			dst[0] = posY == 0 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS;
			dst[1] = posX == regionNumX - 1 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS;
			dst[2] = posY == regionNumY - 1 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS;
			dst[3] = posX == 0 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS;
			isInCandidates = false;
		}
	};
public:
	RegionGrowthAlg(void);
	~RegionGrowthAlg(void);
	void calcRegionMap(Mat img, Mat_<int> regionMap, int regionLen = 16, int featureLen = 16);
	int calcDstOfFeature(vector<int> feature1, vector<int> feature2);
	Node createNode(int regionId, int regionNumX, int regionNumY);
	void testDisplayDst(void);
	Mat drawRegionNodeMap(Mat img, vector<Node> regionNodeMap, int regionLen, float zoomRate = 4);
private:
	vector<Point2i> id2point;
	Mat_<int> point2id;
	vector<int> directionMap;
	int getRelateRegionId(int regionId, int direction);
	int regionNumX;
	int regionNumY;
public:
	void showCandidates(list<int> candidates, vector<Node> regionNodeMap);
};

