#include "RegionGrowthAlg.h"
#include <set>
#include <queue>

RegionGrowthAlg::RegionGrowthAlg(void)
	: regionNumX(0)
	, regionNumY(0)
{
}

RegionGrowthAlg::~RegionGrowthAlg(void)
{
}

void RegionGrowthAlg::calcRegionMap(Mat img, Mat_<int> regionMap, int regionLen /*= 16*/, int featureLen /*= 16*/)
{
	cvtColor(img, img, CV_RGB2GRAY);
	int imgRows = img.rows;
	int imgCols = img.cols;
	regionNumY = imgRows / regionLen;
	regionNumX = imgCols / regionLen;
	int marginY = imgRows % regionLen;
	int regionNum = regionNumX * regionNumY;
	Rect rectOfHandle(0, marginY, regionNumX * regionLen, regionNumY * regionLen);
	img = img(rectOfHandle);
	regionMap = regionMap(rectOfHandle);

	//��Ŷ�Ӧ��
	id2point.resize(regionNum);
	point2id.create(regionNumY, regionNumX); // (row, col)
	for (int i = 0; i < regionNum; i++)
	{
		id2point[i] = Point2i(i % regionNumX, i / regionNumX);
	}
	for (int i = 0; i < regionNumY; i++)
	{
		int* pRow = point2id.ptr<int>(i);
		for (int j = 0; j < regionNumX; j++)
		{
			pRow[j] = i * regionNumX + j;
		}
	}

	//relate node map table
	directionMap.resize(4);
	directionMap[0] = 2;
	directionMap[1] = 3;
	directionMap[2] = 0;
	directionMap[3] = 1;
	
	//calc histograms as features and mean bright
	vector< vector<int> > features(regionNum);
	vector<float> regionBrights(regionNum);
	for (int i = 0; i < regionNum; i++)
	{
		Point2i pos = id2point[i];
		int loopYStart = pos.y * regionLen;
		int loopYEnd = loopYStart + regionLen;
		int loopXStart = pos.x * regionLen;
		int loopXEnd = loopXStart + regionLen;
		vector<int> feature(featureLen, 0);
		float regionBright = 0;
		for (int j = loopYStart; j < loopYEnd; j++)
		{
			uchar* pRow = img.ptr<uchar>(j);
			for (int k = loopXStart; k < loopXEnd; k++)
			{
				feature[pRow[k] / featureLen]++;
				regionBright += pRow[k];
			}
		}
		features[i] = feature;
		regionBrights[i] = regionBright / (regionLen * regionLen);
		assert(regionBrights[i] >= 0 && regionBrights[i] < 256);
	}

	// init region node map
	vector<Node> regionNodeMap(regionNum);
	for (int i = 0; i < regionNumY; i++)
	{
		for (int j = 0; j < regionNumX; j++)
		{
			int id = i * regionNumX + j;
			regionNodeMap[id].id = id;
			regionNodeMap[id].minDst = REG_MAX_DST;
			regionNodeMap[id].dstNum = 0;
			regionNodeMap[id].dst[0] = regionNodeMap[id].dst[1] = regionNodeMap[id].dst[2] = regionNodeMap[id].dst[3] = REG_NO_ACCESS;
			regionNodeMap[id].isInCandidates = false;
		}
	}
	for (int i = 0; i < regionNumX; i++)
	{
		int bottom = regionNumY - 1;
		regionNodeMap[point2id(0, i)].dst[0] = REG_CAN_NOT_ACCESS;
		regionNodeMap[point2id(bottom, i)].dst[2] = REG_CAN_NOT_ACCESS;
	}
	for (int i = 0; i < regionNumY; i++)
	{
		int right = regionNumX - 1;
		regionNodeMap[point2id(i, 0)].dst[3] = REG_CAN_NOT_ACCESS;
		regionNodeMap[point2id(i, right)].dst[1] = REG_CAN_NOT_ACCESS;
	}
	
#ifdef _DEBUG
	for (int i = 0; i < regionNumY; i++)
	{
		for (int j = 0; j < regionNumX; j++)
		{
			assert(regionNodeMap[point2id(i, j)].dst[0] == (i == 0 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS));
			assert(regionNodeMap[point2id(i, j)].dst[1] == (j == regionNumX - 1 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS));
			assert(regionNodeMap[point2id(i, j)].dst[2] == (i == regionNumY - 1 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS));
			assert(regionNodeMap[point2id(i, j)].dst[3] == (j == 0 ? REG_CAN_NOT_ACCESS : REG_NO_ACCESS));
		}
	}
#endif // _DEBUG

	//��ʼ��groundSet
	set<int> groundSet;
	for (int i = 0; i < regionNum; i++)
	{
		Point2i pos = id2point[i];
		int loopYStart = pos.y * regionLen;
		int loopYEnd = loopYStart + regionLen;
		int loopXStart = pos.x * regionLen;
		int loopXEnd = loopXStart + regionLen;
		int flag = 0;
		for (int j = loopYStart; j < loopYEnd; j++)
		{
			int* pRow = regionMap.ptr<int>(j);
			for (int k = loopXStart; k < loopXEnd; k++)
			{
				flag += pRow[k];
			}
		}
		if (flag > 0)
		{
			groundSet.insert(i);
		}
	}
	
	//��ʼ��candidates
	int regionThreshold = 200;
	list<int> candidates;
	for (int i = 0; i < regionNum; i++)
	{
		if (groundSet.find(i) == groundSet.end()) // foreach region in groundSet
		{
			continue;
		}
		for (int j = 0; j < 4; j++)
		{
			if (regionNodeMap[i].dst[j] == REG_CAN_NOT_ACCESS) // foreach relate region exist
			{
				continue;
			}
			int relateRegionId = getRelateRegionId(i, j);
			Node& node = regionNodeMap[relateRegionId]; // find the 0 direction node
			if (groundSet.find(node.id) == groundSet.end()) // nodeI is not in groundSet
			{
				int dst = node.dst[directionMap[j]] = calcDstOfFeature(features[i], features[node.id]);
				node.dstNum++;
				//node.minDst = dst < node.minDst ? dst : node.minDst;
				if (dst > regionThreshold)
				{
					node.minDst = dst;
				}
				else if (node.isInCandidates) // node in candidates
				{
					if (dst < node.minDst) // need to reorder the candidate list
					{
						node.minDst = dst;
						if (candidates.size() > 1) // candidate more then 1
						{
							candidates.remove(node.id);
							for (list<int>::iterator it = candidates.begin(); it != candidates.end();)
							{
								if (regionNodeMap[*it].minDst > node.minDst)
								{
									candidates.insert(it, node.id);
									break;
								}
								if (++it == candidates.end())
								{
									candidates.push_back(node.id);
								}
							}
						}
						else {} // do nothing
					}
					else {} // do nothing
				}
				else // node not in candidates
				{
					node.minDst = dst;
					node.isInCandidates = true;
					if (candidates.size() > 0)
					{
						for (list<int>::iterator it = candidates.begin(); it != candidates.end();)
						{
							if (regionNodeMap[*it].minDst > node.minDst)
							{
								candidates.insert(it, node.id);
								break;
							}
							if (++it == candidates.end())
							{
								candidates.push_back(node.id);
							}
						}
					}
					else
					{
						candidates.push_front(node.id);
					}
				}
			}
			else {} // do nothing
		}
	}

	// Calculate meanBright from groundSet
	float meanBright = 0;
	double tmpBright = 0;
	for (set<int>::iterator it = groundSet.begin(); it != groundSet.end(); it++)
	{
		tmpBright += regionBrights[*it];
	}
	meanBright = tmpBright / groundSet.size();

	//showCandidates(candidates, regionNodeMap);
	// Region growth processing
	float brightThreshold = 50;
	while (candidates.size() > 0)
	{
		int candidateId = candidates.front();
		int groundSetSize = groundSet.size();
		meanBright = meanBright * (groundSetSize / (groundSetSize + 1.0)) + regionBrights[candidateId] / (groundSetSize + 1.0);
		candidates.pop_front();
		groundSet.insert(candidateId);
		for (int i = 0; i < 4; i++)
		{
			if (regionNodeMap[candidateId].dst[i] == REG_NO_ACCESS)
			{
				int relateRegionId = getRelateRegionId(candidateId, i);
				Node& node = regionNodeMap[relateRegionId]; // find the 0 direction node
				int dst = node.dst[directionMap[i]] = calcDstOfFeature(features[candidateId], features[node.id]);
				node.dstNum++;
				if (dst > regionThreshold || abs(meanBright - regionBrights[node.id]) > brightThreshold)
				{
					node.minDst = dst;
				}
				else if (node.isInCandidates) // node in candidates
				{
					if (dst < node.minDst) // need to reorder the candidate list
					{
						node.minDst = dst;
						if (candidates.size() > 1) // candidate more then 1
						{
							candidates.remove(node.id);
							for (list<int>::iterator it = candidates.begin(); it != candidates.end();)
							{
								if (regionNodeMap[*it].minDst > node.minDst)
								{
									candidates.insert(it, node.id);
									break;
								}
								if (++it == candidates.end())
								{
									candidates.push_back(node.id);
								}
							}
						}
						else {} // do nothing
					}
					else {} // do nothing
				}
				else // node not in candidates
				{
					node.minDst = dst;
					node.isInCandidates = true;
					if (candidates.size() > 0)
					{
						for (list<int>::iterator it = candidates.begin(); it != candidates.end();)
						{
							if (regionNodeMap[*it].minDst > node.minDst)
							{
								candidates.insert(it, node.id);
								break;
							}
							if (++it == candidates.end())
							{
								candidates.push_back(node.id);
							}
						}
					}
					else
					{
						candidates.push_front(node.id);
					}
				}
			}
		}
	}
	
	// refine groundSet
	Mat_<uchar> regionMat(regionNumY, regionNumX);
	Mat_<uchar> regionMat2(regionNumY, regionNumX);
	regionMat.setTo(0);
	for (set<int>::iterator it = groundSet.begin(); it != groundSet.end(); it++)
	{
		Point2i pos = id2point[*it];
		regionMat(pos.y, pos.x) = 1;
	}
	Mat kernel(3, 3, CV_32F, cv::Scalar(0));
	kernel.setTo(1);
	filter2D(regionMat, regionMat2, img.depth(), kernel);
	for (int i = 0; i < regionNumY; i++)
	{
		uchar* pRow = regionMat.ptr(i);
		uchar* pRow2 = regionMat2.ptr(i);
		for (int j = 0; j < regionNumX; j++)
		{
			if (pRow[j] == 0 && pRow2[j] > 4)
			{
				pRow[j] = 1;
				//groundSet.insert(point2id(i, j));
			}
		}
	}
	for (int i = regionNumY - 1; i > 0; i--)
	{
		uchar* pUpRow = regionMat.ptr(i - 1);
		uchar* pRow = regionMat.ptr(i);
		for (int j = 0; j < regionNumX; j++)
		{
			if (pUpRow[j] > 0 && pRow[j] == 0)
			{
				pUpRow[j] == 0;
			}
		}
	}
	groundSet.clear();
	for (int i = 0; i < regionNumY; i++)
	{
		uchar* pRow = regionMat.ptr(i);
		for (int j = 0; j < regionNumX; j++)
		{
			if (pRow[j] > 0)
			{
				groundSet.insert(point2id(i, j));
			}
		}
	}

// 	resize(regionMat, regionMat, Size(regionMat.cols*16, regionMat.rows*16));
// 	imshow("", regionMat);
// 	waitKey();

	// fill regionMap as returned object
	for (set<int>::iterator it = groundSet.begin(); it != groundSet.end(); it++)
	{
		Point2i pos = id2point[*it];
		int loopYStart = pos.y * regionLen;
		int loopYEnd = loopYStart + regionLen;
		int loopXStart = pos.x * regionLen;
		int loopXEnd = loopXStart + regionLen;
		for (int i = loopYStart; i < loopYEnd; i++)
		{
			int* pRow = regionMap.ptr<int>(i);
			for (int j = loopXStart; j < loopXEnd; j++)
			{
				pRow[j] = 1;
			}
		}
	}

	/*
	showCandidates(candidates, regionNodeMap);
	Mat rv = drawRegionNodeMap(img, regionNodeMap, regionLen, 3);*/
	
}

int RegionGrowthAlg::calcDstOfFeature(vector<int> feature1, vector<int> feature2)
{
	int dst = 0;
	int p1 = 0, p2 = 0; // p1:��p2:��
	int sum1 = 0, sum2 = 0;
	int featureLen = feature1.size();
	int loopLen = featureLen - 1;
	vector<int> tmp(featureLen, 0);
	for (int i = 0; i < featureLen; i++)
	{
		tmp[i] = feature1[i] - feature2[i];
	}
	while (p1 < loopLen || p2 < loopLen)
	{
		while (p1 < loopLen && tmp[p1] <= 0) p1++;
		while (p2 < loopLen && tmp[p2] >= 0) p2++;
		int iFlag = tmp[p1] + tmp[p2];
		int tmpDst = p1 - p2;
		tmpDst = tmpDst * tmpDst;
		if (iFlag > 0)
		{
			dst += (-tmp[p2]) * tmpDst;
			tmp[p1] = iFlag;
			tmp[p2] = 0;
		}
		else
		{
			dst += (tmp[p1]) * tmpDst;
			tmp[p2] = iFlag;
			tmp[p1] = 0;
		}
	}
	return dst;
}

void RegionGrowthAlg::testDisplayDst(void)
{
// 	for (int i = 1; i < regionNumY; i++)
// 	{
// 		for (int j = 1; j < regionNumX; j++)
// 		{
// 			Point pos(j*regionLen,i*regionLen);
// 			std::stringstream ss;
// 			string tmp;
// 			ss << calcDstOfFeature(features[point2id(i,j)], features[point2id(i,j-1)]);
// 			ss >> tmp;
// 			putText(img, tmp, pos, FONT_HERSHEY_SIMPLEX, 0.2, Scalar(60, 50, 90));
// 		}
// 	}
}

cv::Mat RegionGrowthAlg::drawRegionNodeMap(Mat img, vector<Node> regionNodeMap, int regionLen, float zoomRate /*= 4*/)
{
	Mat rv;
	int regionNumY = img.rows / regionLen;
	int regionNumX = img.cols / regionLen;
	int regionNum = regionNumX * regionNumY;
	resize(img, rv, Size(img.cols * zoomRate, img.rows*zoomRate));
	cvtColor(rv, rv, CV_GRAY2BGR);
	for (int i = 0; i < regionNum; i++)
	{
		int margion = 3;
		int marginWidth = 12;
		int marginHeight = 8;
		int newRegionLen = regionLen * zoomRate;
		Point2i pos = id2point[i];
		Point textPos[5];
		textPos[0].x = textPos[2].x = textPos[4].x = (pos.x + 0.5) * newRegionLen - marginWidth / 2;
		textPos[0].y = pos.y * newRegionLen + marginHeight + margion;
		textPos[2].y = (pos.y + 1) * newRegionLen - margion;
		textPos[1].y = textPos[3].y = textPos[4].y = (pos.y + 0.5) * newRegionLen + marginHeight / 2;
		textPos[1].x = (pos.x + 1) * newRegionLen - marginWidth - margion;
		textPos[3].x = pos.x * newRegionLen + margion;
		Scalar color[5] = {Scalar(255, 255, 255), Scalar(255, 0, 0), Scalar(0, 0, 0), Scalar(0, 0, 255), Scalar(0, 255, 0)};
		for (int j = 0; j < 4; j++)
		{
			std::stringstream ss;
			string tmp;
			ss << regionNodeMap[i].dst[j];
			ss >> tmp;
			putText(rv, tmp, textPos[j], FONT_HERSHEY_SIMPLEX, 0.25, color[j]);
		}
		int minDst = regionNodeMap[i].minDst;
		std::stringstream ss;
		string tmp;
		(minDst == REG_MAX_DST) ? ss << "NA" : ss << minDst;
		ss >> tmp;
		putText(rv, tmp, textPos[4], FONT_HERSHEY_SIMPLEX, 0.25, color[4]);

		Point idPos(pos.x*newRegionLen,pos.y*newRegionLen+marginHeight);
		rv.at<Vec3b>(pos.y*newRegionLen, pos.x*newRegionLen) = Vec3b(0, 0, 255);
		ss.str("");
		ss.clear();
		ss << i;
		ss >> tmp;
		putText(rv, tmp, idPos, FONT_HERSHEY_SIMPLEX, 0.2, Scalar(100,100,0));
	}
	return rv;
}

//************************************
// Method:    getRelateRegionId
// FullName:  RegionGrowthAlg::getRelateRegionId
// Access:    private 
// Returns:   int
// Qualifier: ���������
// Parameter: int regionId
// Parameter: int direction
//************************************
int RegionGrowthAlg::getRelateRegionId(int regionId, int direction)
{
	int relateRegionId = -1;
	switch (direction)
	{
	case 0:
		relateRegionId = regionId - regionNumX;
		break;
	case 1:
		relateRegionId = regionId + 1;
		break;
	case 2:
		relateRegionId = regionId + regionNumX;
		break;
	case 3:
		relateRegionId = regionId - 1;
		break;
	default:
		break;
	}
	return relateRegionId;
}

void RegionGrowthAlg::showCandidates(list<int> candidates, vector<Node> regionNodeMap)
{
	for (list<int>::iterator it = candidates.begin(); it != candidates.end(); it++)
	{
		cout << "Id: " << *it << "   Dst: " << regionNodeMap[*it].minDst << endl;
	}
}
