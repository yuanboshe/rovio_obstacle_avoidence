#include "ros/ros.h"
#include "rovio_base/manDrv.h"
#include "rovio_base/image.h"
#include "rovio_base/report.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "Utility.h"
#include "RegionGrowthAlg.h"

using namespace cv;
using namespace std;

ros::ServiceClient imageClient;
ros::ServiceClient controlClient;
ros::ServiceClient reportClient;
rovio_base::image imageSrv;
rovio_base::manDrv controlSrv;
rovio_base::report reportSrv;

typedef struct
{
  std::string cmd;
  uint length;
  int lDirection; // 1:forward, -1:back, 0:no move
  uint lNum;
  int rDirection; // 1:forward, -1:back, 0:no move
  uint rNum;
  int rearDirection; // -1:left, 1:right, 0:no move
  uint rearNum;
  uint headPosition;
  uchar batteryStatus;
  bool isLedOn;
  bool isIrOn;
  bool isDetectedBarrier;
  uchar chargerStatus;
} rvMCUReport;

rvMCUReport getReport()
{
  rvMCUReport rv;
  if (reportClient.call(reportSrv))
  {
    rv.length = reportSrv.response.length;
    rv.lDirection = reportSrv.response.lDirection;
    rv.lNum = reportSrv.response.lNum;
    rv.rDirection = reportSrv.response.rDirection;
    rv.rNum = reportSrv.response.rNum;
    rv.rearDirection = reportSrv.response.rearDirection;
    rv.rearNum = reportSrv.response.rearNum;
    rv.headPosition = reportSrv.response.headPosition;
    rv.isLedOn = reportSrv.response.isLedOn;
    rv.isIrOn = reportSrv.response.isIrOn;
    rv.isDetectedBarrier = reportSrv.response.isDetectedBarrier;
    ROS_INFO("MCU Report:\nlength=%d", rv.length);
    ROS_INFO("Left direction:num=%d:%d", rv.lDirection, rv.lNum);
    ROS_INFO("Right direction:num=%d:%d", rv.rDirection, rv.rNum);
    ROS_INFO("Rear direction:num=%d:%d", rv.rearDirection, rv.rearNum);
    ROS_INFO("headPosition=%d", rv.headPosition);
    ROS_INFO("isLedOn=%d,isIrOn=%d,isDetectedBarrier=%d", rv.isLedOn, rv.isIrOn, rv.isDetectedBarrier);
  }
  else
  {
    ROS_ERROR("Failed to call service rovioReport");
  }

  return rv;
}

int control(int drive, int speed)
{
  int rv;
  controlSrv.request.drive = drive;
  controlSrv.request.speed = speed;
  if (controlClient.call(controlSrv))
  {
    rv = controlSrv.response.code;
    ROS_INFO("Control response code: %d", rv);
  }
  else
  {
    ROS_ERROR("Failed to call service rovioControl");
  }

  return rv;
}

Mat getImage()
{
  Mat rv;
  if (imageClient.call(imageSrv))
  {
    ROS_INFO("Image size: %dx%d", (int )imageSrv.response.img.width, (int )imageSrv.response.img.height);
    cv_bridge::CvImagePtr cvImgPtr;
    try
    {
      cvImgPtr = cv_bridge::toCvCopy(imageSrv.response.img, sensor_msgs::image_encodings::BGR8);
      rv = cvImgPtr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  else
  {
    ROS_ERROR("Failed to call service rovioImage");
  }
  return rv;
}

Mat getImg(Mat img, Rect ior)
{
  return img(ior);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rovioTest");
  ros::NodeHandle n;
  imageClient = n.serviceClient<rovio_base::image>("rovioImage");
  controlClient = n.serviceClient<rovio_base::manDrv>("rovioControl");
  reportClient = n.serviceClient<rovio_base::report>("rovioReport");

  Mat imgOrd = getImage();
  cvtColor(imgOrd, imgOrd, CV_RGB2GRAY);
  const int AREA_SIZE = 16;
  int ordRows = imgOrd.rows;
  int ordCols = imgOrd.cols;
  int areaNumY = ordRows / AREA_SIZE;
  int areaNumX = ordCols / AREA_SIZE;
  int marginY = ordRows % AREA_SIZE;
  int areaNum = areaNumX * areaNumY;
  Rect ior(0, marginY, areaNumX * AREA_SIZE, areaNumY * AREA_SIZE);
  Mat img = getImg(imgOrd, ior);
  VideoWriter videoWriter("/home/viki/Rovio.avi", CV_FOURCC('M', 'J', 'P', 'G'), 3.0, Size(img.cols, img.rows));
  int lastDirection = 1;
  for (int i = 0; i < 1000; i++)
  {
    Mat img = getImg(getImage(), ior);
    Mat_<int> regionMap(img.size());
    regionMap.setTo(0);
    regionMap(regionMap.rows - 18, regionMap.cols / 2) = 1;
    regionMap(regionMap.rows - 10, regionMap.cols / 2) = 1;
    RegionGrowthAlg alg;
    alg.calcRegionMap(img, regionMap);

    // Detect forward
    int fAreaWidth = img.cols - 200;
    int fAreaHeight = img.rows / 4;
    int fTopX = (img.cols - fAreaWidth) / 2;
    int fTopY = img.rows - fAreaHeight;
    int ignorePixels = 1000;
    Mat_<int> fArea = regionMap(Rect(fTopX, fTopY, fAreaWidth, fAreaHeight));
    int fAreaSum = 0;
    for (int i = 0; i < fArea.rows; i++)
    {
      int* pRow = fArea.ptr<int>(i);
      for (int j = 0; j < fArea.cols; j++)
      {
        if (pRow[j] == 0)
          fAreaSum++;
      }
    }
    bool flagForward = fAreaSum < ignorePixels;

    // Detect left and right
    int lrAreaWidth = 100;
    int marginX = 0;
    int lrAreaHeight = fAreaHeight;
    int lrTopY = img.rows - lrAreaHeight;
    int lTopX = marginX;
    int rTopX = img.cols - marginX - lrAreaWidth;
    int lrIgnorePixels = 1000;
    Mat_<int> lArea = regionMap(Rect(lTopX, lrTopY, lrAreaWidth, lrAreaHeight));
    Mat_<int> rArea = regionMap(Rect(rTopX, lrTopY, lrAreaWidth, lrAreaHeight));
    int lAreaSum = 0;
    int rAreaSum = 0;
    for (int i = 0; i < lArea.rows; i++)
    {
      int* plRow = lArea.ptr<int>(i);
      int* prRow = rArea.ptr<int>(i);
      for (int j = 0; j < lArea.cols; j++)
      {
        if (plRow[j] == 0)
          lAreaSum++;
        if (prRow[j] == 0)
          rAreaSum++;
      }
    }
    bool flagLeft = lAreaSum < lrIgnorePixels;
    bool flagRight = rAreaSum < lrIgnorePixels;

    //fArea.setTo(2);
    lArea.setTo(3);
    rArea.setTo(4);

    Utility util;
    util.drawSegmentBorder(img, regionMap);

    // Mark info
    //标记
    int leftSum = 0;
    int rightSum = 0;
    int loopi = img.rows;
    int loopj = img.cols;
    for (int i = 0; i < loopi; i++)
    {
      int* pLeftRow = regionMap.ptr<int>(i);
      int* pRIghtRow = pLeftRow + loopj / 2;
      int loop = loopj / 2;
      for (int j = 0; j < loop; j++)
      {
        if (pLeftRow[j] > 0)
        {
          leftSum++;
        }
        if (pRIghtRow[j] > 0)
        {
          rightSum++;
        }
      }
    }
    Point pos(loopj / 2 - 150, loopi / 2);
    std::stringstream ss;
    string tmp;
    ss << leftSum;
    ss >> tmp;
    putText(img, tmp, pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0));
    pos.x = loopj / 2 + 100;
    ss.str("");
    ss.clear();
    ss << rightSum;
    ss >> tmp;
    putText(img, tmp, pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0));
    int textLen = 40;
    pos.x = fArea.cols / 2 - textLen + fTopX;
    pos.y = fArea.rows / 2 + fTopY;
    ss.str("");
    ss.clear();
    ss << fAreaSum;
    ss >> tmp;
    putText(img, tmp, pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 50, 255));
    pos.x = lArea.cols / 2 - textLen + lTopX;
    pos.y = lArea.rows / 2 + lrTopY;
    ss.str("");
    ss.clear();
    ss << lAreaSum;
    ss >> tmp;
    putText(img, tmp, pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 50, 255));
    pos.x = rArea.cols / 2 - textLen + rTopX;
    pos.y = rArea.rows / 2 + lrTopY;
    ss.str("");
    ss.clear();
    ss << rAreaSum;
    ss >> tmp;
    putText(img, tmp, pos, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 50, 255));

    //检测直线区域
    int lineLen = 200;
    int lineStartX = img.cols / 2 - lineLen / 2;
    int lineEndX = img.cols / 2 + lineLen / 2;
    int lineY = img.rows - 140;
    Point lineStart(lineStartX, lineY);
    Point lineEnd(lineEndX, lineY);
    line(img, lineStart, lineEnd, Scalar(255, 20, 20));

    int blockNum = 0;
    int* pLineRow = regionMap.ptr<int>(lineY);
    for (int j = lineStartX; j < lineEndX; j++)
    {
      if (pLineRow[j] == 0)
      {
        blockNum++;
      }
    }
    bool isBlocked = blockNum > lineLen / 2;

    //视频
    //cvtColor(img, img, CV_GRAY2RGB);
    imshow("", img);
    waitKey(10);
    videoWriter << img;

    //控制
    isBlocked = (!flagLeft && !flagRight) || isBlocked;
    int waitTime = 1;
    rvMCUReport rvMcu = getReport();
    if (rvMcu.isIrOn && rvMcu.isDetectedBarrier)
      isBlocked = true;
    if (true)
    {
      if (isBlocked)
      {
        int maxDif = 5000;
        if (leftSum - rightSum > maxDif)
        {
          lastDirection = -1;
        }
        else if (rightSum - leftSum > maxDif)
        {
          lastDirection = 1;
        }
        if (lastDirection == -1)
        {
          control(5, 8);
          waitKey(waitTime);
        }
        else
        {
          control(6, 8);
          waitKey(waitTime);
        }
      }
      else if (flagForward)
      {
        control(1, 6);
      }
      else if (flagLeft)
      {
        control(3, 8);
      }
      else if (flagRight)
      {
        control(4, 8);
      }
      else
      {
        printf("Error control");
      }
    }

  }

  return 0;
}
