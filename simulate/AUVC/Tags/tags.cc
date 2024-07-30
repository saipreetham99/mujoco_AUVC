#include <mujoco/mujoco.h>
#include <vector>

#include "tags.h"
#include "AR_SRC/Gaussian.h"
#include "AR_SRC/FloatImage.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "AR_SRC/Edge.h"
#include "AR_SRC/FloatImage.h"
#include "AR_SRC/GLine2D.h"
#include "AR_SRC/GLineSegment2D.h"
#include "AR_SRC/Gaussian.h"
#include "AR_SRC/GrayModel.h"
#include "AR_SRC/Gridder.h"
#include "AR_SRC/Homography33.h"
#include "AR_SRC/MathUtil.h"
#include "AR_SRC/Quad.h"
#include "AR_SRC/Segment.h"
#include "AR_SRC/TagFamily.h"
#include "AR_SRC/UnionFindSimple.h"
#include "AR_SRC/XYWeight.h"

#include "AR_SRC/Tag36h11.h"

#include "AR_SRC/TagDetector.h"

void auvc::processImage(auvcData *data, unsigned char* color_buffer){
  cv::Mat image_grey;
  cv::Mat flipped;
  cv::Mat image  = cv::Mat(data->nrows, data->ncols, CV_8UC3,color_buffer);
  cv::cvtColor(image, flipped, cv::COLOR_RGB2GRAY);
  cv::flip(flipped, image_grey, 0);

  // printf("rows %d : cols %du\n",image.rows, image.cols);
  // printf("img.step/img.elemSize() : %zu\n",image.step/image.elemSize());
  cv::imshow("window", image_grey);
  cv::waitKey(0);


  AprilTags::TagDetector* m_tagDetector(NULL);
  m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_grey);
  printf("Number of Tags detected = %zu\n", detections.size());
  for (int i=0; i<detections.size(); i++) {
    // TODO: Get Detection Angle, position etc
    printf("Id: %d, Hamming %d\n",detections[i].id, detections[i].hammingDistance);
    detections[i].draw(image);
  }
  delete m_tagDetector;

}
