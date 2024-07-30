#ifndef AR_TAGS
#define AR_TAGS

#include <mujoco/mujoco.h>
#include <opencv2/opencv.hpp>

#define maxPoints 100

struct ExtractedPoints_{
    int nPoints;
    float points[8 * maxPoints];
};
typedef struct ExtractedPoints_ ExtractedPoints_;

namespace auvc{

void processImage(auvcData *data, cv::Mat* flipped, cv::Mat* image, cv::Mat* image_gray, unsigned char* color_buffer);
}
#endif //AR_TAGS
