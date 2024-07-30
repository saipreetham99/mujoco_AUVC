#ifndef AR_TAGS
#define AR_TAGS

#include <mujoco/mujoco.h>

#define maxPoints 100

struct ExtractedPoints_{
    int nPoints;
    float points[4 * maxPoints];
};
typedef struct ExtractedPoints_ ExtractedPoints_;

namespace auvc{

void processImage(auvcData *data, unsigned char* color_buffer);

}
#endif //AR_TAGS
