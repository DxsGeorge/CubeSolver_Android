#include <jni.h>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "ImageFilters.h"
#include "MyUtils.h"

extern "C" {

JNIEXPORT void JNICALL
    Java_com_example_user_opencv2_MainActivity_process(JNIEnv *env,
                                                       jlong matAddrGr,
                                                       jlong matAddrRgba,
                                                       jint tracking,
                                                       jint detected,
                                                       jint stat,
                                                       jlong points_addr,
                                                       jint thr,
                                                       jboolean colorextract,
                                                       jint faces_count,
                                                       jint succ,
                                                       jlong prevface_addr,
                                                       jint undetectednum,
                                                       jlong pt_addr,
                                                       jlong lastpt_addr);


}