#include <native-lib.h>

using namespace std;
using namespace cv;


extern "C" {


    JNIEXPORT void JNICALL Java_com_example_user_opencv2_MainActivity_process(JNIEnv *env,
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
                                                                              jlong lastpt_addr)
    {
        //std::string hello = "entered";
        //env->NewStringUTF(hello.c_str());
        Mat &mRGBA = *(Mat *) matAddrRgba;
        Mat &mGrey = *(Mat *) matAddrGr;
        Mat prev_mGrey;
        Mat dst = FilterImage(mRGBA);
        Mat hsv, pyrdown;
        pyrDown(mRGBA, pyrdown, Size(mRGBA.cols / 2, mRGBA.rows / 2));
        cvtColor(pyrdown, hsv, CV_RGB2HSV);
        vector<Point2f> pts, prev_pts;
        vector<Point> prevface, pt, lastpt;
        Mat &point_mat = *(Mat *) points_addr;
        Mat &prevface_mat = *(Mat *) prevface_addr;
        Mat &pt_mat = *(Mat *) pt_addr;
        Mat &lastpt_mat = *(Mat *) lastpt_addr;
        Point v1(0, 0), v2(0, 0), p0(0, 0);
        vector<Scalar> face1;
        for (int i = 0; i < point_mat.rows; ++i) {
            Vec<float, 2> v = point_mat.at<Vec<float, 2> >(i, 0);
            Point2f p(v[0], v[1]);
            pts.push_back(p);
        }
        for (int i = 0; i < prevface_mat.rows; ++i)
        {
            Vec<int, 2> v = prevface_mat.at<Vec<int, 2> >(i, 0);
            Point p(v[0], v[1]);
            prevface.push_back(p);
        }
        for (int i = 0; i < pt_mat.rows; ++i)
        {
            Vec<int, 2> v = pt_mat.at<Vec<int, 2> >(i, 0);
            Point p(v[0], v[1]);
            pt.push_back(p);
        }
        for (int i = 0; i < lastpt_mat.rows; ++i)
        {
            Vec<int, 2> v = lastpt_mat.at<Vec<int, 2> >(i, 0);
            Point p(v[0], v[1]);
            lastpt.push_back(p);
        }
        if (tracking > 0) {
            detected = 2;
            TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.3);
            Size subPixWinSize(10, 10), winSize(5, 5);
            vector<uchar> status;
            Mat err;
            calcOpticalFlowPyrLK(prev_mGrey, mGrey, prev_pts, pts, status, err, winSize);
            /*
            for (size_t i = 0; i < pointsf.size(); ++i)
            {
                //circle(src, pointsf_next[i], 2, Scalar(0, 255, 0), 1);
                //circle(off_view, pointsf_next[i], 2, Scalar(0, 255, 0), 1);
            }
            circle(src, pointsf_next[0], 2, Scalar(255, 0, 0), 1);
            circle(src, pointsf_next[1], 2, Scalar(0, 255, 0), 1);
            circle(src, pointsf_next[2], 2, Scalar(0, 0, 255), 1);
            circle(src, pointsf_next[3], 2, Scalar(255, 255, 255), 1);
             */
            prev_pts = pts;

            //check outliers from lk tracing
            for (size_t i = 0; i < status.size(); ++i) {
                if (status[i] == 0) stat++;
            }
            if (stat == 3) {
                tracking = 0;
                stat = 0;
            }
            for (size_t i = 0; i < pts.size(); ++i) {
                if (pts[i].x > pyrdown.cols / 2 || pts[i].y > pyrdown.rows / 2) tracking = 0;
            }

            //Find cube pose in 3D
            //rvec = FindCubeOrientation(pointsf_next, cam, dist, tvec);
            //vector<Point2f> impts;
            //projectPoints(normal, rvec, tvec, cam, dist, impts);

            //Check if lost
            float ds1 = Distance(pts[0], pts[1]);
            float ds2 = Distance(pts[2], pts[3]);
            if (max(ds1, ds2) / min(ds1, ds2) > 1.4) tracking = 0;
            float ds3 = Distance(pts[0], pts[2]);
            float ds4 = Distance(pts[1], pts[3]);
            if (max(ds3, ds4) / min(ds3, ds4) > 1.4) tracking = 0;
            if (ds1 < 10 || ds2 < 10 || ds3 < 10 || ds4 < 10) tracking = 0;
            if (tracking == 0) {
                detected = 0;
                colorextract = false;
            }
        }
        if (tracking == 0)
        {
            //find cube face
            vector<Vec4i> lines;
            vector<Point> edges;
            HoughLinesP(dst, lines, 1, CV_PI / 45, thr, 10, 5);
            /*
            for (size_t i = 0; i < lines.size(); i++)
            {
                line(src, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 1);
            }
            */
            if (lines.size() < 50) thr = max(thr - 1, 2);
            else thr++;
            //cout << "lines: " << lines.size() << "threshold: " << thr << endl;
            FindCubeFace1(lines, prevface, succ, detected, tracking, v1, v2, p0, pts, pt);
        }
        else
        {
            Point p = pts[0];
            Point p1 = pts[1];
            Point p2 =pts[2];
            v1 = Point(p1.x - p.x, p1.y - p.y);
            v2 = Point(p2.x - p.x, p2.y - p.y);
            pt = {Point(p.x - v1.x - v2.x, p.y - v1.y - v2.y),
                  Point(p.x + 2 * v2.x - v1.x, p.y + 2 * v2.y - v1.y),
                  Point(p.x + 2 * v1.x - v2.x, p.y + 2 * v1.y - v2.y)};
            prevface = {pt[0], pt[1], pt[2]};
        }
        if ((detected > 0 || undetectednum < 1)) {
            if (detected <= 0) {
                undetectednum++;
                pt = lastpt;
            } else {
                undetectednum = 0;
                lastpt = pt;
            }
            Point p3(pt[2].x + pt[1].x - pt[0].x, pt[2].y + pt[1].y - pt[0].y);
            line(mRGBA, pt[0], pt[1], Scalar(0, 255, 0), 2);
            line(mRGBA, pt[1], p3, Scalar(0, 255, 0), 2);
            line(mRGBA, p3, pt[2], Scalar(0, 255, 0), 2);
            line(mRGBA, pt[2], pt[0], Scalar(0, 255, 0), 2);
            /*
            line(off_view, pt[0], pt[1], Scalar(0, 255, 0), 2);
            line(off_view, pt[1], p3, Scalar(0, 255, 0), 2);
            line(off_view, p3, pt[2], Scalar(0, 255, 0), 2);
            line(off_view, pt[2], pt[0], Scalar(0, 255, 0), 2);
            */
            pt = winded(pt[0], pt[1], pt[2], p3);

            v1 = Point(pt[1].x - pt[0].x, pt[1].y - pt[0].y);
            v2 = Point(pt[3].x - pt[0].x, pt[3].y - pt[0].y);
            p0 = Point(pt[0].x, pt[0].y);


            vector<Point2f> ep;
            int i = 1;
            int j = 5;

            if (colorextract) {
                //cube is tracked
                for (size_t k = 0; k < 9; ++k) {
                    ep.push_back(Point2f((p0.x + i * v1.x / 6.0 + j * v2.x / 6.0),
                                         (p0.y + i * v1.y / 6.0 + j * v2.y / 6.0)));
                    i += 2;
                    if (i == 7) {
                        i = 1;
                        j -= 2;
                    }
                }
                float rad = Distance(v1, Point(0, 0)) / 6.0;
                rad = 0.8 * rad;
                float stickdist = Distance(ep[0], ep[1]);
                for (size_t i = 0; i < ep.size(); ++i) {
                    //if (ep[i].x>rad && ep[i].x < src.cols / 2 - rad && ep[i].y>rad && ep[i].y < src.rows - rad)
                    {
                        Scalar col_avg1, col_avg2, col_avg3;
                        vector<Point2f> cubepoints = pointcube(ep[i], stickdist);

                        //Vec3b color = yuv.at<Vec3b>(ep[i]);
                        col_avg1 = colavg(hsv, ep[i], stickdist);
                        vector<Scalar> col_avg = {col_avg1, col_avg2, col_avg3};
                        face1.push_back(col_avg1);
                        for (size_t it = 0; it < cubepoints.size(); ++it) {
                            circle(mRGBA, cubepoints[it], 2, Scalar(255, 255, 255), 1);
                        }
                    }
                }
                bool newface = true;
                if (face1.size() != 9) newface = false;
                /*
                for (size_t i = 0; i < faces.size() && newface; ++i) {
                    if (ScalarCompare(faces[i].getCenterHSV(), face1[4]) < 25.5)
                    if (CompareOnlyH(faces[i].getCenterHSV(), face1[4]) < 25)
                    if	(face2.size() < 9 || FaceCompareYUV(faces[i], face2) < 1.8)
                    {
                        cout << "wrong face" << endl;
                        newface = false;
                    }

                }
                */
                if (newface) {
                    //faces.push_back(SimpleFace(face1));
                }
                face1.clear();
                colorextract = false;
            }
        }
    }
    #if 1
    jstring

    JNICALL
    Java_com_example_user_opencv2_MainActivity_stringFromJNI(
            JNIEnv *env,
            jobject /* this */) {
        std::string hello = "Hello from C++";
        return env->NewStringUTF(hello.c_str());
        }


    jstring
    Java_com_example_user_opencv2_MainActivity_validate(JNIEnv *env, jobject, jlong addrGray, jlong addrRgba) {
        cv::Rect();
        cv::Mat();
        cv::Mat lines;
        cv::Mat src;
        std::string hello2 = "Hello from validate";
        return env->NewStringUTF(hello2.c_str());
        }
    #endif

}


