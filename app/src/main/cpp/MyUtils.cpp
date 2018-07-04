#include "MyUtils.h"

vector<vector<float>> ArrLstToVec()
{

}

float Distance (int x1, int y1, int x2, int y2)
{
    float abs1=pow(float (x1-x2),2)+pow(float (y1-y2),2);
    float dist=sqrt(abs1);
    return dist;
}

float Distance (Point p1, Point p2)
{
    return Distance(p1.x,p1.y,p2.x,p2.y);
}

bool SamePoint (Point p1, Point p2, int offset)
{
    return (abs(p1.x-p2.x)<offset && abs(p1.y-p2.y)<offset);
}

bool FindIntersection(Vec4i l1, Vec4i l2, float &ua, float &ub, Point &pt)
{
    Point p1(l1[0], l1[1]) , p2 (l1[2], l1[3]), p3 (l2[0], l2[1]) , p4(l2[2], l2[3]);
    float den = (p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y);
    if (abs(den) < 0.1) return false;
    ua = (p4.x - p3.x)*(p1.y - p3.y) - (p4.y - p3.y)*(p1.x - p3.x);
    ub = (p2.x - p1.x)*(p1.y - p3.y) - (p2.y - p1.y)*(p1.x - p3.x);
    ua = ua / den;
    ub = ub / den;
    float x = p1.x + ua*(p2.x - p1.x);
    float y = p1.y + ua*(p2.y - p1.y);
    pt = Point(x, y);
    if (ua>0 && ub>0 && ua<1 && ub<1) return true;
    return false;
}

float AngletoX(Vec4i pt)
{
    float ang = atan(float(pt[1] - pt[3] )/ float(pt[0] - pt[2]));
    ang >= 0 ? ang : ang += CV_PI;
    return ang;
}

int compfaces(vector<Point> f1, vector<Point> f2)
{
    int totd = 0;
    for (int i = 0; i < f1.size(); ++i)
    {
        int mind = 10000;
        for (int j = 0; j < f2.size(); ++j)
        {
            float d = Distance(f1[i], f2[j]);
            if (d < mind) mind = d;
        }
        totd += mind;
    }

    return (totd / 4);
}

LinePair::LinePair()
{};

LinePair::LinePair(Vec4i line1, Vec4i line2)
{
    this->line1 = line1;
    this->line2 = line2;
    if (SamePoint(Point(line1[0], line1[1]), Point(line2[0], line2[1]), 10))
    {
        this->same = Point(line1[0], line1[1]);
        this->end1 = Point(line1[2], line1[3]);
        this->end2 = Point(line2[2], line2[3]);
    }
    if (SamePoint(Point(line1[0], line1[1]), Point(line2[2], line2[3]), 10))
    {
        this->same = Point(line1[0], line1[1]);
        this->end1 = Point(line1[2], line1[3]);
        this->end2 = Point(line2[0], line2[1]);
    }
    if (SamePoint(Point(line1[2], line1[3]), Point(line2[0], line2[1]), 10))
    {
        this->same = Point(line1[2], line1[3]);
        this->end1 = Point(line1[0], line1[1]);
        this->end2 = Point(line2[2], line2[3]);
    }
    if (SamePoint(Point(line1[2], line1[3]), Point(line2[2], line2[3]), 10))
    {
        this->same = Point(line1[2], line1[3]);
        this->end1 = Point(line1[0], line1[1]);
        this->end2 = Point(line2[0], line2[1]);
    }
    this->distance = Distance(Point(line1[0], line1[1]), Point(line1[2], line1[3]));
    this->evidence = 0;
}

LinePair::LinePair(Point same, Point end1, Point end2)
{
    this->same = same;
    this->end1 = end1;
    this->end2 = end2;
    this->line1 = Vec4i{ same.x, same.y, end1.x, end1.y };
    this->line2 = Vec4i{ same.x, same.y, end2.x, end2.y };
    this->distance = (Distance(same, end1)+Distance(same,end2))/2.0;
    this->evidence = 0;
}

Vec4i LinePair::getLine1()
{
    return this->line1;
}

Vec4i LinePair::getLine2()
{
    return this->line2;
}

float LinePair::getDist()
{
    return distance;
}

Point LinePair::getend1()
{
    return this->end1;
}

Point LinePair::getend2()
{
    return this->end2;
}

Point LinePair::getsame()
{
    return this->same;
}

void FindCubeFace1(vector<Vec4i> lines, vector<Point> &prevface, int &succ, int &detected, int &tracking,
                   Point &v1, Point &v2, Point &p0, vector<Point2f> &features, vector<Point> &pt)
{
    //find cube corners based on paper
    vector <LinePair> lp;
    vector <Point> edges, test, minps;
    float qwe = 0.06;
    int minch = 10000;
    for (size_t i = 0; i < lines.size(); i++)
    {
        for (size_t j = i + 1; j < lines.size(); j++)
        {
            int matched = 0;
            LinePair hypothesis;
            Point p1, p2, p3, p4;
            float dd1, dd2;
            p1 = Point(lines[i][0], lines[i][1]);
            p2 = Point(lines[i][2], lines[i][3]);
            p3 = Point(lines[j][0], lines[j][1]);
            p4 = Point(lines[j][2], lines[j][3]);
            dd1 = Distance(p1, p2);
            dd2 = Distance(p3, p4);
            if (max(dd1, dd2) / min(dd1, dd2) > 1.3) continue;
            if (SamePoint(p1, p3, 10))
            {
                hypothesis = LinePair(lines[i], lines[j]);
                matched++;
            }
            if (SamePoint(p1, p4, 10))
            {
                hypothesis = LinePair(lines[i], lines[j]);
                matched++;
            }
            if (SamePoint(p2, p3, 10))
            {
                hypothesis = LinePair(lines[i], lines[j]);
                matched++;
            }
            if (SamePoint(p2, p4, 10))
            {
                hypothesis = LinePair(lines[i], lines[j]);
                matched++;
            }
            if (matched == 0)
            {
                //check intersection at 1/3 or 2/3
                Point pt, temp, same, end1, end2;
                float ua, ub;
                if (FindIntersection(lines[i], lines[j], ua, ub, pt))
                {
                    int ok1 = 0, ok2 = 0;
                    if (abs(ua - 1.0 / 3) < 0.05) ok1 = 1;
                    if (abs(ua - 2.0 / 3) < 0.05)  ok1 = 2;
                    if (abs(ub - 1.0 / 3)<0.05)  ok2 = 1;
                    if (abs(ub - 2.0 / 3)<0.05) ok2 = 2;
                    if (ok1>0 && ok2>0)
                    {
                        if (ok1 == 2)
                        {
                            temp = p1;
                            p1 = p2;
                            p2 = temp;

                        }
                        if (ok2 == 2)
                        {
                            temp = p3;
                            p3 = p4;
                            p4 = temp;
                        }
                        end1 = Point((p3.x + 2.0 / 3 * (p2.x - p1.x)), (p3.y + 2.0 / 3 * (p2.y - p1.y)));
                        end2 = Point((p1.x + 2.0 / 3 * (p4.x - p3.x)), (p1.y + 2.0 / 3 * (p4.y - p3.y)));
                        same = Point((p1.x - 1.0 / 3 * (p4.x - p3.x)), (p1.y - 1.0 / 3 * (p4.y - p3.y)));
                        hypothesis = LinePair(same, end1, end2);
                        matched = 1;
                    }
                }
            }
            if (matched == 1)
            {
                float ang1, ang2, ang;
                ang1 = atan(float(p1.y - p2.y) / float(p1.x - p2.x));
                ang2 = atan(float(p3.y - p4.y) / float(p3.x - p4.x));
                if (ang1 < 0) ang1 += CV_PI;
                if (ang2 < 0) ang2 += CV_PI;
                ang = abs(abs(ang2 - ang1) - CV_PI / 2);
                if (ang < 0.5) lp.push_back(hypothesis);
            }
        }
    }

    //check how many lines align to a grid
    for (size_t i = 0; i < lp.size(); i++)
    {
        Point same, end1, end2;
        same = lp[i].getsame();
        end1 = lp[i].getend1();
        end2 = lp[i].getend2();
        float ang1, ang2, distance;
        distance = lp[i].getDist();
        if (end1.x - same.x != 0) ang1 = atan((end1.y - same.y) / (end1.x - same.x));
        else ang1 = CV_PI / 2;
        if (end2.x - same.x != 0) ang2 = atan((end2.y - same.y) / (end2.x - same.x));
        else ang2 = CV_PI / 2;
        if (ang1 < 0) ang1 += CV_PI;
        if (ang2 < 0) ang2 += CV_PI;
        distance = 1.7*distance;
        int evidence = 0;
        int totallines = 0;
        Eigen::Matrix3f A, Ainv;
        A << end2.x - same.x, end1.x - same.x, same.x,
                end2.y - same.y, end1.y - same.y, same.y,
                0, 0, 1;
        Ainv = A.inverse();
        for (size_t j = 0; j < lines.size(); j++)
        {
            float ang = AngletoX(lines[j]);
            float a1, a2;
            a1 = abs(abs(ang - ang1) - CV_PI / 2);
            a2 = abs(abs(ang - ang2) - CV_PI / 2);
            if (a1 > 0.1 && a2 > 0.1) continue;
            Point q1, q2;
            q1 = Point(lines[j][0], lines[j][1]); q2 = Point(lines[j][2], lines[j][3]);
            Eigen::Matrix<float, 3, 1> v;
            v << q1.y,
                    q1.x,
                    1;
            Eigen::Matrix<float, 3, 1> vp1 = Ainv*v;
            if (vp1(0, 0) > 1.1 || vp1(0, 0)<-0.1) continue;
            if (vp1(1, 0) > 1.1 || vp1(1, 0)<-0.1) continue;
            if ((abs(vp1(0, 0) - 1 / 3.0) > qwe)
                &&
                (abs(vp1(0, 0) - 2 / 3.0) > qwe)
                &&
                (abs(vp1(1, 0) - 1 / 3.0) > qwe)
                &&
                (abs(vp1(1, 0) - 2 / 3.0) > qwe)) continue;
            Eigen::Matrix<float, 3, 1> v2;
            v2 << q2.y,
                    q2.x,
                    1;
            Eigen::Matrix<float, 3, 1> vp2 = Ainv*v2;
            if (vp2(0, 0) > 1.1 || vp2(0, 0)<-0.1) continue;
            if (vp2(1, 0) > 1.1 || vp2(1, 0)<-0.1) continue;
            if ((abs(vp2(0, 0) - 1 / 3.0) > qwe)
                &&
                (abs(vp2(0, 0) - 2 / 3.0) > qwe)
                &&
                (abs(vp2(1, 0) - 1 / 3.0) > qwe)
                &&
                (abs(vp2(1, 0) - 2 / 3.0) > qwe)) continue;
            lp[i].evidence++;
        }
    }
    std::sort(lp.begin(), lp.end(), [](LinePair a, LinePair b){ return (a.evidence > b.evidence); });
    if (lp.size() > 0)
    {
        //check for same grid at least 3 times
        for (size_t i = 0; i < lp.size(); ++i)
        {

            if (lp[i].evidence > 0.05*lines.size())
            {
                Point same(lp[i].getsame()), end1(lp[i].getend1()), end2(lp[i].getend2());
                Point p3(end2.x + end1.x - same.x, end2.y + end1.y - same.y);
                test = { same, end1, end2, p3 };
                p3 = Point(prevface[2].x + prevface[1].x - prevface[0].x, prevface[2].y + prevface[1].y - prevface[0].y);
                vector<Point> tc{ prevface[0], prevface[1], prevface[2], p3 };
                int ch = compfaces(test, tc);
                if (ch < minch)
                {
                    minch = ch;
                    minps = { same, end1, end2 };
                }
            }
        }
        if (minps.size() > 0)
        {
            prevface = minps;
            if (minch < 10)
            {
                succ += 1;
                pt = prevface;
                detected = 1;
            }
        }
        else
        {
            succ = 0;
        }
        if (succ > 2)
        {
            pt = {};
            for (int i = 1; i < 3; ++i)
            {
                for (int j = 1; j < 3; ++j)
                {
                    pt.push_back(Point(p0.x + float(i) / 3 * v1.x + float(j) / 3 * v2.x, p0.y + float(i) / 3 * v1.y + float(j) / 3 * v2.y));
                }
            }
            features = {};
            for (size_t i = 0; i < pt.size(); ++i)
            {
                features.push_back(Point2f(pt[i].x, pt[i].y));
            }
            tracking = 1;
            succ = 0;
        }
    }


}

SimpleFace::SimpleFace()
{

}

SimpleFace::SimpleFace(vector<Scalar> face1)
{
    this->centerHSV = face1[4];
    this->colorsHSV = face1;
};

Scalar SimpleFace::getCenterHSV()
{
    return this->centerHSV;
}

vector<Scalar> SimpleFace::getColorsHSV()
{
    return this->colorsHSV;
}

Scalar colavg(Mat src, Point2f point, float dist)
{
    Scalar col, sum;
    vector<Point2f> points = pointcube(point, dist);
    for (size_t i = 0; i < points.size(); ++i)
    {
        sum += Scalar(src.at<Vec3b>(points[i]));
    }
    col = Scalar(sum[0]/9,sum[1]/9,sum[2]/9);
    return col;
}

AnglePoint::AnglePoint()
{

}

AnglePoint::AnglePoint(float angle, Point point)
{
    this->angle = angle;
    this->point = point;
}

float AnglePoint::getAngle()
{
    return this->angle;
}

Point AnglePoint::getPoint()
{
    return this->point;
}

vector<Point> winded(Point p1, Point p2, Point p3, Point p4)
{
    vector<AnglePoint> ps;
    vector<Point> ps1;
    Point2f avg = Point2f(0.25*(p1.x + p2.x + p3.x + p4.x), 0.25*(p1.y + p2.y + p3.y + p4.y));
    AnglePoint ts1(atan2(p1.y - avg.y, p1.x - avg.x), p1);
    AnglePoint ts2(atan2(p2.y - avg.y, p2.x - avg.x), p2);
    AnglePoint ts3(atan2(p3.y - avg.y, p3.x - avg.x), p3);
    AnglePoint ts4(atan2(p4.y - avg.y, p4.x - avg.x), p4);
    ps = {ts1, ts2, ts3, ts4};
    std::sort(ps.begin(), ps.end(), [](AnglePoint a, AnglePoint b){ return (a.getAngle() > b.getAngle()); });
    for (size_t i = 0; i < ps.size(); i++)
    {
        ps1.push_back(ps[i].getPoint());
    }
    return ps1;
}

vector<Point2f> pointcube(Point2f a, float dist)
{
    vector<Point2f> cube;
    dist = dist / 4.0;
    Point2f ul = Point2f(a.x - dist, a.y - dist);
    Point2f um = Point2f(a.x, a.y - dist);
    Point2f ur = Point2f(a.x + dist, a.y - dist);
    Point2f ml = Point2f(a.x - dist, a.y);
    Point2f mm = a;
    Point2f mr = Point2f(a.x + dist, a.y);
    Point2f bl = Point2f(a.x - dist, a.y + dist);
    Point2f bm = Point2f(a.x, a.y + dist);
    Point2f br = Point2f(a.x + dist, a.y + dist);
    cube = { ul, um, ur, ml, mm, mr, bl, bm, br };
    return cube;
}

