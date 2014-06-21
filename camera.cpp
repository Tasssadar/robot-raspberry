#include <assert.h>

#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <unistd.h>

#include "camera.h"
#include "tcpserver.h"
#include "util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

#ifndef NORPI
  #include <raspicam/raspicam_cv.h>
#endif

#define RES_X 640
#define RES_Y 360

static void *camera_run_capture_thread(void *camera)
{
    ((Camera*)camera)->capture_thread_work();
    return NULL;
}

static void threshold_set(int, void *)
{
    sCamera.find_bear();
}

static void mouseEvent(int evt, int x, int y, int flags, void* param)
{
    if(evt == CV_EVENT_LBUTTONDOWN)
    {
        sCamera.clearCutPoints();
    }

    if(flags & CV_EVENT_FLAG_LBUTTON)
    {
        //sCamera.addCutPoint(x, y);
        sCamera.printColorAt(x, y);
    }
}

Camera::Camera()
{
#ifndef NORPI
    m_capture = NULL;
#endif
    m_frame.create(RES_Y, RES_X, CV_8UC3);
    m_frame.setTo(Scalar(0, 0, 0));

    m_lastDisplayFrame = m_frame;

    m_run_capture = false;
    m_threshold = 70;
    m_show_gui = false;
    m_last_diff = -1;

    for(size_t i = 0; i < DIFF_MAX_CNT; ++i)
    {
        m_bear[i] = Rect(-1, -1, -1, -1);
        m_mask[i].create(RES_Y, RES_X, CV_8UC1);
        m_mask[i].setTo(Scalar(255, 255, 255));
    }

    loadMask();

    m_detectWallClr[0] = Scalar(0, 24, 31);
    m_detectWallClr[1] = Scalar(29, 196, 255);

    pthread_mutex_init(&m_frame_mutex, 0);
    pthread_cond_init(&m_frame_cond, 0);
}

Camera::~Camera()
{
    pthread_cond_destroy(&m_frame_cond);
    pthread_mutex_destroy(&m_frame_mutex);
    close();
}

void Camera::open(int threshold)
{
#ifndef NORPI
    if(m_capture)
        return;

    m_capture = new raspicam::RaspiCam_Cv();
    m_capture->set(CV_CAP_PROP_FRAME_WIDTH, RES_X);
    m_capture->set(CV_CAP_PROP_FRAME_HEIGHT, RES_Y);

    if (!m_capture->open()) {
        LOGE("capture is NULL");
        delete m_capture;
        m_capture = NULL;
        return;
    }

    m_capture->set(CV_CAP_PROP_FRAME_WIDTH, RES_X);
    m_capture->set(CV_CAP_PROP_FRAME_HEIGHT, RES_Y);
#endif

    m_run_capture = true;
    pthread_create(&m_capture_thread, 0, camera_run_capture_thread, this);

    if(threshold > 0)
        setThreshold(threshold);
}

void Camera::close()
{
#ifndef NORPI
    if(m_capture == NULL)
        return;
#endif

    m_run_capture = false;
    pthread_join(m_capture_thread, 0);

#ifndef NORPI
    m_capture->release();
    delete m_capture;
    m_capture = NULL;
#endif
}

void Camera::capture_thread_work()
{
    while(m_run_capture)
    {
#ifndef NORPI
        m_capture->grab();
        pthread_mutex_lock(&m_frame_mutex);
        m_capture->retrieve(m_frame);
        rotateFrame(m_frame);
        pthread_cond_broadcast(&m_frame_cond);
        pthread_mutex_unlock(&m_frame_mutex);
#else
        usleep(16000);
        pthread_mutex_lock(&m_frame_mutex);
        pthread_cond_broadcast(&m_frame_cond);
        pthread_mutex_unlock(&m_frame_mutex);
#endif
    }
}

void Camera::rotateFrame(cv::Mat& frame)
{
    switch(m_rotation)
    {
        case 90:
            cv::transpose(frame, frame);
            cv::flip(frame, frame, 1);
            break;
        case 180:
            cv::flip(frame, frame, -1);
            break;
        case 270:
            cv::transpose(frame, frame);
            cv::flip(frame, frame, 0);
            break;
        case 0:
        default:
            return;
    }
}

void Camera::update(uint32_t diff)
{
    cvWaitKey(1);
}

void Camera::updateCamView()
{
    if(!m_show_gui)
        return;

    Mat tmp;
    const Scalar wColor = Scalar(0, 255, 0);

    pthread_mutex_lock(&m_frame_mutex);
    tmp = m_frame.clone();
    pthread_mutex_unlock(&m_frame_mutex);

    cvtColor(tmp, m_lastDisplayFrame, COLOR_BGR2HSV);

    line(tmp, Point(0, m_cut_y), Point(RES_X, m_cut_y), wColor);

    if(!m_cut_pts.empty())
    {
        const Point *p = (const cv::Point*) Mat(m_cut_pts).data;
        int size = Mat(m_cut_pts).rows;
        polylines(tmp, &p, &size, 1, false, wColor);
    }
    imshow("camera", tmp);
}

void Camera::capture(uint32_t idx)
{
    const int i = idx/DIFF_TYPE_MAX;
    const int type = idx%DIFF_TYPE_MAX; 
    if(i >= DIFF_MAX_CNT)
        return;

    pthread_mutex_lock(&m_frame_mutex);
    if(!m_frame.empty())
    {
        char name[128];
        snprintf(name, sizeof(name), "diff_%02d-%02d.jpg", i, type);
        imwrite(name, m_frame);
        //if(type == 0)
            //findWall(m_frame, i);
        cvtColor(m_frame, m_diffs[i][type], CV_BGR2GRAY);
        LOGD("Diff %02d-%02d captured", i, type);
    }
    pthread_mutex_unlock(&m_frame_mutex);

    if(type == 1)
    {
        if(m_diffs[i][0].empty())
        {
            LOGE("No diff 1");
            return;
        }

        Mat diff;
        subtract(m_diffs[i][0], m_diffs[i][1], diff);
        m_diffs[i][1] = diff;
        m_last_diff = i;
        m_bear[i] = find_bear(diff, i);
    }
}

bool Camera::isPointUnderCurve(const Point& p)
{
    if(m_cut_pts.empty())
        return true;

    int closest = 0;
    int dist = abs(m_cut_pts[0].x -  p.x);
    int tmp;
    for(size_t i = 1; i < m_cut_pts.size(); ++i)
    {
        tmp = abs(m_cut_pts[i].x - p.x);
        if(tmp < dist)
        {
            closest = i;
            dist = tmp;
        }
    }

    return p.y > m_cut_pts[closest].y;
}

void Camera::find_bear()
{
    if(m_last_diff != -1)
    {
        m_bear[m_last_diff] = find_bear(m_diffs[m_last_diff][1], m_last_diff);
    }
}

static bool sort_poly(const Point& a, const Point&b)
{
    return (a.y < b.y) || (a.y == b.y && a.x < b.x); 
}

void Camera::findWall(const cv::Mat& frame, int diffIdx)
{
    Mat imgHSV, imgThresholded;
    cvtColor(frame, imgHSV, COLOR_BGR2HSV);

    //const Scalar low(0, 24, 31);
    //const Scalar high(29, 196, 255);
    inRange(imgHSV, m_detectWallClr[0], m_detectWallClr[1], imgThresholded);

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    std::vector<std::vector<Point> > contours;
    findContours(imgThresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    int maxAreaIdx = -1;
    for(size_t i = 0; i < contours.size(); ++i)
    {
        double area = contourArea(contours[i]);
        if(area > maxArea)
        {
            maxAreaIdx = i;
            maxArea = area;
        }
    }

    const Scalar red(190, 255, 255);
    drawContours(imgThresholded, contours, maxAreaIdx, red, -1, 8);

    if(maxAreaIdx != -1)
    {
        vector<Point> poly;
        double peri = arcLength(Mat(contours[maxAreaIdx]), true);
        approxPolyDP(Mat(contours[maxAreaIdx]), poly, 0.02 * peri, true);

        vector<Point> cutPoints = poly;
        std::sort(cutPoints.begin(), cutPoints.end(), sort_poly);
        cutPoints.resize(2);
        cutPoints[0].y += 10;
        cutPoints[1].y += 10;

        for(size_t i = 0; i < poly.size(); ++i)
        {
            LOGD("[%d, %d]", poly[i].x, poly[i].y);
            circle(imgThresholded, poly[i], 5, m_detectWallClr[1], 5);
        }
        LOGD("CUT POINTS: [%d, %d], [%d, %d]", cutPoints[0].x, cutPoints[0].y, cutPoints[1].x, cutPoints[1].y);

        if(cutPoints[0].x > cutPoints[1].x)
            std::swap(cutPoints[0], cutPoints[1]);

        cutPoints.insert(cutPoints.begin(), Point(0, cutPoints[0].y));
        cutPoints.push_back(Point(RES_X, cutPoints[1].y));
        cutPoints.push_back(Point(RES_X, 0));
        cutPoints.push_back(Point(0, 0));

        m_mask[diffIdx].create(RES_Y, RES_X, CV_8UC1);
        m_mask[diffIdx].setTo(Scalar(255, 255, 255));
        fillConvexPoly(m_mask[diffIdx], cutPoints.data(), cutPoints.size(), Scalar(0, 0, 0));
        //fillConvexPoly(imgThresholded, cutPoints.data(), cutPoints.size(), m_detectWallClr[1]);
        imshow("mask", m_mask[diffIdx]);
    }

    if(m_show_gui)
        imshow("detect", imgThresholded);
}

cv::Rect Camera::find_bear(const cv::Mat& diff, int diffIdx)
{
    if(diff.empty())
        return Rect(-1, -1, -1, -1);

    Mat tmp;
    std::vector<std::vector<Point> > contours;
    std::vector<Rect> boundingRects;

    cv::bitwise_and(diff, m_mask[diffIdx], tmp);
    cv::threshold(tmp, tmp, m_threshold, 255, 0);

    //morphological opening (remove small objects from the foreground)
    erode(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    findContours(tmp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    boundingRects.resize(contours.size());

    double max = 0;
    int biggestIdx = -1;
    int lowestIdx = -1;
    int lowestHeight = 0;
    vector<Point> bigPoly;
    for(size_t i = 0; i < contours.size(); ++i)
    {
        double t = contourArea(contours[i]);
        vector<Point> poly;
        approxPolyDP(Mat(contours[i]), poly, 3, true);
        boundingRects[i] = boundingRect( Mat(poly) );

        //if(boundingRects[i].y + boundingRects[i].height/2 > m_cut_y && t > max)
        if(t > max)
        {
            Point c(boundingRects[i].x + boundingRects[i].width/2, boundingRects[i].y + boundingRects[i].height/2);
            if(isPointUnderCurve(c))
            {
                max = t;
                biggestIdx = i;
                poly.swap(bigPoly);
                //printf("area: %f\n", t);
            }
        }

        if(boundingRects[i].y + boundingRects[i].height > lowestHeight)
        {
            lowestHeight = boundingRects[i].y + boundingRects[i].height;
            lowestIdx = i;
        }
    }

    // Use only the top third - the head
    if(biggestIdx != -1)
    {
        int min_x = bigPoly[0].x;
        int max_x = bigPoly[0].x;
        int width = 0;
        const int top = (boundingRects[biggestIdx].y + int(boundingRects[biggestIdx].height*0.3));
        std::vector<Point> newPoly;
        std::sort(bigPoly.begin(), bigPoly.end(), sort_poly);
        for(size_t i = 0; i < bigPoly.size(); ++i)
        {
            min_x = std::min(min_x, bigPoly[i].x);
            max_x = std::max(max_x, bigPoly[i].x);
            LOGD("%03lu: %d %d - %d %d/%d", i, bigPoly[i].x, bigPoly[i].y, (max_x - min_x), width, int(width*1.2));
            //if(i > 5 && (max_x - min_x) > int(width*1.2))
            if(bigPoly[i].y >= top)
                break;
            width = max_x - min_x;
            newPoly.push_back(bigPoly[i]);    
        }
        boundingRects[biggestIdx] = boundingRect(Mat(newPoly));
    }

    if(m_show_gui)
    {
        Mat drawing = Mat::zeros( tmp.size(), CV_8UC3 );

        const Scalar cWhite = Scalar(255, 255, 255);
        const Scalar cGreen = Scalar(0, 255, 0);
        const Scalar cRed = Scalar(255, 0, 0);
        const Scalar cBlue = Scalar(0, 0, 255);
        const Scalar cYellow = Scalar(255, 255, 0);

        const Scalar *contClr = NULL;

        for(size_t i = 0; i < contours.size(); ++i)
        {
            const Rect& bRect = boundingRects[i];

            if((int)i == lowestIdx)
                contClr = &cYellow;
            else if(bRect.width >= bRect.height*2.5)
                contClr = &cRed;
            else if((int)i == biggestIdx)
                contClr = &cBlue;
            else
                contClr = &cWhite;

            drawContours(drawing, contours, i, *contClr, -1, 8);

            if((int)i == biggestIdx)
            {
                rectangle(drawing, bRect.tl(), bRect.br(), cGreen, 2, 8, 0);
                LOGD("Found: %d %d", bRect.x, bRect.y);
            }
        }

        imshow("diff", drawing);
    }

    if(biggestIdx != -1)
        return boundingRects[biggestIdx];
    return Rect(-1, -1, -1, -1);
}

void Camera::setShowGui(bool show)
{
    if(m_show_gui == show)
        return;

    m_show_gui = show;
    if(m_show_gui)
    {
        namedWindow("camera", CV_WINDOW_NORMAL);
        namedWindow("diff", CV_WINDOW_NORMAL);
        createTrackbar( "threshold", "diff", &m_threshold, 255, threshold_set);
        setMouseCallback("camera", mouseEvent, 0);
    }
    else
    {
        destroyAllWindows();
    }
}

void Camera::addCutPoint(int x, int y)
{
    m_cut_pts.push_back(Point(x, y));
}

void Camera::finalizeCutCurve()
{
    if(m_cut_pts.size() < 2)
        return;

    Point p = m_cut_pts[0];
    p.x = 0;
    m_cut_pts.insert(m_cut_pts.begin(), p);

    p = m_cut_pts.back();
    p.x = RES_X;
    m_cut_pts.push_back(p);
}

static bool startsWith(const std::string& haystack, const char *needle)
{
    const size_t len = strlen(needle);
    return haystack.size() > len &&
        haystack.compare(0, len, needle, len) == 0;
}

void Camera::setVar(const std::string& name, int val)
{
    if(name == "cgui")
        setShowGui(val);
    else if(name == "crotation")
        setRotation(val);
    else if(name == "cthreshold")
        setThreshold(val);
    else if(name == "ccamera")
    {
        if(val != 0)
        {
            open();
            waitForFrame(2);
        }
        else
            close();
    }
    else if(startsWith(name, "cwallclr"))
    {
        const int type = name[8]-'0';
        m_detectWallClr[type] = Scalar(val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF);
        LOGD("Setting wall color to [%d](%d, %d, %d)!", type, val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF);
    }
    else
        LOGE("Unknown var %s = %d", name.c_str(), val);
}

int Camera::getVar(const std::string& name)
{
    if(name == "cgui")
        return m_show_gui;
    else if(name == "crotation")
        return m_rotation;
    else if(name == "cthreshold")
        return m_threshold;
#ifndef NORPI
    else if(name == "ccamera")
        return int(m_capture != NULL);
#endif
    else
    {
        LOGE("Unknown var %s", name.c_str());
        return -1;
    }
}

void Camera::execAct(const std::string& name)
{
    if(name == "cupdate")
        updateCamView();
    else if(name == "creloadmask")
        loadMask();
    else if(startsWith(name, "cdiff"))
    {
        capture(name[5]-'0');

        Packet pkt(SMSG_ACT_RES);
        pkt << name;
        pkt << int32_t(1);
        sTcpServer.write(pkt);
    }
    else if(startsWith(name, "cber"))
    {
        const int type = name[4]-'0';
        const Rect& b = m_bear[type];

        LOGD("Bear %d at [%d;%d] %dx%d", type, b.x, b.y, b.width, b.height);

        Packet pkt(SMSG_ACT_RES);
        pkt << name;
        pkt << int16_t(b.x);
        pkt << int16_t(b.y);
        pkt << int16_t(b.width);
        pkt << int16_t(b.height);
        sTcpServer.write(pkt);

        if(m_show_gui)
            m_bear[type] = find_bear(m_diffs[type][1], type);
    }
    else
        LOGE("Unknown action %s", name.c_str());
}

void Camera::setRotation(int deg)
{
    if(m_rotation == deg)
        return;

    m_rotation = deg;

    // wait for new, rotated frame
    LOGD("waiting for rotated frame");
    waitForFrame(2);
}

bool Camera::waitForFrame(int timeout_sec)
{
    int res;
    struct timespec timeout;

#ifndef NORPI
    if(m_capture == NULL)
    {
        LOGD("Can't wait for frame, capture is not running");
        return false;
    }
#endif

    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 2;

    pthread_mutex_lock(&m_frame_mutex);
    res = pthread_cond_timedwait(&m_frame_cond, &m_frame_mutex, &timeout);
    pthread_mutex_unlock(&m_frame_mutex);

    if(res != 0)
        LOGE("wait failed with %d (%s)", res, strerror(res));
    return res == 0;
}

void Camera::loadMask()
{
    char buff[32];
    for(size_t i = 0; i < DIFF_MAX_CNT; ++i)
    {
        snprintf(buff, sizeof(buff), "mask%02d.png", i);
        Mat mask = imread(buff, CV_LOAD_IMAGE_GRAYSCALE);
        if(!mask.empty())
            m_mask[i] = mask;
    }
}

void Camera::printColorAt(int x, int y)
{
    Vec3b vec = m_lastDisplayFrame.at<Vec3b>(x, y);
    LOGD("Color at [%d, %d]: (%d, %d, %d)", x, y, vec[0], vec[1], vec[2]);
    vec = m_lastDisplayFrame.at<Vec3b>(y, x);
    LOGD("Color2 at [%d, %d]: (%d, %d, %d)", x, y, vec[0], vec[1], vec[2]);
}
