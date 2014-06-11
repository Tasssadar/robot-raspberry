#include <assert.h>

#include <stdio.h>
#include <vector>
#include <stdlib.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "camera.h"
#include "tcpserver.h"

#include <raspicam/raspicam_cv.h>

using namespace cv;

#define RES_X 320
#define RES_Y 240

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
        sCamera.updateCamView();
    }

    if(flags & CV_EVENT_FLAG_LBUTTON)
    {
        sCamera.addCutPoint(x, y);
        sCamera.updateCamView();
    }
}

Camera::Camera()
{
    m_capture = NULL;
    m_run_capture = false;
    m_threshold = 70;
    m_show_gui = false;
    m_last_diff = -1;

    pthread_mutex_init(&m_frame_mutex, 0);
}

Camera::~Camera()
{
    close();
}

void Camera::open(int threshold)
{
    assert(m_capture == NULL);

    m_capture = new raspicam::RaspiCam_Cv();
    m_capture->set(CV_CAP_PROP_FRAME_WIDTH, RES_X);
    m_capture->set(CV_CAP_PROP_FRAME_HEIGHT, RES_Y);

    if (!m_capture->open()) {
        fprintf(stderr, "Camera: capture is NULL \n");
        delete m_capture;
        m_capture = NULL;
        return;
    }

    m_capture->set(CV_CAP_PROP_FRAME_WIDTH, RES_X);
    m_capture->set(CV_CAP_PROP_FRAME_HEIGHT, RES_Y);

    m_run_capture = true;
    pthread_create(&m_capture_thread, 0, camera_run_capture_thread, this);

    if(threshold > 0)
        setThreshold(threshold);
}

void Camera::close()
{
    if(m_capture == NULL)
        return;

    m_run_capture = false;
    pthread_join(m_capture_thread, 0);

    m_capture->release();
    delete m_capture;
    m_capture = NULL;
}

void Camera::capture_thread_work()
{
    while(m_run_capture)
    {
        m_capture->grab();
        pthread_mutex_lock(&m_frame_mutex);
        m_capture->retrieve(m_frame);
        rotateFrame(m_frame);
        pthread_mutex_unlock(&m_frame_mutex);
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
    static const Scalar wColor = Scalar(0, 255, 0);

    pthread_mutex_lock(&m_frame_mutex);
    tmp = m_frame.clone();
    pthread_mutex_unlock(&m_frame_mutex);

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

    char buff[64];
    snprintf(buff, sizeof(buff), "img_%02d_%02d.bmp", i, type); 
    pthread_mutex_lock(&m_frame_mutex);
    if(!m_frame.empty())
    {
        imwrite(buff, m_frame);
        cvtColor(m_frame, m_diffs[i][type], CV_RGB2GRAY);
        printf("Diff %02d-%02d captured\n", i, type);
    }
    pthread_mutex_unlock(&m_frame_mutex);

    if(type == 1)
    {
        if(m_diffs[i][0].empty())
        {
            printf("No diff 1\n");
            return;
        }
        
        Mat diff;
        subtract(m_diffs[i][0], m_diffs[i][1], diff);
        m_diffs[i][1] = diff;
        m_last_diff = i;
        find_bear(diff);
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
        find_bear(m_diffs[m_last_diff][1]);
}

static bool sort_poly(const Point& a, const Point&b)
{
    return (a.y < b.y) || (a.y == b.y && a.x < b.x); 
}
void Camera::find_bear(const cv::Mat& diff)
{
    if(diff.empty())
        return;

    Mat tmp;
    std::vector<std::vector<Point> > contours;
    std::vector<Rect> boundingRects;
    cv::threshold(diff, tmp, m_threshold, 255, 0);

    findContours(tmp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    boundingRects.resize(contours.size());

    double max = 0;
    int maxCnt = -1;
    vector<Point> bigPoly;
    for(int i = 0; i < contours.size(); ++i)
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
                maxCnt = i;
                poly.swap(bigPoly);
                //printf("area: %f\n", t);
            }
        }
    }

    // Use only the top third - the head    
    if(maxCnt != -1)
    {
        int min_x = bigPoly[0].x;
        int max_x = bigPoly[0].x;
        int width = 0;
        const int top = (boundingRects[maxCnt].y + int(boundingRects[maxCnt].height*0.3));
        std::vector<Point> newPoly;
        std::sort(bigPoly.begin(), bigPoly.end(), sort_poly);
        for(size_t i = 0; i < bigPoly.size(); ++i)
        {
            
            min_x = std::min(min_x, bigPoly[i].x);
            max_x = std::max(max_x, bigPoly[i].x);
            printf("%03d: %d %d - %d %d/%d\n", i, bigPoly[i].x, bigPoly[i].y, (max_x - min_x), width, int(width*1.2));
            //if(i > 5 && (max_x - min_x) > int(width*1.2))
            if(bigPoly[i].y >= top)
                break;
            width = max_x - min_x;
            newPoly.push_back(bigPoly[i]);    
        }
        boundingRects[maxCnt] = boundingRect(Mat(newPoly));
    }

    if(!m_show_gui)
    {
        if(maxCnt == -1)
            printf("Not found\n");
        else
        {
            const Rect& bRect = boundingRects[maxCnt];
            printf("\nFound: %d %d\n", bRect.x + bRect.width/2, bRect.y + bRect.height/2);
            sTcpServer.write("bear %d %d %d %d\n", bRect.x, bRect.y, bRect.width, bRect.height);
        }
    }
    else
    {
#if 0
        Mat drawing = Mat::zeros( tmp.size(), CV_8UC3 );
        static const Scalar bColor = Scalar(0, 0, 255);
        static const Scalar wColor = Scalar(0, 255, 0);
        static const Scalar color = Scalar(255, 255, 255);
        for( int i = 0; i< contours.size(); i++ )
        {
            const Rect& bRect = boundingRects[i];

            if(maxCnt == i)
            {
                printf("\nFound: %d %d\n", bRect.x + bRect.width/2, bRect.y + bRect.height/2);
                sTcpServer.write("bear %d %d %d %d\n", bRect.x, bRect.y, bRect.width, bRect.height);
            }

            drawContours( drawing, contours, i, maxCnt == i ? wColor : color, 3, 8);
            rectangle(drawing, bRect.tl(), bRect.br(), bColor, 2, 8, 0);
        }
        line(drawing, Point(0, m_cut_y), Point(RES_X, m_cut_y), wColor);
        imshow("diff", drawing);
#else
        Mat drawing = Mat::zeros( tmp.size(), CV_8UC3 );
        static const Scalar color = Scalar(255, 255, 255);

        drawContours( drawing, contours, maxCnt, color, -1, 8);
        {
            static const Scalar gColor = Scalar(0, 255, 0);
            const Rect& bRect = boundingRects[maxCnt];
            rectangle(drawing, bRect.tl(), bRect.br(), gColor, 2, 8, 0);
        }
        
        imshow("diff", drawing);
        imwrite("diff.bmp", drawing);

        if(maxCnt != -1) {
            const Rect& bRect = boundingRects[maxCnt];
            printf("\nFound: %d %d\n", bRect.x + bRect.width/2, bRect.y + bRect.height/2);
            sTcpServer.write("bear %d %d %d %d\n", bRect.x, bRect.y, bRect.width, bRect.height);
        }
#endif
    }
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

void Camera::setVar(const std::string& name, int val)
{
    if(name == "cgui")
        setShowGui(val);
    else if(name == "crotation")
        setRotation(val);
    else if(name == "cthreshold")
        setThreshold(val);
}

int Camera::getVar(const std::string& name)
{
    if(name == "cgui")
        return m_show_gui;
    else if(name == "crotation")
        return m_rotation;
    else if(name == "cthreshold")
        return m_threshold;
}
