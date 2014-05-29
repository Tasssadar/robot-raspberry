#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include <pthread.h>
#include <vector>

#include "opencv2/core/core.hpp"

namespace cv {
    class VideoCapture;
};

namespace raspicam {
    class RaspiCam_Cv;
};

enum
{
    DIFF_TYPE_A = 0,
    DIFF_TYPE_B = 1,
    DIFF_TYPE_MAX = 2,
    
    DIFF_MAX_CNT = 2,
};

class Camera
{
public:
    static Camera& instance()
    {
        static Camera inst;
        return inst;
    }

    int threshold() const { return m_threshold; }
    void setThreshold(int t) { m_threshold = t; }
    void setShowGui(bool show);

    int cutY() const { return m_cut_y; }
    void setCutY(int y) { m_cut_y = y; }
    void clearCutPoints()
    {
        m_cut_pts.clear();
    }
    void addCutPoint(int x, int y);
    void finalizeCutCurve();
    bool isPointUnderCurve(const cv::Point& p);

    void updateCamView();

    void close();
    void open(int threshold = 0);

    void update(uint32_t diff);
    void capture_thread_work();

    void capture(uint32_t idx);
    void find_bear(const cv::Mat& diff);
    void find_bear();

private:
    Camera();
    virtual ~Camera();

    raspicam::RaspiCam_Cv *m_capture;
    pthread_t m_capture_thread;
    pthread_mutex_t m_frame_mutex;
    volatile bool m_run_capture;
    cv::Mat m_frame;
    cv::Mat m_diffs[DIFF_MAX_CNT][DIFF_TYPE_MAX];
    int m_threshold;
    bool m_show_gui;
    int m_cut_y;
    int m_last_diff;
    std::vector<cv::Point> m_cut_pts;
};

#define sCamera Camera::instance()
#endif