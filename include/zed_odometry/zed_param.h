#ifndef PARAM_H_
#define PARAM_H_

#include <string>
#include <opencv2/opencv.hpp>

struct CALIBINFO{
    float fx;
    float fy;
    float cx;
    float cy;

    float k1;
    float k2;
    float p1;
    float p2;
};

class ZParam
{
private:
    ZParam(const std::string &source);
    ~ZParam(void);
public:
    static ZParam* getInstance(const std::string &source)
    {
        static ZParam *zInstance = NULL;
        if(zInstance == NULL)
        {
            zInstance = new ZParam(source);
        }

        return zInstance;
    }
    void initVariables();
public:
    struct CALIBINFO mCalibLeft, mCalibRight;  

    std::string mSource;

    int mwidth;
    int mheight;    

    float mBF; // Baseline times fx
    float mBaseline;
    float mCV; // RY
    float mRX; // RX
    float mRZ; // RZ

};

#endif /* PARAM_H_ */
