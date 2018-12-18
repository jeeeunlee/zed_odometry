
#ifndef OPTIMIZER_H
#define OPTIMIZER_H



#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


class Optimizer{

public:
    Optimizer();
    void BundleAdjustment();

};

#endif // OPTIMIZER_H