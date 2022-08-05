#include "ImpedanceController.h"
#include "MathUtil.h"

bool ImpedanceController::calcImpedanceControl(double dt, const GaitParam& gaitParam,
                                               std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_icEEOffset /*generate frame, endeffector origin*/) const{
  for(int i=0;i<gaitParam.eeName.size();i++){
    if(!this->isImpedanceMode[i])  continue;

    cnoid::Vector6 offsetPrev; // generate frame. endEffector origin
    cnoid::Vector6 dOffsetPrev; // generate frame. endEffector origin
    gaitParam.icEEOffset[i].value(offsetPrev, dOffsetPrev);

    cnoid::Matrix3 eeR = cnoid::AngleAxisd(offsetPrev.tail<3>().norm(),(offsetPrev.tail<3>().norm()>0)?offsetPrev.tail<3>().normalized() : cnoid::Vector3::UnitX()) * gaitParam.refEEPose[i].linear();

    cnoid::Vector6 refWrenchLocal; //endEffector frame. endeffector origin
    refWrenchLocal.head<3>() = eeR.transpose() * gaitParam.refEEWrench[i].head<3>();
    refWrenchLocal.tail<3>() = eeR.transpose() * gaitParam.refEEWrench[i].tail<3>();

    cnoid::Vector6 actWrenchLocal; //endEffector frame. endeffector origin
    actWrenchLocal.head<3>() = eeR.transpose() * gaitParam.actEEWrench[i].head<3>();
    actWrenchLocal.tail<3>() = eeR.transpose() * gaitParam.actEEWrench[i].tail<3>();

    cnoid::Vector6 offsetPrevLocal; //endEffector frame. endeffector origin
    offsetPrevLocal.head<3>() = eeR.transpose() * offsetPrev.head<3>();
    offsetPrevLocal.tail<3>() = eeR.transpose() * offsetPrev.tail<3>();

    cnoid::Vector6 dOffsetPrevLocal; //endEffector frame. endeffector origin
    dOffsetPrevLocal.head<3>() = eeR.transpose() * dOffsetPrev.head<3>();
    dOffsetPrevLocal.tail<3>() = eeR.transpose() * dOffsetPrev.tail<3>();

    cnoid::Vector6 dOffsetLocal; //endEffector frame. endeffector origin
    for(size_t j=0;j<6;j++){
      if(this->M[i][j] == 0.0 && this->D[i][j] == 0.0 && this->K[i][j]==0.0){
        dOffsetLocal[j] = 0.0;
        continue;
      }

      dOffsetLocal[j] =
        ((actWrenchLocal[j] - refWrenchLocal[j]) * this->wrenchGain[i][j] * dt * dt
         - this->K[i][j] * offsetPrevLocal[j] * dt * dt
         + this->M[i][j] * dOffsetPrevLocal[j]*dt)
        / (this->M[i][j] + this->D[i][j] * dt + this->K[i][j] * dt * dt);
    }

    cnoid::Vector6 dOffset; //generate frame. endeffector origin
    dOffset.head<3>() = eeR * dOffsetLocal.head<3>();
    dOffset.tail<3>() = eeR * dOffsetLocal.tail<3>();

    cnoid::Vector6 offset = offsetPrev + dOffset;
    offset = mathutil::clampMatrix<cnoid::Vector6>(offset, this->compensationLimit[i]);
    o_icEEOffset[i].reset(offset, dOffset/dt);
  }

  return true;
}