/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : DataExchange.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_DATAEXCHANGE_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_DATAEXCHANGE_H_

#include <mujoco/mujoco.h>
#include <vector>
#include <Eigen/Dense>


class DataExchange {
public:
  DataExchange();
  void initialize(int dof);
  void exchangeData(const mjModel* m, mjData* d, bool isPositionControl);

  // 获取力传感器数据
  std::vector<Eigen::VectorXd> getForceTorqueData(const mjModel* m, mjData* d);

private:
  int dof = 0;
  unsigned long long int ticks = 0;

  void updateSimulationTime();
  void updateWaistData(const mjModel* m, mjData* d);
  void updateNeckData(const mjModel* m, mjData* d);
  void updateArmData(const mjModel* m, mjData* d);
  void applyPositionCommands(const mjModel* m, mjData* d);
  void applyTorqueCommands(const mjModel* m, mjData* d);
};

#endif // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_DATAEXCHANGE_H_
