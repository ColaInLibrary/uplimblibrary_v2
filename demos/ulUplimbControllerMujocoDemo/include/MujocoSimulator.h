/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-12
 * @Version       : 0.0.1
 * @File          : MujocoSimulator.h
 ******************************************************************************
 */

#ifndef UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_MUJOCOSIMULATOR_H_
#define UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_MUJOCOSIMULATOR_H_

#include <mujoco/mujoco.h>

class MujocoSimulator {
public:
  MujocoSimulator();
  ~MujocoSimulator();

  bool initialize(const char* modelPath);
  void step();
  void reset();
  void shutdown();

  mjModel* getModel() { return m_; }
  mjData* getData() { return d_; }

private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  bool loadModel(const char* modelPath);
  void applyInitialKeyframe();
  void printModelInfo();
  void analyzeGravityTorque();
};

#endif // UL_DEMOS_ULUPLIMBCONTROLLERMUJOCODEMO_MUJOCOSIMULATOR_H_
