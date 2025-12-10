/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-11-11
 * @Version       : 0.0.1
 * @File          : Body.cpp
 ******************************************************************************
 */

#include "Body.h"

#include <utility>


namespace ul::controller {
Body::Body(std::string bodyName, int dof) : name(std::move(bodyName)),
                                            joints_motion(dof),
                                            joints_security(dof),
                                            dof(dof) {}


const std::string &Body::getName() const { return this->name; }

int Body::getDof() const { return this->dof; }
}
