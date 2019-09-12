/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
//
// C++ Models: MechanicalStateController
//

#ifndef SOFA_COMPONENT_CONTROLLER_MECHANICALSTATECONTROLLER_INL
#define SOFA_COMPONENT_CONTROLLER_MECHANICALSTATECONTROLLER_INL

#include <SofaUserInteraction/MechanicalStateController.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/MouseEvent.h>
//#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/UpdateMappingVisitor.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

namespace sofa
{

namespace component
{

namespace controller
{
    using sofa::defaulttype::Quat;
    using sofa::defaulttype::Vec;

template <class DataTypes>
MechanicalStateController<DataTypes>::MechanicalStateController()
    : index( initData(&index, (unsigned int)0, "index", "Index of the controlled DOF") )
    , onlyTranslation( initData(&onlyTranslation, false, "onlyTranslation", "Controlling the DOF only in translation") )
    , buttonDeviceState(initData(&buttonDeviceState, false, "buttonDeviceState", "state of ths device button"))
    , mainDirection( initData(&mainDirection, sofa::defaulttype::Vec<3,Real>((Real)0.0, (Real)0.0, (Real)-1.0), "mainDirection", "Main direction and orientation of the controlled DOF") )
    , d_speedFactor(initData(&d_speedFactor, SReal(1.0), "speedFactor", "factor to increase/decrease the movements speed"))
{
    this->f_listening.setValue(true);
    mainDirection.beginEdit()->normalize();
    mainDirection.endEdit();
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::init()
{
    using core::behavior::MechanicalState;
    mState = dynamic_cast<MechanicalState<DataTypes> *> (this->getContext()->getMechanicalState());
    if (!mState)
        msg_error() << "MechanicalStateController has no binding MechanicalState";
    device = false;
}


template <class DataTypes>
void MechanicalStateController<DataTypes>::applyController()
{    
    if(device)
    {
        if(mState)
        {
//			if(mState->read(sofa::core::ConstVecCoordId::freePosition())->getValue())
            {
                helper::WriteAccessor<Data<VecCoord> > x = *this->mState->write(core::VecCoordId::position());
                helper::WriteAccessor<Data<VecCoord> > xfree = *this->mState->write(core::VecCoordId::freePosition());
                xfree[0].getCenter() = position;
                x[0].getCenter() = position;

                xfree[0].getOrientation() = m_orientation;
                x[0].getOrientation() = m_orientation;
            }
        }
        device = false;
    }

    if ( !onlyTranslation.getValue()  && ((mouseMode==BtLeft) || (mouseMode==BtRight)))
    {
        int dx = eventX - mouseSavedPosX;
        int dy = eventY - mouseSavedPosY;
        mouseSavedPosX = eventX;
        mouseSavedPosY = eventY;

        if (mState)
        {
            helper::WriteAccessor<Data<VecCoord> > x = *this->mState->write(core::VecCoordId::position());
            mState->vRealloc( sofa::core::MechanicalParams::defaultInstance(), core::VecCoordId::freePosition() ); // freePosition is not allocated by default
            helper::WriteAccessor<Data<VecCoord> > xfree = *this->mState->write(core::VecCoordId::freePosition());

            unsigned int i = index.getValue();

            Vec<3,Real> vx(1,0,0);
            Vec<3,Real> vy(0,1,0);
            Vec<3,Real> vz(0,0,1);

            if (mouseMode==BtLeft)
            {
                xfree[i].getOrientation() = x[i].getOrientation() * Quat(vy, dx * (Real)0.001) * Quat(vz, dy * (Real)0.001);
                x[i].getOrientation() = x[i].getOrientation() * Quat(vy, dx * (Real)0.001) * Quat(vz, dy * (Real)0.001);
            }
            else
            {
                sofa::helper::Quater<Real>& quatrot = x[i].getOrientation();
                sofa::defaulttype::Vec<3,Real> vectrans(dy * mainDirection.getValue()[0] * (Real)0.05, dy * mainDirection.getValue()[1] * (Real)0.05, dy * mainDirection.getValue()[2] * (Real)0.05);
                vectrans = quatrot.rotate(vectrans);

                x[i].getCenter() += vectrans;
                x[i].getOrientation() = x[i].getOrientation() * Quat(vx, dx * (Real)0.001);

                //	x0[i].getCenter() += vectrans;
                //	x0[i].getOrientation() = x0[i].getOrientation() * Quat(vx, dx * (Real)0.001);

                if(xfree.size() > 0)
                {
                    xfree[i].getCenter() += vectrans;
                    xfree[i].getOrientation() = x[i].getOrientation() * Quat(vx, dx * (Real)0.001);
                }
            }
        }
    }
    else if( onlyTranslation.getValue() )
    {
        if( mouseMode )
        {
            int dx = eventX - mouseSavedPosX;
            int dy = eventY - mouseSavedPosY;
            mouseSavedPosX = eventX;
            mouseSavedPosY = eventY;

// 			Real d = sqrt(dx*dx+dy*dy);
// 			if( dx<0 || dy<0 ) d = -d;

            if (mState)
            {
                helper::WriteAccessor<Data<VecCoord> > x = *this->mState->write(core::VecCoordId::position());

                unsigned int i = index.getValue();

                switch( mouseMode )
                {
                case BtLeft:
                    x[i].getCenter() += Vec<3,Real>((Real)dx,(Real)0,(Real)0);
                    break;
                case BtRight :
                    x[i].getCenter() += Vec<3,Real>((Real)0,(Real)dy,(Real)0);
                    break;
                case BtMiddle :
                    x[i].getCenter() += Vec<3,Real>((Real)0,(Real)0,(Real)dy);
                    break;
                default :
                    break;
                }
            }
        }
    }


    sofa::simulation::Node *node = static_cast<sofa::simulation::Node*> (this->getContext());
    sofa::simulation::MechanicalProjectPositionAndVelocityVisitor mechaProjectVisitor(core::MechanicalParams::defaultInstance()); mechaProjectVisitor.execute(node);
    sofa::simulation::MechanicalPropagateOnlyPositionAndVelocityVisitor mechaVisitor(core::MechanicalParams::defaultInstance()); mechaVisitor.execute(node);
    sofa::simulation::UpdateMappingVisitor updateVisitor(core::ExecParams::defaultInstance()); updateVisitor.execute(node);
};


template <class DataTypes>
void MechanicalStateController<DataTypes>::applyTranslation(sofa::defaulttype::Vec3 translation)
{
    if (mState)
    {
        helper::WriteAccessor<Data<VecCoord> > x = *this->mState->write(core::VecCoordId::position());

        unsigned int i = index.getValue();
        const SReal& factor = d_speedFactor.getValue();
        x[i].getCenter() += translation * factor;
    }
}


template <class DataTypes>
bool MechanicalStateController<DataTypes>::worldToLocal(sofa::defaulttype::Vec3& vector)
{
    if (mState)
    {
        helper::WriteAccessor<Data<VecCoord> > x = *this->mState->write(core::VecCoordId::position());
        unsigned int i = index.getValue();
        m_orientation = x[i].getOrientation();
        vector = m_orientation.rotate(vector);
        return true;
    }
    return false;
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::moveUp()
{
    Vec3 vec(0, 1, 0);
    worldToLocal(vec);
    msg_info() << "(0, 1, 0)  -> " << vec;
    applyTranslation(vec);
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::moveDown()
{
    Vec3 vec(0, -1, 0);
    worldToLocal(vec);
    msg_info() << "(0, -1, 0)  -> " << vec;
    applyTranslation(vec);
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::moveLeft()
{
    Vec3 vec(-1, 0, 0);
    worldToLocal(vec);
    msg_info() << "(-1, 0, 0)  -> " << vec;
    applyTranslation(vec);
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::moveRight()
{
    Vec3 vec(1, 0, 0);
    worldToLocal(vec);
    msg_info() << "(1, 0, 0)  -> " << vec;
    applyTranslation(vec);
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::moveForward()
{
    Vec3 vec(0, 0, -1);
    worldToLocal(vec);
    msg_info() << "(1, 0, 0)  -> " << vec;
    applyTranslation(vec);
}

template <class DataTypes>
void MechanicalStateController<DataTypes>::moveBackward()
{
    Vec3 vec(0, 0, 1);
    worldToLocal(vec);
    msg_info() << "(-1, 0, 0)  -> " << vec;
    applyTranslation(vec);
}


template <class DataTypes>
void MechanicalStateController<DataTypes>::onBeginAnimationStep(const double /*dt*/)
{
    buttonDevice=buttonDeviceState.getValue();
    applyController();
}



template <class DataTypes>
core::behavior::MechanicalState<DataTypes> *MechanicalStateController<DataTypes>::getMechanicalState() const
{
    return mState;
}



template <class DataTypes>
void MechanicalStateController<DataTypes>::setMechanicalState(core::behavior::MechanicalState<DataTypes> *_mState)
{
    mState = _mState;
}



template <class DataTypes>
unsigned int MechanicalStateController<DataTypes>::getIndex() const
{
    return index.getValue();
}



template <class DataTypes>
void MechanicalStateController<DataTypes>::setIndex(const unsigned int _index)
{
    index.setValue(_index);
}



template <class DataTypes>
const sofa::defaulttype::Vec<3, typename MechanicalStateController<DataTypes>::Real > &MechanicalStateController<DataTypes>::getMainDirection() const
{
    return mainDirection.getValue();
}



template <class DataTypes>
void MechanicalStateController<DataTypes>::setMainDirection(const sofa::defaulttype::Vec<3,Real> _mainDirection)
{
    mainDirection.setValue(_mainDirection);
}



template <class DataTypes>
void MechanicalStateController<DataTypes>::onMouseEvent(core::objectmodel::MouseEvent* /*mev*/)
{

}

template <class DataTypes>
void MechanicalStateController<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::KeypressedEvent::checkEventType(event))
    {
        sofa::core::objectmodel::KeypressedEvent* ke = static_cast<sofa::core::objectmodel::KeypressedEvent*>(event);
        msg_info() << "MechanicalStateController handleEvent gets character '" << ke->getKey() << "'. ";

        if (ke->getKey() == '+')
            moveForward();
        else if (ke->getKey() == '-')
            moveForward();
        else if (ke->getKey() == '8')
            moveUp();
        else if (ke->getKey() == '2')
            moveDown();
        else if (ke->getKey() == '4')
            moveLeft();
        else if (ke->getKey() == '6')
            moveRight();
    }
}



template <>
SOFA_USER_INTERACTION_API void MechanicalStateController<defaulttype::Vec1Types>::applyController();

template <>
SOFA_USER_INTERACTION_API void MechanicalStateController<defaulttype::Vec1Types>::applyTranslation(sofa::defaulttype::Vec3 translation);

template <>
SOFA_USER_INTERACTION_API bool MechanicalStateController<defaulttype::Vec1Types>::worldToLocal(sofa::defaulttype::Vec3& vector);

template <>
SOFA_USER_INTERACTION_API void MechanicalStateController<defaulttype::Vec1Types>::onMouseEvent(core::objectmodel::MouseEvent *mev);

template <>
SOFA_USER_INTERACTION_API void MechanicalStateController<defaulttype::Rigid3Types>::onMouseEvent(core::objectmodel::MouseEvent *mev);



} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_MECHANICALSTATECONTROLLER_H
