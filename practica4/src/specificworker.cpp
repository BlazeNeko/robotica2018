/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include "specificmonitor.h"
#include <innermodel/innermodel.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

    stateWork = State::IDLE;
    threshold = 250;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    timer.start(Period);
    return true;
}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);

    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    //read laser data

    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    }) ;

    switch( stateWork ) {

    case State::IDLE:
        std::cout<<"IDLE"<<std::endl;
        if ( targ.getStatus() ) {
            //P1 = punto origen del robot
            //P2 = target.pick[index]
            //Ecuacion general de la recta: Ax + By + C = 0
            linear.A = -targ.getX() - bState.x;
            //A = -P2x - P1x
            //B = P2z - P1z
            linear.B = targ.getZ() - bState.z;
            linear.C = -(linear.A * bState.x + linear.B * bState.z);
            //La recta la usamos para que el robot termine de rodear al obstáculo cuando se encuentra de nuevo en la recta entre el origen y el destino
            //calcular aqui el origen del robot y los valores A, B y C para el cálculo de la recta, que se usa enla función bug().
            stateWork = State::GOTO;
        }
        break;
    case State::GOTO:
        goTarget();
        break;
    case State::BUG:
        bug();
        break;
    }
}

void SpecificWorker::goTarget()

{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);

    Rot2D robotAngle(bState.alpha);
    auto position = robotAngle.invert() * (QVec::vec2(targ.getX() - bState.x, targ.getZ() - bState.z));
    //Direction to change
    float rotAngle = atan2(position.x(), position.y());
    float dir = position.norm2();
    //Laser Data
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return     a.dist < b.dist;
    }) ;

    // If there is an obstacle ahead, then transit to BUG
    if( ldata.front().dist < threshold)
    {
        differentialrobot_proxy->setSpeedBase(0, 0);
        stateWork = State::BUG;
        return;
    }
    // If close to obstacle stop and transit to IDLE
    if(dir < 100)
    {
        stateWork = State::IDLE;
        targ.toogleStatus();
        return;

    }

}

void SpecificWorker::bug()
{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    //Usar la recta calculada previamente (las variables A, B y C)
    //la recta permite al robot dejar de rodear al obstáculo cuando el robot se encuentra de nuevo en ella. (if( A * posRobot.x + B * posRobot.z + C == 0), terminar el bug).
    //En vez de hacer que el robot esté exactamente en la recta ( ecuacion == 0), le damos un margen de error tanto positivo como negativo ( < 1 y > -1 )
    if((linear.A * bState.x + linear.B * bState.z + linear.C) < 1 && (linear.A * bState.x + linear.B * bState.z + linear.C > -1))
    {
        stateWork = State::GOTO;
        return;
    }
    else {

    }
}

bool SpecificWorker::obstacle()

{

}

bool SpecificWorker::targetAtSight()

{

}
// add new pick to waylist
void SpecificWorker::setPick(const Pick &myPick) {
    targ.setPick(myPick);
}


