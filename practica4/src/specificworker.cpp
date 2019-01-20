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
	turnWay = Way::LEFT;
	
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
        if ( targ.getStatus() ) {
            //P1 = punto origen del robot
            //P2 = target.pick[index]
            //Ecuacion general de la recta: Ax + By + C = 0
            linear.A = -targ.getX() - bState.x;
            //A = -P2x - P1x
            //B = P2z - P1z
            linear.B = targ.getZ() - bState.z;
            linear.C = -(linear.A * bState.x + linear.B * bState.z);
            std::cout<<"A: "<<linear.A<<" B: "<<linear.B<<" C: "<<linear.C<<std::endl;
            //La recta la usamos para que el robot termine de rodear al obstáculo cuando se encuentra de nuevo en la recta entre el origen y el destino
            //calcular aqui el origen del robot y los valores A, B y C para el cálculo de la recta, que se usa enla función bug().
            stateWork = State::GOTO;
            std::cout<<"GOTO"<<std::endl;
        }
        break;
    case State::GOTO:
        goTarget();
        break;
    case State::BUG:
        bug(ldata);
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
	


    if(std::abs(rotAngle) > .1) {
	differentialrobot_proxy->setSpeedBase(0, rotAngle);	
    }
    else {
    	differentialrobot_proxy->setSpeedBase(1000, 0);

		if(obstacle(ldata, angle[1], angle[3])) {
			differentialrobot_proxy->setSpeedBase(0, 0);
			stateWork = State::BUG;
            selectDirectionBug(ldata);
            std::cout<<"BUG"<<std::endl;
			return;
        }
    }

    // If close to obstacle stop and transit to IDLE
    if(dir < threshold)
    {
	differentialrobot_proxy->setSpeedBase(0, 0);
        stateWork = State::IDLE;
        targ.toogleStatus();
        std::cout<<"IDLE"<<std::endl;
        return;

    }

}

/*void SpecificWorker::bug()
{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return     a.dist < b.dist;
    }) ;
    //Usar la recta calculada previamente (las variables A, B y C)
    //la recta permite al robot dejar de rodear al obstáculo cuando el robot se encuentra de nuevo en ella. (if( A * posRobot.x + B * posRobot.z + C == 0), terminar el bug).
    //En vez de hacer que el robot esté exactamente en la recta ( ecuacion == 0), le damos un margen de error tanto positivo como negativo ( < 1 y > -1 )
    if((linear.A * bState.x + linear.B * bState.z + linear.C) < 1 && (linear.A * bState.x + linear.B * bState.z + linear.C > -1))
    {
        stateWork = State::GOTO;
        return;
    }
	if(ldata.front().angle < 0) 
		differentialrobot_proxy->setSpeedBase(0, M_PI/5);
	else
		differentialrobot_proxy->setSpeedBase(0, -M_PI/5);
}*/

void SpecificWorker::bug (RoboCompLaser::TLaserData ldata){
	RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    linear.A = -targ.getX() - bState.x;
    //A = -P2x - P1x
    //B = P2z - P1z
    linear.B = targ.getZ() - bState.z;
    linear.C = -(linear.A * bState.x + linear.B * bState.z);

    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return     a.dist < b.dist;
    }) ;

    bool linearState = (linear.A * bState.x + linear.B * bState.z + linear.C) < 1 && (linear.A * bState.x + linear.B * bState.z + linear.C > -1);
    if(linearState && !obstacle(ldata, angle[0], angle[1]) ||linearState && !obstacle(ldata, angle[3], angle[4]) )
    {
         std::cout<<"In line with target "<<std::endl;
        stateWork = State::GOTO;
        return;
    }

    float rot = 1/2 + 1%2;
    float adv = 1000/2 + 1000%2;
    if(obstacle(ldata, angle[0]-10, angle[3]+5)){
        if(turnWay == Way::RIGHT){
            differentialrobot_proxy->setSpeedBase(0, -rot);
            return;
        }
        differentialrobot_proxy->setSpeedBase(0, rot);
        return;
    }
        differentialrobot_proxy->setSpeedBase(adv, 0);

    if(turnWay == Way::RIGHT && !obstacle(ldata, angle[0], angle[1])){
        differentialrobot_proxy->setSpeedBase(adv, rot);
        return;
    }
    if (turnWay == Way::LEFT && !obstacle(ldata, angle[3], angle[4])){
        differentialrobot_proxy->setSpeedBase(adv, -rot);
        return;
    }
}

bool SpecificWorker::obstacle(RoboCompLaser::TLaserData ldata, int start, int end){
  for(int i=start; i<= end; i++){
    if(ldata[i].dist < threshold+50)
      return true;
  }
  return false;
}

bool SpecificWorker::targetAtSight(RoboCompLaser::TLaserData ldata)
{
   QPolygonF polygon;
   QVec r = innerModel->transform("world", "base");
   polygon << QPointF(r.x(), r.z());
   for (int i = angle[1]; i < angle[3]; i++ ){
     QVec lr = innerModel->laserTo("world", "laser", ldata[i].dist, ldata[i].angle);
     polygon << QPointF(lr.x(), lr.z());
   }
   polygon << QPointF(r.x(), r.z());
   QPointF p(targ.getX(), targ.getZ()); 
 
   return  polygon.containsPoint( p , Qt::WindingFill );  
    return false;
}

//Select pick
void SpecificWorker::setPick(const Pick &myPick) {
    targ.setPick(myPick);
    std::cout<<"x:"<<myPick.x<<" z:"<<myPick.z <<std::endl;
}
void SpecificWorker::selectDirectionBug(RoboCompLaser::TLaserData ldata){
  for(int i = 0; i <= angle[2]-angle[1]; i++){
    if (ldata[angle[1]+i].dist < threshold){
      std::cout << ldata[angle[1]+i].dist << " take right" << endl;
      turnWay = Way::RIGHT;
      return;
    }
    if (ldata[angle[3]-i].dist < threshold){
      std::cout << ldata[angle[3]-i].dist << "take left" << endl;
      turnWay = Way::LEFT;
      return;
    }
  }
}

