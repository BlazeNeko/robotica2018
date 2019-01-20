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
    try
            {
      RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
      std::string innermodel_path = par.value;
                    innerModel = new InnerModel(innermodel_path);
     }
     catch(std::exception e) { qFatal("Error reading config params"); }
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
            //Calcula la linea de la ecuación
            linear.A = bState.z -targ.getZ();
            linear.B = targ.getX() - bState.x;
            linear.C= (bState.x * (linear.A*-1)) - (bState.z * linear.B);


            //Ecuacion general de la recta: Ax + By + C = 0
            //linear.C = (bState.x *targ.getZ() - targ.getX() * bState.z);
            //linear.C = -(linear.A * bState.x + linear.B * bState.z);
            std::cout<<"bstateX: "<<bState.x<<" bState.z: "<<bState.z<<std::endl;
            std::cout<<"A: "<<linear.A<<" B: "<<linear.B<<" C: "<<linear.C<<std::endl;
            std::cout<<(linear.A * bState.x + linear.B * bState.z + linear.C)<<std::endl;
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
	

    //comprueba si está en angulo del objetivo y gira en caso de que no.
    if(std::abs(rotAngle) > .1) {
	differentialrobot_proxy->setSpeedBase(0, rotAngle);	
    }
    else {
        //Por el contrario si está en linea recta aumenta
    	differentialrobot_proxy->setSpeedBase(1000, 0);
        //Si encuentra un obstaculo al frente entre los angulos 30-70 se dispone a rodearlo
		if(obstacle(ldata, angle[1], angle[3])) {
			differentialrobot_proxy->setSpeedBase(0, 0);
			stateWork = State::BUG;
            //Selecciona la dirección en la cual girar
            selectDirectionBug(ldata);
            //guarda la posición actual para que no se repita ni se quede en bucle
            lastPosition[0] = bState.x;
            lastPosition[1] = bState.z;
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

void SpecificWorker::bug (RoboCompLaser::TLaserData ldata){
	RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    //Calcula si está en linea al objetivo y si está en la misma posición que la última vez
    bool inL = inLine();
    bool samePosition = (lastPosition[0]== (int)bState.x && lastPosition[1] == (int)bState.z);
    //Si está en linea, en distinta posición que al entrar en el bug y sin obstaculo, se dirige al objetivo
    if(inL && !obstacle(ldata,angle[1],angle[3]) && !samePosition)
    {
        std::cout<<"In line with target "<<std::endl;
        std::cout<<"GOTO"<<std::endl;
        stateWork = State::GOTO;
        return;
    }
    float rot = 1/2 + 1%2;
    float adv = 1100/2 + 1100%2;
    if(obstacle(ldata, angle[0]-10, angle[3]+5)){
        if(turnWay == Way::RIGHT){
            differentialrobot_proxy->setSpeedBase(0, -rot);
            return;
        }
        differentialrobot_proxy->setSpeedBase(0, rot);
        return;
    }

    if(turnWay == Way::RIGHT && !obstacle(ldata, angle[0], angle[1])){
        differentialrobot_proxy->setSpeedBase(adv, rot);
        return;
    }
    if (turnWay == Way::LEFT && !obstacle(ldata, angle[3], angle[4])){
        differentialrobot_proxy->setSpeedBase(adv, -rot);
        return;
    }
    differentialrobot_proxy->setSpeedBase(adv, 0);

}
//Comprueba si hay obstaculo entre los angulos pasados
bool SpecificWorker::obstacle(RoboCompLaser::TLaserData ldata, int start, int end){
  for(int i=start; i<= end; i++){
    if(ldata[i].dist < threshold+50)
      return true;
  }
  return false;
}
//Comprueba si estás en un rango cercano a la linea
bool SpecificWorker::inLine() {
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
        double dist = fabs((linear.A*bState.x + linear.B*bState.z + linear.C))/sqrt((linear.A*linear.A) + (linear.B*linear.B));
        //double dist = ((a*bState.x) + (b*bState.z) + c);
        //dist = fabs(dist);
        qDebug()<<"LINE CROSS--------------------------------> " << dist;

        if (dist<30){
            qDebug()<<"Inside LINE CROSS--------------------------------> " << dist;
            return true;
        }
        return false;
}

//Select pick
void SpecificWorker::setPick(const Pick &myPick) {
    targ.setPick(myPick);
    std::cout<<"x:"<<myPick.x<<" z:"<<myPick.z <<std::endl;
}
//Selecciona la dirección en la que girar, viendo qué tiene más cerca si a la izquierda o derecha
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

