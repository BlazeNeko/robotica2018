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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innermodel_path = par.value;
		innermodel = new InnerModel(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	timer.start(Period);


	return true;
}

void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )

{

	RoboCompCommonBehavior::Parameter aux;

	aux.editable = true;

	configGetString( "","InnerModelPath", aux.value,"nofile");

	params["InnerModelPath"] = aux;

}


void SpecificWorker::compute()
{

	differentialrobot_proxy->getBaseState(bState);

	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 

	std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; }) ; //sort laser data from small to large distances using a lambda function.

	innermodel->updateTranslationPointers("base", bState.x, 0, bState.y ,0, bState.alpha, 0);

	switch( state ){

		case State::IDLE:
			if ( target.isActive() )
				//P1 = punto origen del robot
				//P2 = target.pick[index]
				//Ecuacion general de la recta: Ax + By + C = 0
				//A = -P2x - P1x 
				//B = P2z - P1z
				//C = -(A*P1x + B*P1z)
				//La recta la usamos para que el robot termine de rodear al obstáculo cuando se encuentra de nuevo en la recta entre el origen y el destino
				//calcular aqui el origen del robot y los valores A, B y C para el cálculo de la recta, que se usa enla función bug().
				state = State::GOTO;
		break;
		case State::GOTO:
			gotoTarget();
		break;
		case State::BUG:
			bug();
		break;
	}

}

void SpecificWorker::gotoTarget()

 {

    if( obstacle == true)   // If there is an obstacle ahead, then transit to BUG

   {

      state = State::BUG;

      return;

   }

    QVec rt = innermodel->transform("base", target.getPose(), "world");

    float dist = rt.norm();

    float ang  = atan2(rt.x(), rt.z());

   if(dist < 100)          // If close to obstacle stop and transit to IDLE

  {

    state = State::IDLE;

    target.setActive(true);

   return;

  }

  float adv = dist;

  if ( fabs( rot) > 0.05 )

   adv = 0;

 }

void SpecificWorker::bug()
{
	//Usar la recta calculada previamente (las variables A, B y C)
	//la recta permite al robot dejar de rodear al obstáculo cuando el robot se encuentra de nuevo en ella. (if( A * posRobot.x + B * posRobot.z + C == 0), terminar el bug).
	//En vez de hacer que el robot esté exactamente en la recta ( ecuacion == 0), le damos un margen de error tanto positivo como negativo ( < 1 y > -1 )
}

bool SpecificWorker::obstacle()

{

}

bool SpecificWorker::targetAtSight()

{

}



void SpecificWorker::setPick(const Pick &myPick)
{
//subscribesToCODE

}


