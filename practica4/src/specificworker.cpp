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

   laserData = laser_proxy->getLaserData();

  innermodel->updateTranslationPointers("base", bState.x, 0, bState.y ,0, bState.alpha, 0);

  switch( state )

 {

   case State::IDLE:

   if ( target.isActive() )

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

    if( obstacle == true)   // If ther is an obstacle ahead, then transit to BUG

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


