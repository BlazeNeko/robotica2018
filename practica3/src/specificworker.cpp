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
#include <numeric>

/**
* \brief Default constructor
*/
target targ;
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
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

void SpecificWorker::setPick(const Pick &myPick){
	targ.setPick(myPick);
}


void SpecificWorker::compute()
{
	const float threshold = 150;
	
	// Obtener estado del mapa base	
	RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);

	// Obtener posición del robot
	Rot2D robotAngle(bState.alpha);
	auto position = robotAngle.invert() * (QVec::vec2(targ.getX() - bState.x, targ.getZ() - bState.z));

	// Obtener angulo y distancia
	float rotAngle = atan2(position.x(), position.y());
	float dir = position.norm2();
	
	if(dir < threshold) {
		differentialrobot_proxy->setSpeedBase(0, 0);
	}
	else if(std::abs(rotAngle) > .1) {
		differentialrobot_proxy->setSpeedBase(0, rotAngle);	
	}
	else {
		differentialrobot_proxy->setSpeedBase(1000, 0);
	}
}



