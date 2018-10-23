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
	QMutexLocker locker(mutex);
	targ.pick = myPick;
}

Pick SpecificWorker::getMyPick(){
	QMutexLocker locker(mutex);
	return targ.pick;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	static RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	//differentialrobot_proxy->setSpeedBase(bState);	

	std::cout << bState.x << " " << bState.z << std::endl;

	if(targ.isActive) {
		auto tw =getMyPick();
		Rot2D rot (bState.alpha);
		QVector3D y = QVector3D(getMyPick().x,0,getMyPick().z);
		QVector3D t = QVector3D(bState.x,0,bState.z);
		//auto r = rot.invert() * (y-t);
		//qDebug() << r;	

	}

}



