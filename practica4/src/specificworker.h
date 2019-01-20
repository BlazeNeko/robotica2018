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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
//
struct target {
    Pick pick;
    bool status = false;

    void setPick(Pick newPick) {
        status = true;
            pick = newPick;
    }
    float getX() {
        return pick.x;
    }
    float getZ() {
        return pick.z;
    }
    void toogleStatus() {
        status = !status;
    }
    bool getStatus() {
        return status;
    }
};
//
enum State {IDLE,GOTO,BUG};
//
struct line {
    float A;
    float B;
    float C;
};
//
enum Way {LEFT, RIGHT};

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    SpecificWorker(MapPrx& mprx);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    //
    void bug(RoboCompLaser::TLaserData ldata);
    //
    bool targetAtSight(RoboCompLaser::TLaserData ldata);
    //
    void goTarget();
    //
    void setPick(const Pick &myPick);
    //
    bool obstacle(RoboCompLaser::TLaserData ldata, int start, int end);
	//
	void selectDirectionBug(RoboCompLaser::TLaserData ldata);
	//
    bool inLine();
	
public slots:
    void compute();

private:
    //
    int threshold;
    //
    int angle[5] = {10,30,50,70,90};
	//
	Way turnWay;	
	//
    State stateWork;
    //
    InnerModel *innerModel;
    //
    target targ;
    //
    line linear;
	//
    int lastPosition[2];

	

};

#endif
