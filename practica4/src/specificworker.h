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
    Pick pick[10];
    int currentSize=0;
    int index=0;
    bool status = false;

    void setPick(Pick newPick) {
        status = true;
        if(currentSize < 10) {
            pick[currentSize] = newPick;
            currentSize++;
        }
    }
    float getX() {
        return pick[index].x;
    }
    float getZ() {
        return pick[index].z;
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


class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    SpecificWorker(MapPrx& mprx);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    //
    void bug();
    //
    bool obstacle();
    //
    bool targetAtSight();
    //
    void goTarget();
    //
    void setPick(const Pick &myPick);

public slots:
    void compute();

private:
    //
    int threshold;
    //
    State stateWork;
    //
    InnerModel *innerModel;
    //
    target targ;
    //
    line linear;

};

#endif
