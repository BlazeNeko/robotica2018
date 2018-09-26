#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
        connect(&timer, SIGNAL(timeout()), this, SLOT(lcdUp()) );
        timer.start(500);
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::lcdUp()
{
    value=this->lcdNumber->value();
    
    this->lcdNumber->display(value+1);

}
void ejemplo1::doButton()
{
	if(timer.isActive()) {
            timer.stop();
        }
        else
            timer.start();
}




