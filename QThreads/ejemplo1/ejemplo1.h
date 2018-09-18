#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include "timer.h"
class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
public:
    Timer timer;
    int value = 0;
    ejemplo1();
    virtual ~ejemplo1();
public slots:
	void doButton();
        void lcdUp();

};

#endif // ejemplo1_H
