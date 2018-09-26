#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <QTimer>
class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT
    QTimer timer;
    int value = 0;
public:
    ejemplo1();
    virtual ~ejemplo1();
public slots:
	void doButton();
        void lcdUp();

};

#endif // ejemplo1_H
