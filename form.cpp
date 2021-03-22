#include "form.h"
#include "ui_form.h"

void Form::showPath(QList<QList<double> > *p)
{
    path = p;
    repaint();
}

void Form::appendPath(double x, double y)
{
    path_.append({x,y});
    repaint();
}

Form::Form(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Form)
{
    ui->setupUi(this);
    path = nullptr;
    path_.clear();
}

void Form::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    if(path!=nullptr)//路径
    {
        QPainter painter(this);
        QPen pen;
        pen.setWidth(2);
        pen.setColor(Qt::black);
        painter.setPen(pen);

        for(int i=0;i<path->size();i++)
        {
            double x = path->at(i).at(0)/10.0;
            double y = path->at(i).at(1)/10.0;
            QRect rect(x-2,y-2,4,4);
            painter.drawEllipse(rect);
        }
    }

    if(!path_.isEmpty())//路径点
    {
        QPainter painter(this);
        QPen pen;
        pen.setWidth(2);
        pen.setColor(Qt::blue);
        painter.setPen(pen);

        for(int i=0;i<path_.size();i++)
        {
            double x = path_.at(i).at(0)*50+400;
            double y = path_.at(i).at(1)*50+400;
            QRect rect(x-2,y-2,4,4);
            painter.drawEllipse(rect);
        }

    }
}

Form::~Form()
{
    delete ui;
}
