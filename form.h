#ifndef FORM_H
#define FORM_H

#include <QWidget>
#include <QDebug>
#include <QPainter>
namespace Ui {
class Form;
}

class Form : public QWidget
{
    Q_OBJECT
private:
    QList<QList<double>> *path;
    QList<QList<double>> path_;

private slots:
    void showPath(QList<QList<double>> * p);//获取整体路径
    void appendPath(double x,double y);//获取路径点

public:
    explicit Form(QWidget *parent = nullptr);
    void paintEvent(QPaintEvent *event) override;
    ~Form();

private:
    Ui::Form *ui;
};

#endif // FORM_H
