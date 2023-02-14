#ifndef UTILS_H
#define UTILS_H
#include<QString>
#include<string>
#include<iostream>
#include<sstream>
using namespace std;
QString str2qstr(const string str);
string qstr2str(const QString qstr);
//数据类型转换模板函数
template <class Type>
Type stringToNum(const string str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

#endif // UTILS_H
