#ifndef UTILS_H
#define UTILS_H
#include<QString>
#include<string>
#include<iostream>
#include<sstream>
#include<QDebug>
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

void debugstring(char* comment,string inputstring);

QString showBool(bool);

string getImageNameFromPath(string);

string saveBool(bool);

string getPathFromFullPath(string path);
#endif // UTILS_H


