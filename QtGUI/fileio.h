#ifndef FILEIO_H
#define FILEIO_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <QWidget>
#include <QImage>
#include <QFileInfo>
#include <QFileDialog>
#include <QDebug>
#include <QWidget>
#include"utils.h"
using namespace std;
class ProjectInfo{
public:
    ProjectInfo();
    ~ProjectInfo(){}
    void setProjectName(string);
    void addImagePath(string);
    void deleteImagePath(int);
    void clearAllInfo();

    bool isInit;
    string projectName;
    int imagesNum;
    vector<string> imagePaths;
    bool isCalitration;
    vector<double> KMatrix;
    bool isSfm;
    string sfmResultPath;
    bool isDenseRec;
    string denseRecResultPath;
    bool isSurface;
    string surfacePath;
};

class FileIO
{
public:
    FileIO(){}
    ~FileIO(){}
    void setFilePath(string);
    ProjectInfo* getInfoFromFile(QWidget*);
    void printInfoTofile(ProjectInfo*);
private:
    string filePath;
    std::ifstream ifstrm;
    std::ofstream ofstrm;
};


#endif // FILEIO_H
