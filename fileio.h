#ifndef FILEIO_H
#define FILEIO_H

#include<string>
#include<vector>
using namespace std;
class ProjectInfo{
public:
    ProjectInfo();
    ~ProjectInfo();
    void setProjectName(string);
    void addImagePath(string);
    void deleteImagePath(int);
    void clearAllInfo();
    void setProject();
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




#endif // FILEIO_H
