#include"fileio.h"

ProjectInfo::ProjectInfo(){
   isInit = false;
   imagesNum = 0;
   isCalitration = false;
   isSfm = false;
   isDenseRec= false;
   isSurface= false;
}

void ProjectInfo::setProjectName(const string projectNamePara){
    projectName = projectNamePara;
}

void ProjectInfo::addImagePath(const string imagePathPara){
    imagePaths.push_back(imagePathPara);
    imagesNum++;
}

//imageIndex from 0
void ProjectInfo::deleteImagePath(int imageIndex){
    if(imageIndex < imagesNum){
        imagePaths.erase(imagePaths.begin()+ imageIndex);
        imagesNum--;
    }
}

