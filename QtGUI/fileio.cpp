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

void ProjectInfo::clearAllInfo(){
    isInit = false;
    projectName = "";
    imagesNum = 0;
    imagePaths.clear();
    isCalitration = false;
    KMatrix.clear();
    isSfm = false;
    isDenseRec= false;
    isSurface= false;
}


void FileIO::setFilePath(std::string fileName) {
    qDebug() << "fileName" <<fileName.c_str();
    filePath = fileName;
}

ProjectInfo* FileIO::getInfoFromFile(QWidget* Rec)
{
    QString OpenFile, OpenFilePath;
    OpenFile = QFileDialog::getOpenFileName(Rec,
                                            "please choose an image file",
                                            "",
                                            "Image Files(*.txt);;All(*.*)");
    char* fp = OpenFile.toUtf8().data();
    std::string s = fp;
    filePath = s;
    ProjectInfo* projectInfo = new ProjectInfo();
    ifstrm.open(filePath);
    std::vector<std::vector<std::string> > slist;
    std::string line;
    while (std::getline(ifstrm, line)) {
        std::vector<std::string> vlist;
        std::string word;
        std::istringstream ists(line);
        while (ists >> word)
        {
            vlist.push_back(word);
        }
        slist.push_back(vlist);
    }
    projectInfo->isInit = true;
    projectInfo->projectName = slist[0][1];
    projectInfo->imagesNum = atoi(slist[1][1].c_str());
    unsigned _index = 2 + unsigned(projectInfo->imagesNum);
    for(unsigned i = 2;i < _index; i++){
        projectInfo->imagePaths.push_back(slist[i][0]);
    }
    if(atoi(slist[_index][1].c_str()) == 1){
        projectInfo->isCalitration = true;
        for(auto item :slist[_index+1]){
            projectInfo->KMatrix.push_back(stringToNum<double>(item));
        }
        _index += 2;
    }else {
        projectInfo->isCalitration = false;
        _index += 1;
    }
    if(atoi(slist[_index][1].c_str()) == 1){
        projectInfo->isSfm = true;
        projectInfo->sfmResultPath = slist[_index][2];
        _index++;
    }else {
    projectInfo->isSfm = false;
    _index++;
    }
    if(atoi(slist[_index][1].c_str()) == 1){
        projectInfo->isDenseRec = true;
        projectInfo->denseRecResultPath = slist[_index][2];
        _index++;
    }else {
    projectInfo->isDenseRec = false;
    _index++;
    }
    if(atoi(slist[_index][1].c_str()) == 1){
        projectInfo->isSurface = true;
        projectInfo->surfacePath = slist[_index][2];
        _index++;
    }else {
    projectInfo->isSurface = false;
    _index++;
    }
    ifstrm.close();
    return projectInfo;
}

void FileIO::printInfoTofile(ProjectInfo* projectInfo){
    ofstrm.open(filePath);
    ofstrm << "projectName" <<" "<< projectInfo->projectName<<std::endl;
    ofstrm << "imagesNum" << " " << projectInfo->imagesNum << std::endl;
    for(auto item :projectInfo->imagePaths){
        ofstrm << item << std::endl;
    }
    ofstrm << "isCalibration" <<" "<< saveBool(projectInfo->isCalitration)<<endl;
    if(projectInfo->isCalitration){
        for(auto item :projectInfo->KMatrix){
            ofstrm << item << " ";
        }
        ofstrm<<std::endl;
    }
    if(!projectInfo->isSfm){
        ofstrm<<"isSfm 0 invalidPath"<<endl;
    } else{
        ofstrm<<"isSfm 1"<<" "<<projectInfo->sfmResultPath<<endl;
    }
    if(!projectInfo->isDenseRec){
        ofstrm<<"isDenseRec 0 invalidPath"<<endl;
    } else{
        ofstrm<<"isDenseRec 1"<<" "<<projectInfo->denseRecResultPath<<endl;
    }
    if(!projectInfo->isSurface){
        ofstrm<<"isSurface 0 invalidPath"<<endl;
    } else{
        ofstrm<<"isSurface 1"<<" "<<projectInfo->surfacePath<<endl;
    }
    ofstrm.close();
}

//projectName mikey
//imagesNum 8
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4252.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4253.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4254.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4255.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4256.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4257.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4258.JPG
///home/wt/Projects/QTLearn/Rec/Rec/imageforder/images/IMG_4259.JPG
//isCalibration 1
//1.0 0 250 0 2.0 250 0 0 1
//isSfm 0 invalidPath
//isDenseRec 0 invalidPath
//isSurface 0 invalidPath
