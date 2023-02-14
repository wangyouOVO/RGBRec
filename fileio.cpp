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
    imagesNum = 0;
    imagePaths.clear();
    isCalitration = false;
    KMatrix.clear();
    isSfm = false;
    isDenseRec= false;
    isSurface= false;
}

void ProjectInfo::setProject(string filePath){
//TODO: get the project information from filePath
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
    qDebug()<<OpenFile;
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
    projectInfo->projectName = slist[0][1];
    projectInfo->imagesNum = atoi(slist[1][1].c_str());
    unsigned _index = 2 + unsigned(projectInfo->imagesNum);
    for(unsigned i = 2;i < _index; i++){
        projectInfo->imagePaths.push_back(slist[i][0]);
    }
    if(atoi(slist[_index][1].c_str()) == 1){
        projectInfo->isCalitration = true;
        for(auto item :slist[_index+1]){

        }
        projectInfo->KMatrix;
    }
    ifstrm.close();
    return projectInfo;
}

void FileIO::printInfoTofile(ProjectInfo* projectInfo){
    ofstrm.open(filePath);
    ofstrm << "projectName1 " << projectInfo->projectName<<std::endl;
    ofstrm.close();
}
