#include"utils.h"

QString str2qstr(const string str)
{
    return QString::fromLocal8Bit(str.data());
}

string qstr2str(const QString qstr)
{
    QByteArray cdata = qstr.toLocal8Bit();
    return string(cdata);
}

void debugstring(char* comment,string inputstring){
    QString outputQstring;
    outputQstring = str2qstr(inputstring);
    qDebug()<<comment<<" : "<< outputQstring;
}

QString showBool(bool b){
    QString yes = "YES";
    QString no = "NO";
    return b?yes:no;
}

string getImageNameFromPath(string path){
    string subname;
    for (auto i = path.end() - 1; *i != '/'; i--)
    {
      subname.insert(subname.begin(), *i);
    }
    return subname;
}


string joinStrVec(const vector<string> v, string splitor) {
  string s = "";
  if (v.size() == 0) return s;
  for (unsigned i = 0; i != v.size()  - 1; ++i) {
    s += (v[i] + splitor);
  }
  s += v[v.size() - 1];
  return s;
}
