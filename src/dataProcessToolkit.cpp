//
// Created by jun on 19-8-16.
//

#include "dataProcessToolkit.h"


bool readLog2Vector(const std::string & fileName, int num_Cols, int col, std::vector<double> & logVec){

    if (col >= num_Cols){
        std::cout << "col must smaller than num_Cols. But, in this call, col = " << col << " , num_Cols = " << num_Cols << std::endl;
        return false;
    }

    std::string line;
    std::string a[num_Cols];

    std::stringstream line_stream;

    std::ifstream in(fileName);    // here, must utilize absolute dictionary

    while(not in.eof()){
        std::getline(in, line);
        line_stream << line;
        for (int i = 0; i != num_Cols ; ++i) {
            line_stream >> a[i];
        }

        logVec.push_back(std::stod(a[col]));

//        for (int j = 0; j != num_Cols ; ++j) {
//            std::cout << std::stod(a[j]) << " ";
//        }
//        std::cout << logVec.back() << std::endl;

        line_stream.clear();
    }
    in.close();
    logVec.pop_back();
    std::cout << logVec.size() << std::endl;

    return true;
}

bool writeVector2CSV(const std::vector<double> & logData, std::ofstream & outLog_csv){
    for (auto item : logData){
        outLog_csv << item << ",";
    }
    return true;
}

bool writeArray2CSV(const double * logData, int num_LogData, std::ofstream & outLog_csv){

    for (int i = 0; i != num_LogData; ++i) {
        outLog_csv << logData[i] << ",";
    }
    return true;
}

bool writeDouble2CSV(double foo, std::ofstream & outLog_dat){
    outLog_dat << foo << ",";
    return true;
}

bool writeVector2DAT(const std::vector<double> & logData, std::ofstream & outLog_dat){
    for (auto item : logData){
        outLog_dat << item << "\t";
    }
    return true;
}

bool writeArray2DAT(const double * logData, int num_LogData, std::ofstream & outLog_dat){

    for (int i = 0; i != num_LogData; ++i) {
        outLog_dat << logData[i] << "\t";
    }
    return true;
}

bool writeDouble2DAT(double foo, std::ofstream & outLog_dat){
    outLog_dat << foo << "\t";
    return true;
}
