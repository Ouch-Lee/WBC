//
// Created by jun on 19-8-16.
//

#ifndef DIAMOND2D_CTRL_DATAPROCESSTOOLKIT_H
#define DIAMOND2D_CTRL_DATAPROCESSTOOLKIT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

bool readLog2Vector(const std::string & fileName, int num_Cols, int col, std::vector<double> & logVec); // Apply to '.dat'

bool writeVector2CSV(const std::vector<double>& logData, std::ofstream & outLog_csv);
bool writeArray2CSV(const double * logData, int num_LogData, std::ofstream & outLog_csv);
bool writeDouble2CSV(double foo, std::ofstream & outLog_csv);

bool writeVector2DAT(const std::vector<double> & logData, std::ofstream & outLog_dat);
bool writeArray2DAT(const double * logData, int num_LogData, std::ofstream & outLog_dat);
bool writeDouble2DAT(double foo, std::ofstream & outLog_dat);


#endif //DIAMOND2D_CTRL_DATAPROCESSTOOLKIT_H
