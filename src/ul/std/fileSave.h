/**
 ******************************************************************************
 * @Description   :
 * @author        : AN Hao
 * @Date          : 25-1-15
 * @Version       : 0.0.1
 * @File          : fileSave.h
 ******************************************************************************
 */

#ifndef UL_SRC_UL_STD_FILESAVE_H_
#define UL_SRC_UL_STD_FILESAVE_H_
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <ul/math/Vector.h>

namespace ul {
namespace std17 {
inline bool vector2csv(const ::std::vector<::ul::math::Vector>& matrix, const ::std::string& filename) {
  ::std::ofstream outputFile(filename);

  if (!outputFile.is_open()) {
    ::std::cout << "Error: Unable to open file '" << filename << "' for writing." << ::std::endl;
    return false;
  }

  // 将矩阵数据写入CSV文件
  for (int i = 0; i < matrix.size(); ++i) {
    for (int j = 0; j < matrix[0].size(); ++j) {
      outputFile << matrix[i][j];
      // 在元素间添加逗号，除了最后一个元素后面
      if (j < matrix[0].size() - 1) {
        outputFile << ",";
      }
    }
    // 每一行结束后添加换行符
    outputFile << "\n";
  }
  // 关闭文件流
  outputFile.close();
  return true;
}

inline bool getCSVData(const ::std::string& file_name, ::std::vector<::std::vector<::ul::math::Real>>& data) {
  ::std::ifstream file(file_name);
  if (file.is_open()) {
      ::std::string line;
      while (std::getline(file, line)) {
          ::std::vector<::ul::math::Real> row;
          ::std::stringstream ss(line);
          ::ul::math::Real value;
          while (ss >> value) {
              // Skip commas
              if (ss.peek() == ',') {
                  ss.ignore();
              }
              row.push_back(value);
          }
          data.push_back(row);
      }
      file.close();
  } else {
      ::std::cerr << "Unable to open file " << file_name << std::endl;
      return false;
  }
  return true;
}

}
}
#endif  // UL_SRC_UL_STD_FILESAVE_H_
