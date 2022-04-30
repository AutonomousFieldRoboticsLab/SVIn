/**
 * File: QueryResults.cpp
 * Date: March, November 2011
 * Author: Dorian Galvez-Lopez
 * Description: structure to store results of database queries
 * License: see the LICENSE.txt file
 *
 */

#include "QueryResults.h"
#include <fstream>
#include <iostream>

namespace DBoW2 {

// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Result& ret) {
  os << "<EntryId: " << ret.Id << ", Score: " << ret.Score << ">";
  return os;
}

// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const QueryResults& ret) {
  if (ret.size() == 1)
    os << "1 result:" << std::endl;
  else
    os << ret.size() << " results:" << std::endl;

  QueryResults::const_iterator rit;
  for (rit = ret.begin(); rit != ret.end(); ++rit) {
    os << *rit;
    if (rit + 1 != ret.end()) os << std::endl;
  }
  return os;
}

// ---------------------------------------------------------------------------

void QueryResults::saveM(const std::string& filename) const {
  std::fstream f(filename.c_str(), std::ios::out);

  QueryResults::const_iterator qit;
  for (qit = begin(); qit != end(); ++qit) {
    f << qit->Id << " " << qit->Score << std::endl;
  }

  f.close();
}

// ---------------------------------------------------------------------------

}  // namespace DBoW2
