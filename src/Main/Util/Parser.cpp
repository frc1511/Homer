#include <Util/Parser.h>
#include <fstream>
#include <fmt/core.h>

std::string Parser::getFile(std::filesystem::path path) {
  std::ifstream file(path);
  if (!file) {
    fmt::print("Error: Could not open file \"{}\" from directory \"{}\"\n", path.string(), std::filesystem::current_path().string());
    exit(1);
  }

  std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return str;
}

std::ptrdiff_t Parser::countWhile(Iter currIter, Iter endIter, ConditionFunc condFunc) {
  std::ptrdiff_t count = 0;
  while (currIter != endIter && condFunc(*currIter)) {
    ++currIter;
    ++count;
  }
  return count;
}

std::string Parser::parseWhile(Iter& currIter, Iter endIter, ConditionFunc condFunc) {
  std::ptrdiff_t count = countWhile(currIter, endIter, condFunc);
  std::string str(currIter, currIter + count);
  currIter += count;
  return str;
}

std::ptrdiff_t Parser::countUntil(Iter currIter, Iter endIter, std::string chars) {
  return countWhile(currIter, endIter, [&chars](char c) {
    return chars.find(c) == std::string::npos;
  });
}

std::string Parser::parseUntil(Iter& currIter, Iter endIter, std::string chars) {
  std::ptrdiff_t count = countUntil(currIter, endIter, chars);
  std::string str(currIter, currIter + count);
  currIter += count;
  return str;
}

double Parser::parseNumber(Iter& currIter, Iter endIter) {
  // Handle + and -
  int sign = 1;
  if (*currIter == '-') {
    sign = -1;
    ++currIter;
  }
  else if (*currIter == '+') {
    ++currIter;
  }

  // Parse the integer part.
  std::string int_str = parseWhile(currIter, endIter, [](char c) {
    return std::isdigit(c);
  });
  
  if (currIter == endIter || *currIter != '.') {
    return std::stoi(int_str) * sign;
  }

  // Parse the decimal part.
  ++currIter;
  std::string dec_str = parseWhile(currIter, endIter, [](char c) {
    return std::isdigit(c);
  });

  std::string str = int_str + "." + dec_str;
  return std::stod(str) * sign;
}
