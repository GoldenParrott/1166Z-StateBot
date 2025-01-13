#include "init.h"

File::File(std::string filename) {
    this->filename = filename;
}

std::string File::readFile(void) {
    std::string fname = std::string("/usd/" + this->filename + ".txt");
    FILE* file = std::fopen(fname.c_str(), "r");
    
    char output[INT_MAX];
    fread(output, 1, INT_MAX, file);

    fclose(file);

    return std::string(output);
}

void File::writeFile(std::string text) {
    std::string fname = std::string("/usd/" + this->filename + ".txt");
    FILE* file = std::fopen(fname.c_str(), "w+");

    fputs(text.c_str(), file);

    fclose(file);
}

void File::appendFile(std::string text) {
    std::string fname = std::string("/usd/" + this->filename + ".txt");
    FILE* file = std::fopen(fname.c_str(), "a");

    fputs(text.c_str(), file);

    fclose(file);
}