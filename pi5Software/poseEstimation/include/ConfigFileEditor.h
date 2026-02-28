#ifndef LINE_QUALITY_CONFIGREADER_H
#define LINE_QUALITY_CONFIGREADER_H

#include <string>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <filesystem>

class ConfigFileEditor {
public:
    explicit ConfigFileEditor(const std::string& dir = ".") : directory(dir){}

    const std::unordered_map<std::string, std::string>& getEntries() const {return entries;}

    void setEntries(const std::unordered_map<std::string, std::string>& pEntries) {entries = pEntries;}

    void readFile(const std::string& filename) {
        std::filesystem::path filePath = std::filesystem::path(directory) / filename;
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open config file: " << filePath << "\n";
            return;
        }

        entries.clear();
        std::string line;
        while (std::getline(file, line)) {
            parseLine(line);
        }
    }

    void writeTemp() const {
        std::filesystem::path tempFile = std::filesystem::path(directory) / "config.tmp";

        try {
            if (!tempFile.parent_path().empty())
                std::filesystem::create_directories(tempFile.parent_path());
        }
        catch (const std::exception& e) {
            std::cerr << "Error creating directories for " << tempFile
                      << ": " << e.what() << "\n";
            return;
        }

        std::ofstream out(tempFile, std::ios::out | std::ios::trunc);
        if (!out.is_open()) {
            std::cerr << "Error: Could not create " << tempFile << "\n";
            return;
        }

        for (const auto& [tag, value] : entries) {
            out << "[" << tag << "] " << value << "\n";
        }

        out.close();
    }

    bool commitTemp(const std::string& finalName = "config.txt") const {
        std::filesystem::path tempFile = std::filesystem::path(directory) / "config.tmp";
        std::filesystem::path finalFile = std::filesystem::path(directory) / finalName;

        try {
            std::filesystem::rename(tempFile, finalFile);
            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Error renaming " << tempFile << " to " << finalFile
                      << ": " << e.what() << "\n";
            return false;
        }
    }

private:
    std::unordered_map<std::string, std::string> entries;
    std::string directory;

    static bool startsWithTag(const std::string& line) {
        return !line.empty() && line.front() == '[' && line.find(']') != std::string::npos;
    }

    void parseLine(const std::string& line) {
        if (!startsWithTag(line)) return;

        size_t end = line.find(']');
        std::string tag = line.substr(1, end - 1);

        std::string value = line.substr(end + 1);
        trim(value);

        entries[tag] = value;
    }

    static void trim(std::string& s) {
        while (!s.empty() && isspace(s.front())) s.erase(s.begin());
        while (!s.empty() && isspace(s.back())) s.pop_back();
    }
};

#endif //LINE_QUALITY_CONFIGREADER_H
