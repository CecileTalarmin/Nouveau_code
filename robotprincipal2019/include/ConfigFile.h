#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <string>

class ConfigFile
{
    public:
        ConfigFile(std::string path);
        virtual ~ConfigFile();

    protected:

    private:
        std::string path;
};

#endif // CONFIGFILE_H
