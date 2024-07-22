#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include "json/json.h"

using namespace std;


inline unordered_set<int> read_txt(const char* file)
{
    unordered_set<int> res;
    fstream f;
    f.open(file);
    if(!f.is_open())
    {
        std::cerr<<"cannot open the file";
    }
    char c;
    std::string s;
    while((c = f.get())!=EOF)
    {
        if(c!=' ' && c!='\n')
            s.push_back(c);
        else
        {
            if(s.size()!=0)
                res.insert(stoi(s));
            s.clear();
        }
    }
    if(s.size()!=0)
        res.insert(stoi(s));
    return res;
}
