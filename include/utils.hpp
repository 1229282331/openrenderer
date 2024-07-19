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

inline void readFileJson(const char* file)
{
	Json::Reader reader;
	Json::Value root;
 
	//从文件中读取，保证当前文件有demo.json文件  
	ifstream in(file, ios::binary);
 
	if (!in.is_open())
	{
		cout << "Error opening file\n";
		return;
	}
 
	if (reader.parse(in, root))
	{
		for(unsigned int i=0; i<root["objects"].size(); i++)
		{
			string name = root["objects"][i]["name"].asString();
			string obj_path = root["objects"][i]["obj_path"].asString();
			bool enableColor = root["objects"][i]["color"]["enable"].asBool();
			Eigen::Vector3f rgbColor;
			for(unsigned int j=0; j<root["objects"][i]["color"]["rgb"].size(); j++)
				rgbColor[j] = root["objects"][i]["color"]["rgb"][j].asFloat();
			string colorTexture_path = root["objects"][i]["textures"]["colorTexture_path"].asString();
			string normalTexture_path = root["objects"][i]["textures"]["normalTexture_path"].asString();
			Eigen::Vector3f translateVec = { root["objects"][i]["transform"]["translate"]["x"].asFloat(), 
											root["objects"][i]["transform"]["translate"]["y"].asFloat(),
											root["objects"][i]["transform"]["translate"]["z"].asFloat() };
			Eigen::Vector3f scaleVec = { root["objects"][i]["transform"]["scale"]["x"].asFloat(), 
									    root["objects"][i]["transform"]["scale"]["y"].asFloat(),
									    root["objects"][i]["transform"]["scale"]["z"].asFloat() };
			float angle = root["objects"][i]["transform"]["rotate"]["angle"].asFloat();
			Eigen::Vector3f axis = { root["objects"][i]["transform"]["rotate"]["axis"][0].asFloat(), 
									root["objects"][i]["transform"]["rotate"]["axis"][1].asFloat(),
									root["objects"][i]["transform"]["rotate"]["axis"][2].asFloat() };
			std::cout << name << '\n';
			
		}
		cout << "Reading Complete!" << endl;
	}
	else
	{
		cout << "parse error\n" << endl;
	}
 
	in.close();
}