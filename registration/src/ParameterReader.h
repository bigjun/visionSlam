/*************************************************************************
	> File Name: ParameterReader.h

 ************************************************************************/
#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H
// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
#include <map>
#include <boost/any.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <ostream>
#include <string>
#include <ros/ros.h>


using namespace std;

// 参数读取类
class ParameterReader
{
public:
    ParameterReader( string filename="../../src/registration/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            std::cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    /*
    string getData( string key )
    {
        map<string, boost::any>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
    */


    int get(const std::string param, std::string & val) {

        if ( data.count(param)==0 ) {
           cerr << "ParameterServer object queried for invalid parameter \"%s\"" << param.c_str() <<  std::endl;
           return -1;
        }

        boost::any value = data[param];
        //std::cout << "param:  " << param << "  value: " <<  boost::any_cast<std::string>(value) << std::endl;

        std::string strValue = boost::any_cast<string>(value);
        val = strValue;

        return 0;
    }

    int get(const std::string param, int& val) {

        if ( data.count(param)==0 ) {
           cerr << "ParameterServer object queried for invalid parameter \"%s\"" << param.c_str() <<  std::endl;
           return -1;
        }

        boost::any value = data[param];
       // std::cout << "param:  " << param << "  value: " <<  boost::any_cast<std::string>(value) << std::endl;

        std::string strValue = boost::any_cast<string>(value);
        val = atof(strValue.c_str());

        return 0;
    }


    int get(const std::string param, double& val) {

        if ( data.count(param)==0 ) {
           cerr << "ParameterServer object queried for invalid parameter \"%s\"" << param.c_str() <<  std::endl;
           return -1;
        }

        boost::any value = data[param];
        //std::cout << "param:  " << param << "  value: " <<  boost::any_cast<std::string>(value) << std::endl;

        std::string strValue = boost::any_cast<string>(value);
        val = atof(strValue.c_str());

        return 0;
    }

     int get(const std::string param, float& val) {

         double  tmpVal;
         this->get (param, tmpVal);
         val = tmpVal;
     }


public:
    map<string, boost::any> data;
};




#endif
