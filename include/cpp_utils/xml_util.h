//
// Created by waxz on 18-11-14.
//

#ifndef LASER_LINE_EXTRACTION_XML_UTIL_H
#define LASER_LINE_EXTRACTION_XML_UTIL_H
#include <cstdio>
#include <string>
#include <map>
#include <vector>
#include <XmlRpc.h>

namespace xml_util{
    typedef std::map<std::string,double> sdmap;
    typedef std::vector< sdmap > vec_sdmap;


    std::string ToXml(sdmap mm){

        std::string structFormat = "<value><struct>%s</struct></value>";
        std::string memberFormat = "<member><name>%s</name><value><double>%f</double></value></member>";

        std::string content;
        char valueData[10000];
        for(auto it : mm){
            char tempData[10000];

            std::string key(it.first);
            double value = it.second;
            sprintf(tempData,memberFormat.c_str(),key.c_str(), value);
            std::string tmpStr = std::string(tempData);
            content += tmpStr;

        }
        std::sprintf(valueData,structFormat.c_str(), content.c_str());
        std::string res(valueData);
        return res;
    }
    std::string ToXml(vec_sdmap mm){
        std::string arrayFormat = "<value><array><data>%s</data></array></value>";
        std::string content;
        for(auto i : mm){
            content += ToXml(i);
        }
        char valueData[10000];
        std::sprintf(valueData,arrayFormat.c_str(), content.c_str());
        std::string res(valueData);
        return res;
    }
    template <class T>
    XmlRpc::XmlRpcValue createParam(T data){
        auto xml = ToXml(data);

        int offset = 0;
        XmlRpc::XmlRpcValue param;
        param.fromXml(xml,&offset);
        return param;

    }
}

#endif //LASER_LINE_EXTRACTION_XML_UTIL_H
