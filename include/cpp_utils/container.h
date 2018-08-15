//
// Created by waxz on 18-6-9.
//

#ifndef CATKIN_STARTUP_CONTAINER_H
#define CATKIN_STARTUP_CONTAINER_H

#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <valarray>
#include <string>
#include <map>
#include <cmath>
#include <XmlRpc.h>

using std::vector;
using std::valarray;
using std::map;
using std::string;
namespace container_util {
    // convert vector to valarray
    template<class T>
    valarray<T> createValarrayFromVector(vector<T> vec) {
        size_t size = vec.size();
        valarray<T> val;

        if (size > 0) {
            T *ptr = &(vec[0]);
            val = valarray<T>(ptr, size);
        }

        return val;
    }


    template<class T>
    vector<T> createVectorFromValarray(valarray<T> val) {
        size_t size = val.size();
        vector<T> vec;

        if (size > 0) {
            T *ptr = &(val[0]);
            vec = vector<T>(ptr, ptr + size);
        }

        return vec;
    }

//xmlrpc value to map and vector
    inline XmlRpc::XmlRpcValue getXmlRpcValueFromRos(ros::NodeHandle nh, string key) {
        XmlRpc::XmlRpcValue value;
        nh.getParam(key, value);
        return value;
    }

    inline vector<XmlRpc::XmlRpcValue> createVectorFromXmlRpcValue(XmlRpc::XmlRpcValue value) {
        vector<XmlRpc::XmlRpcValue> vec;
        if (value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (size_t i = 0; i < value.size(); i++) {
                vec.push_back(value[i]);
            }

        }
        return vec;
    }

    template<class T>
    map<string, T> createMapFromXmlRpcValue(XmlRpc::XmlRpcValue value) {
        map<string, T> dict;
        if (value.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (XmlRpc::XmlRpcValue::iterator it = value.begin(); it != value.end(); it++) {
                std::cout << "sss==sss:" << it->second.toXml().c_str();
//                dict[it->first] = atof(it->second.toXml().c_str());
            }

        }
        return dict;
    }


    template<class K, class V>
    bool keyExists(map<K, V> m, K key) {
        if (m.find(key) == m.end()) {
            // not found
            return false;
        } else {
            // found
            return true;
        }
    }


    // source:https://stackoverflow.com/questions/11965732/set-stdvectorint-to-a-range
    template<class OutputIterator, class Size, class Assignable>
    void iota_n(OutputIterator first, Size n, Assignable value) {
        std::generate_n(first, n, [&value]() {
            return value++;
        });
    }

    vector<int> createRangeVector(int size, int start) {
        vector<int> v;
        v.reserve(size);
        iota_n(std::back_inserter(v), size, start); // fill them with 3...16
        return v;


    }

    valarray<int> createRangeValarray(int size, int start) {
        auto vec = createRangeVector(size, start);
        valarray<int> val = createValarrayFromVector(vec);
        return val;

    }

    /*

    std::vector<int> v;                   // no default init
    v.reserve(14);                        // allocate 14 ints
    iota_n(std::back_inserter(v), 14, 3); // fill them with 3...16

    std::for_each(v.begin(), v.end(), [](int const& elem) {
        std::cout << elem << "\n";
    });

     */
}


#endif //CATKIN_STARTUP_CONTAINER_H
