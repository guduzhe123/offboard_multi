//
// Created by zhouhua on 2021/2/18.
//
#include "Cinc.hpp"
#include <iostream>
#include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <sys/time.h>
#include <chrono>
#include <mutex>
#include <Eigen/Core>

#ifndef OFFBOARD_CHLOG_HPP
#define OFFBOARD_CHLOG_HPP
//---------------------
// chlog
//---------------------
// Channel log
using namespace std;
namespace chlog {
    // Level ---- 0:Disable 1:Error, 2:Warn, 3:Info, 4:Verbose, 5:Debug
    typedef enum {
        CHDIS = 0, CHERR, CHWARN, CHINFO, CHVERBO, CHDBG
    } TE_lvl;
    template<typename T>
    using Sp = std::shared_ptr<T>;

    template<typename T, typename... Args>
    inline std::shared_ptr<T> makeSp(Args &&... args) {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    extern bool initChannel(const string &sChannel);

    extern void log(const string &sChannel, TE_lvl lvl, const string &s);

    extern void setLevel(const string &sChannel, TE_lvl lvl);//设定最低输出level 低于此将不输出和写文件

    extern void setCoutLevel(const string &sChannel, TE_lvl lvl);//必须比设定的level低 越低越严重

    extern void setWorkDir(const string &sPath);//use this to group log

    extern void setEnCout(const string &sChannel, bool en);

    extern void setEnlogFile(const string &sChannel, bool en);

    extern bool runTest();

    extern void logFileErr(const string &sFile);

    extern string
    getTimeStamp(bool get_time_and_date = false, bool get_millisecond = false, bool format_for_windows = false);

    inline string my_to_string() {
        return string();
    }


    template<typename T>
    string to_string(const T &param) {
        return std::to_string(param);
    }

    template<>
    inline string to_string(const string &param) {
        return param;
    }


    template<typename T>
    inline string to_string2(const T __val, const int N_precision = 4) {
        const int __n = __gnu_cxx::__numeric_traits<float>::__max_exponent10 + 20;
        return __gnu_cxx::__to_xstring<string>(&std::vsnprintf, __n,
                                               "%.2f", __val);
    }

    template<>
    inline string to_string(const float &param) {
        return to_string2(param);
    }

    template<>
    inline string to_string(const double &param) {
        return to_string2(param);
    }

    template<size_t size>
    inline string to_string(const char (&param)[size]) {
        return string(param);
    }


    template<typename T, typename ...Types>
    inline string my_to_string(const T &first, const Types &...args) {
        return to_string(first) + my_to_string(args...);
    }

    template<typename ...Types>
    inline string my_to_string(const char *first, const Types &...args) {
        return string(first) + my_to_string(args...);
    }

    template<typename ...Types>
    inline void newline(const string &sCh, const Types &...args) {
        log(sCh, CHDIS, my_to_string(args...));
    }

    template<typename ...Types>
    inline void info(const string &sCh, const Types &...args) {
        log(sCh, CHINFO, my_to_string(args...));
    }

    template<typename ...Types>
    inline void warn(const string &sCh, const Types &...args) {
        log(sCh, CHWARN, my_to_string(args...));
    }

    template<typename ...Types>
    inline void err(const string &sCh, const Types &...args) {
        log(sCh, CHERR, my_to_string(args...));
    }

    template<typename ...Types>
    inline void verbo(const string &sCh, const Types &...args) {
        log(sCh, CHVERBO, my_to_string(args...));
    }

    template<typename ...Types>
    inline void dbg(const string &sCh, const Types &...args) {
        log(sCh, CHDBG, my_to_string(args...));
    }

    inline string toStr(const Eigen::Vector3f &v) {
        return "(" + to_string2(v.x()) + ", " + to_string2(v.y()) + ", " + to_string2(v.z()) + ")";
    };
}

#endif //OFFBOARD_CHLOG_HPP
